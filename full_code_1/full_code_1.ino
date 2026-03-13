#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <TinyGPSPlus.h>

// ---------- USER CONFIG ----------
const char* WIFI_SSID     = "Lol";
const char* WIFI_PASSWORD = "789789789";

// Pins
// UART2 for GPS
static const int GPS_RX_PIN = 16; // ESP32 RX2  <- GPS TX
static const int GPS_TX_PIN = 17; // ESP32 TX2  -> GPS RX (optional)

// I2C for MPU6050
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

// Buzzer (optional, active buzzer)
static const int BUZZER_PIN = 25;

// ---------- GLOBALS ----------
HardwareSerial GPSSerial(2);
TinyGPSPlus gps;
WebServer server(80);

// Raw IMU (read via MPU6050 registers)
const uint8_t MPU_ADDR = 0x68;
float ax, ay, az, gx, gy, gz;
unsigned long lastIMUms = 0, lastGPSms = 0;

struct Telemetry {
  double lat = 0, lon = 0;
  double speedKmh = 0;
  float ax_g = 0, ay_g = 0, az_g = 0;  // in "g"
  float gx_dps = 0, gy_dps = 0, gz_dps = 0; // deg/s
  float rollDeg = 0, pitchDeg = 0;
  String mode = "Unknown";
  bool alertBrake = false;
  bool alertErratic = false;
  bool alertOverLean = false;
} T;

float lowpass(float prev, float cur, float alpha=0.2f){ return alpha*cur + (1.0f-alpha)*prev; }

// Simple rolling vars for alerts
float prevSpeedMs = 0;
unsigned long prevSpeedMsTime = 0;
float rollVarAcc = 0; int rollVarN = 0;

void beep(int ms=80){ if(BUZZER_PIN>=0){ digitalWrite(BUZZER_PIN, HIGH); delay(ms); digitalWrite(BUZZER_PIN, LOW);} }

// ---------- IMU ----------
bool mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission()==0;
}
bool mpuRead16(int16_t* vals){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if(Wire.endTransmission(false)!=0) return false;
  if(Wire.requestFrom((int)MPU_ADDR, 14)!=14) return false;
  int16_t axr = (Wire.read()<<8)|Wire.read();
  int16_t ayr = (Wire.read()<<8)|Wire.read();
  int16_t azr = (Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t gxr = (Wire.read()<<8)|Wire.read();
  int16_t gyr = (Wire.read()<<8)|Wire.read();
  int16_t gzr = (Wire.read()<<8)|Wire.read();

  // scale: accel ±2g => 16384 LSB/g; gyro ±250 dps => 131 LSB/(deg/s)
  ax = axr/16384.0f; ay = ayr/16384.0f; az = azr/16384.0f;
  gx = gxr/131.0f;   gy = gyr/131.0f;   gz = gzr/131.0f;
  return true;
}
bool mpuInit(){
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);
  // Wake up
  if(!mpuWrite(0x6B, 0x00)) return false; // PWR_MGMT_1 = 0
  // Set accel ±2g
  if(!mpuWrite(0x1C, 0x00)) return false;
  // Set gyro ±250 dps
  if(!mpuWrite(0x1B, 0x00)) return false;
  return true;
}
void computeOrientation(){
  // basic tilt from accelerometer (assuming mostly gravity)
  // roll: rotation around X, pitch: around Y
  float roll = atan2(ay, az) * 180.0 / PI;
  float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  T.rollDeg  = lowpass(T.rollDeg,  roll);
  T.pitchDeg = lowpass(T.pitchDeg, pitch);
}

// ---------- GPS ----------
void gpsInit(){
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}
void gpsUpdate(){
  while(GPSSerial.available()){
    char c = GPSSerial.read();
    gps.encode(c);
  }
  if (gps.location.isUpdated()){
    T.lat = gps.location.lat();
    T.lon = gps.location.lng();
  }
  if (gps.speed.isUpdated()){
    T.speedKmh = gps.speed.kmph();
  }
}

// ---------- Mode + Alerts ----------
void classifyAndAlerts(){
  // Walking / Scooter / Motorcycle (very simple; tune at test time)
  bool lowSpeed = T.speedKmh < 3.0;
  bool midSpeed = T.speedKmh >= 3.0 && T.speedKmh <= 35.0;
  bool hiSpeed  = T.speedKmh > 35.0;

  bool upright  = (T.pitchDeg > 70 && T.pitchDeg < 110);   // facing forward
  bool leaned   = fabs(T.rollDeg) > 20.0;

  if(lowSpeed){
    T.mode = "Walking";
  } else if(midSpeed && upright && !leaned){
    T.mode = "Scooter";
  } else if(hiSpeed || leaned){
    T.mode = "Motorcycle";
  } else {
    T.mode = "Riding";
  }

  // Harsh braking: derive accel from speed delta (m/s)
  unsigned long now = millis();
  float speedMs = (float)T.speedKmh / 3.6f;
  float a_long = 0;
  if(prevSpeedMsTime!=0){
    float dt = (now - prevSpeedMsTime)/1000.0f;
    if(dt > 0.0f) a_long = (speedMs - prevSpeedMs)/dt;
  }
  prevSpeedMs = speedMs; prevSpeedMsTime = now;

  // Simple thresholds
  T.alertBrake   = (a_long < -2.5f);                 // strong negative accel
  T.alertOverLean= (fabs(T.rollDeg) > 35.0f);        // risky lean

  // Erratic ride: rolling variance proxy using abs roll deviation
  static float lastRoll = 0;
  float droll = fabs(T.rollDeg - lastRoll);
  lastRoll = T.rollDeg;
  rollVarAcc += droll; rollVarN++;
  if(rollVarN >= 10){ // ~2 sec at 5 Hz
    float avgDev = rollVarAcc / rollVarN;
    T.alertErratic = (avgDev > 3.0f); // tune
    rollVarAcc = 0; rollVarN = 0;
  }

  // Beeps (non-blocking-ish, short)
  if(T.alertBrake || T.alertOverLean || T.alertErratic){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(30);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ---------- Web ----------
const char* INDEX_HTML = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Rider Telemetry</title>
<style>
body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;margin:12px}
.card{border:1px solid #ddd;border-radius:12px;padding:12px;margin-bottom:12px}
.row{display:flex;gap:12px;flex-wrap:wrap}
.kv{min-width:150px}
.badge{display:inline-block;padding:4px 8px;border-radius:999px;border:1px solid #ccc}
.btn{padding:8px 12px;border:1px solid #333;border-radius:8px;background:#f8f8f8;cursor:pointer}
small{color:#666}
</style>
</head>
<body>
<h2>Rider Telemetry (Local Only)</h2>
<div class="row">
  <div class="card kv"><b>Mode</b><div id="mode" class="badge">--</div></div>
  <div class="card kv"><b>Speed</b><div><span id="speed">--</span> km/h</div></div>
  <div class="card kv"><b>Roll</b><div><span id="roll">--</span>°</div></div>
  <div class="card kv"><b>Pitch</b><div><span id="pitch">--</span>°</div></div>
  <div class="card kv"><b>Accel</b><div>X:<span id="ax">--</span>g Y:<span id="ay">--</span>g Z:<span id="az">--</span>g</div></div>
  <div class="card kv"><b>GPS</b><div><span id="lat">--</span>, <span id="lon">--</span></div></div>
</div>

<div class="card">
  <b>Alerts</b>
  <div id="alerts">None</div>
  <small>Harsh brake / Over-lean / Erratic ride</small>
</div>

<div class="card">
  <b>Data Logging</b><br/>
  <button class="btn" onclick="toggleLogging()" id="logbtn">Start Logging</button>
  <button class="btn" onclick="downloadCSV()">Download CSV</button>
  <small id="loginfo"></small>
</div>

<script>
let logging=false;
let rows=[];
let lastSave=0;

function toggleLogging(){
  logging=!logging;
  document.getElementById('logbtn').innerText = logging?'Stop Logging':'Start Logging';
  if(!logging){ localStorage.setItem('ridelog', JSON.stringify(rows)); }
}

function downloadCSV(){
  const hdr = ['ts','mode','speed_kmh','roll_deg','pitch_deg','ax_g','ay_g','az_g','lat','lon','alert_brake','alert_overlean','alert_erratic'];
  const lines = [hdr.join(',')];
  for(const r of rows){ lines.push(hdr.map(k=>r[k]).join(',')); }
  const blob = new Blob([lines.join('\\n')], {type:'text/csv'});
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href=url; a.download='rider_log.csv'; a.click();
  URL.revokeObjectURL(url);
}

async function poll(){
  try{
    const r = await fetch('/data');
    const j = await r.json();

    // UI
    setText('mode', j.mode);
    setText('speed', j.speed_kmh.toFixed(1));
    setText('roll',  j.roll_deg.toFixed(1));
    setText('pitch', j.pitch_deg.toFixed(1));
    setText('ax',    j.ax_g.toFixed(2));
    setText('ay',    j.ay_g.toFixed(2));
    setText('az',    j.az_g.toFixed(2));
    setText('lat',   j.lat.toFixed(6));
    setText('lon',   j.lon.toFixed(6));

    const alerts = [];
    if(j.alert_brake)   alerts.push('Harsh Brake');
    if(j.alert_overlean)alerts.push('Over-Lean');
    if(j.alert_erratic) alerts.push('Erratic Ride');
    document.getElementById('alerts').innerText = alerts.length?alerts.join(', '):'None';

    // Logging
    if(logging){
      const row = {
        ts: Date.now(),
        mode: j.mode,
        speed_kmh: j.speed_kmh,
        roll_deg: j.roll_deg,
        pitch_deg:j.pitch_deg,
        ax_g:j.ax_g, ay_g:j.ay_g, az_g:j.az_g,
        lat:j.lat, lon:j.lon,
        alert_brake:j.alert_brake?1:0,
        alert_overlean:j.alert_overlean?1:0,
        alert_erratic:j.alert_erratic?1:0
      };
      rows.push(row);
      // Persist every ~5s in case page closes
      if(Date.now()-lastSave>5000){
        localStorage.setItem('ridelog', JSON.stringify(rows));
        lastSave=Date.now();
      }
      document.getElementById('loginfo').innerText = `Rows: ${rows.length}`;
    }
  }catch(e){ /* ignore */ }
  setTimeout(poll, 200);
}
function setText(id, t){ document.getElementById(id).innerText = t; }
poll();
</script>
</body>
</html>
)HTML";

void handleRoot(){
  server.send(200, "text/html", INDEX_HTML);
}
void handleData(){
  // Make sure we read fresh sensors before responding
  unsigned long now = millis();

  // IMU @ ~50 Hz
  if(now - lastIMUms > 20){
    if(mpuRead16(nullptr)){
      T.ax_g = ax; T.ay_g = ay; T.az_g = az;
      T.gx_dps = gx; T.gy_dps=gy; T.gz_dps=gz;
      computeOrientation();
    }
    lastIMUms = now;
  }
  // GPS update continuously in loop(); here just copy

  classifyAndAlerts();

  String json = "{";
  json += "\"lat\":" + String(T.lat, 6) + ",";
  json += "\"lon\":" + String(T.lon, 6) + ",";
  json += "\"speed_kmh\":" + String(T.speedKmh, 2) + ",";
  json += "\"ax_g\":" + String(T.ax_g, 3) + ",";
  json += "\"ay_g\":" + String(T.ay_g, 3) + ",";
  json += "\"az_g\":" + String(T.az_g, 3) + ",";
  json += "\"roll_deg\":" + String(T.rollDeg, 2) + ",";
  json += "\"pitch_deg\":" + String(T.pitchDeg, 2) + ",";
  json += "\"mode\":\"" + T.mode + "\",";
  json += "\"alert_brake\":" + String(T.alertBrake ? "true":"false") + ",";
  json += "\"alert_overlean\":" + String(T.alertOverLean ? "true":"false") + ",";
  json += "\"alert_erratic\":" + String(T.alertErratic ? "true":"false");
  json += "}";
  server.send(200, "application/json", json);
}

void setup(){
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  // IMU
  if(!mpuInit()){
    Serial.println("MPU6050 init failed");
  } else {
    Serial.println("MPU6050 ready");
  }

  // GPS
  gpsInit();
  Serial.println("GPS serial started");

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi connecting");
  while(WiFi.status()!=WL_CONNECTED){ delay(400); Serial.print("."); }
  Serial.println(); Serial.print("WiFi OK: "); Serial.println(WiFi.localIP());

  // Web
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("HTTP server started");
}

void loop(){
  // GPS bytes parsing
  gpsUpdate();
  server.handleClient();
}
