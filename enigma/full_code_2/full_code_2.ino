#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <TinyGPSPlus.h>   // <---- NEW (Library Manager: TinyGPSPlus by Mikal Hart)

// ======= Wi-Fi (STA mode) =======
const char* WIFI_SSID = "Lol";     // <-- change
const char* WIFI_PASS = "789789789"; // <-- change
WebServer server(80);

// ======= MPU6050 (register-level) =======
#define MPU_ADDR 0x68
const float GYRO_SENS = 131.0f;     // ±250 dps
const float ACC_SENS  = 16384.0f;   // ±2g
const float LPF_ALPHA = 0.15f;      // smoothing for accel

// Orientation state
float pitch = 0.0f, roll = 0.0f;    // deg
float ax_f = 0, ay_f = 0, az_f = 0; // filtered accel [g]
float lx = 0, ly = 0, lz = 0;       // linear accel [g]
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
unsigned long lastLoopMs = 0;
unsigned long lastLogMs  = 0;

// ======= Braking detection =======
const float NORMAL_BRAKE_G = -0.20f;  // normal brake threshold
const float HARSH_BRAKE_G  = -0.30f;  // harsh brake threshold
const float HARSH_MIN_MS   = 200.0f;  // must sustain this long
float brakeBelowTimerMs = 0.0f;
int   brake_event = 0;                // 0 none, 1 normal, 2 harsh

// ======= Motion state (IMU-only) =======
enum MotionState { IDLE=0, WALK=1, RIDE=2 };
MotionState currState = IDLE;

// Sliding window for |linear accel| magnitude
const int SW_N = 40; // ~1.6s @25Hz
float swBuf[SW_N]; int swIdx = 0; bool swFilled = false;
unsigned long lastSWMs = 0;

// ======= Logging (IMU + GPS safe) =======
struct Sample {
  uint32_t ms;
  float pitch, roll, ax_f, ay_f, az_f, lx, ly, lz;
  int brake_event;
  int state; // 0 idle, 1 walk, 2 ride
  // NEW: GPS fields (0 if no fix/no module)
  double gps_lat;
  double gps_lon;
  float  gps_speed_kmh;
};
const size_t MAX_SAMPLES = 900;
Sample samples[MAX_SAMPLES];
volatile size_t sampleCount = 0;

// ======= MPU utils =======
static inline void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission(true);
}
static inline void readBurst(uint8_t startReg, uint8_t count, uint8_t *buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, count, true);
  for (uint8_t i = 0; i < count; i++) buf[i] = Wire.read();
}
void mpuBegin() {
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  Wire.begin(21, 22);
  Wire.setClock(50000);
  writeReg(0x6B, 0x00);      // wake
  writeReg(0x1A, 0x03);      // DLPF ~44Hz accel / 42Hz gyro
}
void calibrateAndSeed() {
  const int N = 500;
  long gxSum=0, gySum=0, gzSum=0;
  for (int i=0; i<N; i++) {
    uint8_t d[14]; readBurst(0x3B, 14, d);
    int16_t gx = (int16_t)(d[8]  << 8 | d[9]);
    int16_t gy = (int16_t)(d[10] << 8 | d[11]);
    int16_t gz = (int16_t)(d[12] << 8 | d[13]);
    gxSum += gx; gySum += gy; gzSum += gz;
    delay(2);
  }
  gyroBiasX = gxSum / (float)N;
  gyroBiasY = gySum / (float)N;
  gyroBiasZ = gzSum / (float)N;

  // Seed pitch/roll from accel
  uint8_t d[14]; readBurst(0x3B, 14, d);
  int16_t ax = (int16_t)(d[0] << 8 | d[1]);
  int16_t ay = (int16_t)(d[2] << 8 | d[3]);
  int16_t az = (int16_t)(d[4] << 8 | d[5]);
  ax_f = ax / ACC_SENS; ay_f = ay / ACC_SENS; az_f = az / ACC_SENS;

  float pitch_acc = atan2f(ay_f, sqrtf(ax_f*ax_f + az_f*az_f)) * 180.0f / PI;
  float roll_acc  = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)) * 180.0f / PI;
  pitch = pitch_acc; roll = roll_acc;
}

// ======= GPS (SAFE-INTEGRATED) =======
HardwareSerial GPSSerial(2); // UART2
TinyGPSPlus gps;

static const int GPS_RX_PIN = 16; // ESP32 RX2  <- GPS TX
static const int GPS_TX_PIN = 17; // ESP32 TX2  -> GPS RX (optional)
static const unsigned long GPS_BAUD = 9600;

bool gpsPresent = false;     // saw any NMEA chars on serial
bool gpsHasFix  = false;     // gps.location.isValid() && gps.location.isUpdated()
unsigned long gpsBytesSeen   = 0;
unsigned long gpsStartMillis = 0;
double gpsLat = 0.0, gpsLon = 0.0;
float  gpsSpeedKmh = 0.0f;

void gpsBeginSafe() {
  gpsStartMillis = millis();
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  // No blocking wait here. We will mark presence once bytes arrive.
}

void gpsServiceNonBlocking() {
  // Read whatever is available; never block
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    gps.encode(c);
    gpsBytesSeen++;
  }
  // Presence detection: any byte within first 5s → present
  if (!gpsPresent && gpsBytesSeen > 0) gpsPresent = true;

  // Fix detection
  if (gps.location.isUpdated() && gps.location.isValid()) {
    gpsHasFix = true;
    gpsLat = gps.location.lat();
    gpsLon = gps.location.lng();
  }
  if (gps.speed.isUpdated() && gps.speed.isValid()) {
    gpsSpeedKmh = gps.speed.kmph();
  }
}

// ======= HTML =======
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Rider Telemetry — Live</title>
<style>
  body{font-family:system-ui,Arial;margin:20px}
  .row{display:flex;gap:12px;flex-wrap:wrap}
  .card{flex:1 1 180px;padding:14px 16px;border:1px solid #ddd;border-radius:12px}
  .h{font-size:16px;margin:0 0 6px}
  .v{font-size:26px;margin:0}
  .muted{color:#666}
  a.btn{display:inline-block;margin-top:10px;padding:10px 14px;border-radius:10px;border:1px solid #333;text-decoration:none;color:#333}
</style>
</head><body>
<h2>ESP32 Rider Telemetry — Live</h2>
<p class="muted">Updates ~350 ms. IMU fused orientation + linear accel. GPS is safe-integrated.</p>

<h3>Orientation</h3>
<div class="row">
  <div class="card"><div class="h">Pitch (°)</div><p id="pitch" class="v">--</p></div>
  <div class="card"><div class="h">Roll (°)</div><p id="roll" class="v">--</p></div>
</div>

<h3>Filtered Acceleration (g)</h3>
<div class="row">
  <div class="card"><div class="h">ax_f</div><p id="ax" class="v">--</p></div>
  <div class="card"><div class="h">ay_f</div><p id="ay" class="v">--</p></div>
  <div class="card"><div class="h">az_f</div><p id="az" class="v">--</p></div>
</div>

<h3>Linear Acceleration (g)</h3>
<div class="row">
  <div class="card"><div class="h">lx</div><p id="lx" class="v">--</p></div>
  <div class="card"><div class="h">ly</div><p id="ly" class="v">--</p></div>
  <div class="card"><div class="h">lz</div><p id="lz" class="v">--</p></div>
</div>

<h3>Status</h3>
<div class="row">
  <div class="card"><div class="h">Brake</div><p id="brake" class="v">--</p></div>
  <div class="card"><div class="h">State</div><p id="state" class="v">--</p></div>
</div>

<h3>GPS</h3>
<div class="row">
  <div class="card"><div class="h">GPS Status</div><p id="gps_status" class="v">--</p></div>
  <div class="card"><div class="h">Speed (km/h)</div><p id="gps_speed" class="v">--</p></div>
  <div class="card"><div class="h">Lat, Lon</div><p class="v"><span id="lat">--</span>, <span id="lon">--</span></p></div>
</div>

<p><a class="btn" href="/results">View Results / Download CSV</a></p>

<script>
  async function refresh(){
    try{
      const r = await fetch('/data'); const j = await r.json();
      pitch.textContent = j.pitch.toFixed(2);
      roll.textContent  = j.roll.toFixed(2);
      ax.textContent    = j.ax_f.toFixed(3);
      ay.textContent    = j.ay_f.toFixed(3);
      az.textContent    = j.az_f.toFixed(3);
      lx.textContent    = j.lx.toFixed(3);
      ly.textContent    = j.ly.toFixed(3);
      lz.textContent    = j.lz.toFixed(3);
      brake.textContent = j.brake_txt;
      state.textContent = j.state_txt;

      gps_status.textContent = j.gps_status;
      gps_speed.textContent  = j.gps_speed_kmh.toFixed(2);
      lat.textContent        = j.gps_lat.toFixed(6);
      lon.textContent        = j.gps_lon.toFixed(6);
    }catch(e){}
  }
  refresh(); setInterval(refresh, 350);
</script>
</body></html>
)HTML";

const char RESULTS_HTML_HEAD[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Results</title>
<style>
  body{font-family:system-ui,Arial;margin:20px}
  .stat{padding:12px;border:1px solid #ddd;border-radius:10px;margin-bottom:8px}
  a.btn{display:inline-block;padding:10px 14px;border-radius:10px;border:1px solid #333;text-decoration:none;color:#333}
</style>
</head><body>
<h2>Session Results</h2>
)HTML";
const char RESULTS_HTML_TAIL[] PROGMEM = R"HTML(
<p><a class="btn" href="/download">Download CSV</a> <a class="btn" href="/">Back to Live</a></p>
</body></html>
)HTML";

// ======= Helpers =======
static inline const char* stateToText(MotionState s){
  switch(s){ case IDLE: return "Idle"; case WALK: return "Walking"; case RIDE: return "Riding"; }
  return "-";
}
static inline const char* brakeToText(int e){
  if(e==2) return "HARSH";
  if(e==1) return "Normal";
  return "None";
}
String gpsStatusText(){
  if(!gpsPresent){
    // If no bytes seen ~5s after start, call it "No GPS Module"
    if (millis() - gpsStartMillis > 5000) return "No GPS Module";
    return "Searching for GPS...";
  }
  if(!gpsHasFix) return "Searching for GPS...";
  return "GPS Fix";
}

// ======= HTTP handlers =======
void handleIndex() { server.send_P(200, "text/html", INDEX_HTML); }

void handleData() {
  // Compose safe JSON (0s if no fix/present)
  double lat = (gpsHasFix ? gpsLat : 0.0);
  double lon = (gpsHasFix ? gpsLon : 0.0);
  float  spd = (gpsHasFix ? gpsSpeedKmh : 0.0f);

  String out = "{";
  out += "\"pitch\":" + String(pitch, 4) + ",";
  out += "\"roll\":"  + String(roll, 4)  + ",";
  out += "\"ax_f\":"  + String(ax_f, 4)  + ",";
  out += "\"ay_f\":"  + String(ay_f, 4)  + ",";
  out += "\"az_f\":"  + String(az_f, 4)  + ",";
  out += "\"lx\":"    + String(lx, 4)    + ",";
  out += "\"ly\":"    + String(ly, 4)    + ",";
  out += "\"lz\":"    + String(lz, 4)    + ",";
  out += "\"brake\":" + String(brake_event) + ",";
  out += "\"state\":" + String((int)currState) + ",";
  out += "\"brake_txt\":\"" + String(brakeToText(brake_event)) + "\",";
  out += "\"state_txt\":\"" + String(stateToText(currState)) + "\",";
  out += "\"gps_status\":\"" + gpsStatusText() + "\",";
  out += "\"gps_lat\":" + String(lat, 6) + ",";
  out += "\"gps_lon\":" + String(lon, 6) + ",";
  out += "\"gps_speed_kmh\":" + String(spd, 2);
  out += "}";
  server.send(200, "application/json", out);
}

void handleResults() {
  size_t n = sampleCount;
  float minP=1e9, maxP=-1e9, minR=1e9, maxR=-1e9;
  for (size_t i=0;i<n;i++){
    if (samples[i].pitch < minP) minP = samples[i].pitch;
    if (samples[i].pitch > maxP) maxP = samples[i].pitch;
    if (samples[i].roll  < minR) minR = samples[i].roll;
    if (samples[i].roll  > maxR) maxR = samples[i].roll;
  }
  uint32_t durMs = n ? (samples[n-1].ms - samples[0].ms) : 0;

  String html;
  html.reserve(1200);
  html += FPSTR(RESULTS_HTML_HEAD);
  html += "<div class='stat'><b>Samples:</b> " + String(n) + "</div>";
  html += "<div class='stat'><b>Duration:</b> " + String(durMs/1000.0, 2) + " s</div>";
  html += "<div class='stat'><b>Pitch min/max:</b> " + String(minP,2) + " / " + String(maxP,2) + " °</div>";
  html += "<div class='stat'><b>Roll  min/max:</b> " + String(minR,2) + " / " + String(maxR,2) + " °</div>";
  html += FPSTR(RESULTS_HTML_TAIL);
  server.send(200, "text/html", html);
}

void handleDownload() {
  // CSV with GPS columns (0s if not available)
  String csv = "ms,pitch,roll,ax_f,ay_f,az_f,lx,ly,lz,brake_event,state,gps_lat,gps_lon,gps_speed_kmh\r\n";
  size_t n = sampleCount;
  for (size_t i=0;i<n;i++){
    const Sample &s = samples[i];
    csv += String(s.ms) + ","
        +  String(s.pitch,6) + "," + String(s.roll,6) + ","
        +  String(s.ax_f,6) + "," + String(s.ay_f,6) + "," + String(s.az_f,6) + ","
        +  String(s.lx,6)   + "," + String(s.ly,6)   + "," + String(s.lz,6)   + ","
        +  String(s.brake_event) + "," + String(s.state) + ","
        +  String(s.gps_lat,6) + "," + String(s.gps_lon,6) + ","
        +  String(s.gps_speed_kmh,2) + "\r\n";
  }
  server.send(200, "text/csv", csv);
}

// ======= HTTP routes =======
void handleIndex();
void handleData();
void handleResults();
void handleDownload();

// ======= Setup / Loop =======
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Init IMU...");
  mpuBegin();
  calibrateAndSeed();

  // GPS (safe)
  Serial.println("Init GPS (non-blocking)...");
  gpsBeginSafe();

  Serial.println("Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries=0; while (WiFi.status()!=WL_CONNECTED && tries<60) { delay(200); Serial.print("."); tries++; }
  Serial.println();
  if (WiFi.status()==WL_CONNECTED) {
    Serial.print("WiFi OK. IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi failed. Rebooting in 3s..."); delay(3000); ESP.restart();
  }

  server.on("/",        HTTP_GET, handleIndex);
  server.on("/data",    HTTP_GET, handleData);
  server.on("/results", HTTP_GET, handleResults);
  server.on("/download",HTTP_GET, handleDownload);
  server.begin();

  lastLoopMs = millis();
  lastLogMs  = millis();
  lastSWMs   = millis();
  Serial.println("Server ready.");
}

void updateStateClassifier(unsigned long nowMs){
  // Downsample ~25 Hz to fill sliding window
  if (nowMs - lastSWMs >= 40) {
    lastSWMs = nowMs;
    float mag = sqrtf(lx*lx + ly*ly + lz*lz); // |linear accel| in g
    swBuf[swIdx++] = mag;
    if (swIdx >= SW_N) { swIdx = 0; swFilled = true; }

    // Compute mean & stddev over window
    int n = swFilled ? SW_N : swIdx;
    if (n >= 8) {
      float sum=0; for(int i=0;i<n;i++) sum += swBuf[i];
      float mean = sum / n;
      float var=0; for(int i=0;i<n;i++){ float d=swBuf[i]-mean; var += d*d; }
      var /= n;
      float stddev = sqrtf(var); // in g

      // Heuristic thresholds (tunable)
      if (stddev < 0.02f) currState = IDLE;
      else if (stddev < 0.08f) currState = RIDE;
      else currState = WALK;
    }
  }
}

void updateBrakeDetector(float dt){
  if (lx <= HARSH_BRAKE_G) brakeBelowTimerMs += dt * 1000.0f;
  else brakeBelowTimerMs = 0.0f;

  int newEvent = 0;
  if (lx <= HARSH_BRAKE_G && brakeBelowTimerMs >= HARSH_MIN_MS) newEvent = 2;
  else if (lx <= NORMAL_BRAKE_G) newEvent = 1;

  brake_event = newEvent;
}

void loop() {
  // ----- IMU -----
  uint8_t d[14]; readBurst(0x3B, 14, d);
  int16_t ax = (int16_t)(d[0] << 8 | d[1]);
  int16_t ay = (int16_t)(d[2] << 8 | d[3]);
  int16_t az = (int16_t)(d[4] << 8 | d[5]);
  int16_t gx = (int16_t)(d[8] << 8 | d[9]);
  int16_t gy = (int16_t)(d[10] << 8 | d[11]);

  float axg = ax / ACC_SENS;
  float ayg = ay / ACC_SENS;
  float azg = az / ACC_SENS;
  float gxdps = (gx - gyroBiasX) / GYRO_SENS;
  float gydps = (gy - gyroBiasY) / GYRO_SENS;

  unsigned long now = millis();
  float dt = (now - lastLoopMs) / 1000.0f;
  if (dt < 0.0005f) dt = 0.01f;
  lastLoopMs = now;

  float pitch_acc = atan2f(ayg, sqrtf(axg*axg + azg*azg)) * 180.0f / PI;
  float roll_acc  = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;
  float pitch_gyro = pitch + gydps * dt;
  float roll_gyro  = roll  + gxdps * dt;
  const float alpha = 0.98f;
  pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
  roll  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;

  ax_f += LPF_ALPHA * (axg - ax_f);
  ay_f += LPF_ALPHA * (ayg - ay_f);
  az_f += LPF_ALPHA * (azg - az_f);

  float φ = roll * PI / 180.0f;
  float θ = pitch * PI / 180.0f;
  float gx_g =  sinf(θ);
  float gy_g = -sinf(φ) * cosf(θ);
  float gz_g =  cosf(φ) * cosf(θ);

  lx = ax_f - gx_g;
  ly = ay_f - gy_g;
  lz = az_f - gz_g;

  updateBrakeDetector(dt);
  updateStateClassifier(now);

  // ----- GPS (non-blocking) -----
  gpsServiceNonBlocking();

  // ----- Log every ~350 ms -----
  if (now - lastLogMs >= 500) {
    lastLogMs = now;
    if (sampleCount < MAX_SAMPLES) {
      // Fallback zeros if no fix
      double lat = (gpsHasFix ? gpsLat : 0.0);
      double lon = (gpsHasFix ? gpsLon : 0.0);
      float  spd = (gpsHasFix ? gpsSpeedKmh : 0.0f);

      samples[sampleCount++] = { 
        now, pitch, roll, ax_f, ay_f, az_f, lx, ly, lz, 
        brake_event, (int)currState, 
        lat, lon, spd
      };
    }
  }

  server.handleClient();
  delay(10);
}
