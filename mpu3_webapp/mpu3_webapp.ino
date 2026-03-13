#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

// ---------- Wi-Fi (STA mode) ----------
const char* WIFI_SSID = "Lol";     // <-- change this
const char* WIFI_PASS = "789789789"; // <-- change this

WebServer server(80);

// ---------- MPU6050 raw (no extra libs) ----------
#define MPU_ADDR 0x68
// Gyro scale: ±250 dps -> 131 LSB/(°/s)
const float GYRO_SENS = 131.0f;
// Accel scale: ±2g -> 16384 LSB/g
const float ACC_SENS  = 16384.0f;

// Filter + state
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float pitch = 0.0f, roll = 0.0f;
float axg = 0, ayg = 0, azg = 0;

unsigned long lastLoopMs = 0;
unsigned long lastLogMs  = 0;

// ---------- Logging (from boot) ----------
struct Sample {
  uint32_t ms;
  float pitch, roll, ax, ay, az;
};
const size_t MAX_SAMPLES = 1500;   // ~5 min at 200ms intervals
Sample samples[MAX_SAMPLES];
volatile size_t sampleCount = 0;

// ---------- Utils ----------
static inline void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
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
  // Weak internal pull-ups (no external parts available)
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);

  Wire.begin(21, 22);
  Wire.setClock(50000);          // slower = more stable on weak pull-ups
  writeReg(0x6B, 0x00);          // wake
  writeReg(0x1A, 0x03);          // DLPF: ~44Hz accel / 42Hz gyro
}

// Quick gyro bias + seed angles
void calibrateAndSeed() {
  const int N = 500;
  long gxSum = 0, gySum = 0, gzSum = 0;

  for (int i = 0; i < N; i++) {
    uint8_t d[14];
    readBurst(0x3B, 14, d);
    int16_t gx = (int16_t)(d[8]  << 8 | d[9]);
    int16_t gy = (int16_t)(d[10] << 8 | d[11]);
    int16_t gz = (int16_t)(d[12] << 8 | d[13]);
    gxSum += gx; gySum += gy; gzSum += gz;
    delay(2);
  }
  gyroBiasX = gxSum / (float)N;
  gyroBiasY = gySum / (float)N;
  gyroBiasZ = gzSum / (float)N;

  // Seed angles from accel
  uint8_t d[14];
  readBurst(0x3B, 14, d);
  int16_t ax = (int16_t)(d[0] << 8 | d[1]);
  int16_t ay = (int16_t)(d[2] << 8 | d[3]);
  int16_t az = (int16_t)(d[4] << 8 | d[5]);

  axg = ax / ACC_SENS; ayg = ay / ACC_SENS; azg = az / ACC_SENS;

  float pitch_acc = atan2f(ayg, sqrtf(axg*axg + azg*azg)) * 180.0f / PI;
  float roll_acc  = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;
  pitch = pitch_acc; roll = roll_acc;
}

// ---------- HTTP Pages ----------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 IMU Live</title>
<style>
  body{font-family:system-ui,Arial;margin:20px}
  .card{padding:16px;border:1px solid #ddd;border-radius:12px;margin-bottom:12px}
  .h{font-size:20px;margin:0 0 8px}
  .v{font-size:28px;margin:0}
  .tag{font-size:12px;color:#666}
  .row{display:flex;gap:12px;flex-wrap:wrap}
  .col{flex:1 1 180px}
  .muted{color:#666}
  a.btn{display:inline-block;padding:10px 14px;border-radius:10px;border:1px solid #333;text-decoration:none;color:#333}
</style>
</head><body>
<h2>ESP32 IMU — Live</h2>
<p class="muted">Updates every 200 ms. Logging since boot.</p>
<div class="row">
  <div class="card col"><div class="h">Pitch (°)</div><p id="pitch" class="v">--</p></div>
  <div class="card col"><div class="h">Roll (°)</div><p id="roll" class="v">--</p></div>
</div>
<div class="row">
  <div class="card col"><div class="h">ax (g)</div><p id="ax" class="v">--</p><div class="tag">Accel X</div></div>
  <div class="card col"><div class="h">ay (g)</div><p id="ay" class="v">--</p><div class="tag">Accel Y</div></div>
  <div class="card col"><div class="h">az (g)</div><p id="az" class="v">--</p><div class="tag">Accel Z</div></div>
</div>
<p><a class="btn" href="/results">View Results / Download CSV</a></p>
<script>
  async function refresh(){
    try{
      const r = await fetch('/data');
      const j = await r.json();
      document.getElementById('pitch').textContent = j.pitch.toFixed(2);
      document.getElementById('roll').textContent  = j.roll.toFixed(2);
      document.getElementById('ax').textContent    = j.ax.toFixed(2);
      document.getElementById('ay').textContent    = j.ay.toFixed(2);
      document.getElementById('az').textContent    = j.az.toFixed(2);
    }catch(e){}
  }
  refresh();
  setInterval(refresh, 200);
</script>
</body></html>
)HTML";

const char RESULTS_HTML_HEAD[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 IMU — Results</title>
<style>
  body{font-family:system-ui,Arial;margin:20px}
  .stat{padding:12px;border:1px solid #ddd;border-radius:10px;margin-bottom:8px}
  a.btn{display:inline-block;padding:10px 14px;border-radius:10px;border:1px solid #333;text-decoration:none;color:#333}
</style>
</head><body>
<h2>Results</h2>
)HTML";

const char RESULTS_HTML_TAIL[] PROGMEM = R"HTML(
<p><a class="btn" href="/download">Download CSV</a> <a class="btn" href="/">Back to Live</a></p>
</body></html>
)HTML";

// Handlers
void handleIndex() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  // live JSON
  String out = "{";
  out += "\"pitch\":" + String(pitch, 4) + ",";
  out += "\"roll\":"  + String(roll, 4)  + ",";
  out += "\"ax\":"    + String(axg, 4)   + ",";
  out += "\"ay\":"    + String(ayg, 4)   + ",";
  out += "\"az\":"    + String(azg, 4);
  out += "}";
  server.send(200, "application/json", out);
}

void handleResults() {
  // compute simple stats over logged data
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
  html.reserve(1024);
  html += FPSTR(RESULTS_HTML_HEAD);
  html += "<div class='stat'><b>Samples:</b> " + String(n) + "</div>";
  html += "<div class='stat'><b>Duration:</b> " + String(durMs/1000.0, 2) + " s</div>";
  html += "<div class='stat'><b>Pitch min/max:</b> " + String(minP,2) + " / " + String(maxP,2) + " °</div>";
  html += "<div class='stat'><b>Roll min/max:</b> "  + String(minR,2) + " / " + String(maxR,2)  + " °</div>";
  html += FPSTR(RESULTS_HTML_TAIL);
  server.send(200, "text/html", html);
}

void handleDownload() {
  // CSV header
  String csv = "ms,pitch,roll,ax,ay,az\r\n";
  size_t n = sampleCount;
  for (size_t i=0;i<n;i++){
    csv += String(samples[i].ms);
    csv += "," + String(samples[i].pitch, 6);
    csv += "," + String(samples[i].roll,  6);
    csv += "," + String(samples[i].ax,    6);
    csv += "," + String(samples[i].ay,    6);
    csv += "," + String(samples[i].az,    6);
    csv += "\r\n";
  }
  server.send(200, "text/csv", csv);
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  // IMU
  Serial.println("Init IMU...");
  mpuBegin();
  calibrateAndSeed();

  // Wi-Fi (STA)
  Serial.println("Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 60) { // ~12s
    delay(200);
    Serial.print(".");
    tries++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed. Rebooting in 3s...");
    delay(3000);
    ESP.restart();
  }

  // HTTP routes
  server.on("/",        HTTP_GET, handleIndex);
  server.on("/data",    HTTP_GET, handleData);
  server.on("/results", HTTP_GET, handleResults);
  server.on("/download",HTTP_GET, handleDownload);
  server.begin();

  lastLoopMs = millis();
  lastLogMs  = millis();
  Serial.println("Server ready. Open the IP above in your phone browser.");
}

void loop() {
  // Read IMU @ ~100 Hz, update filter
  uint8_t d[14];
  readBurst(0x3B, 14, d);

  int16_t ax = (int16_t)(d[0] << 8 | d[1]);
  int16_t ay = (int16_t)(d[2] << 8 | d[3]);
  int16_t az = (int16_t)(d[4] << 8 | d[5]);
  int16_t gx = (int16_t)(d[8] << 8 | d[9]);
  int16_t gy = (int16_t)(d[10] << 8 | d[11]);
  // int16_t gz = (int16_t)(d[12] << 8 | d[13]); // not used for pitch/roll

  axg = ax / ACC_SENS;
  ayg = ay / ACC_SENS;
  azg = az / ACC_SENS;

  float gxdps = (gx - gyroBiasX) / GYRO_SENS;
  float gydps = (gy - gyroBiasY) / GYRO_SENS;

  unsigned long now = millis();
  float dt = (now - lastLoopMs) / 1000.0f;
  if (dt < 0.0005f) dt = 0.01f;  // guard
  lastLoopMs = now;

  float pitch_acc = atan2f(ayg, sqrtf(axg*axg + azg*azg)) * 180.0f / PI;
  float roll_acc  = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;

  float pitch_gyro = pitch + gydps * dt;
  float roll_gyro  = roll  + gxdps * dt;

  const float alpha = 0.98f;
  pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
  roll  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;

  // Log every 200 ms from boot
  if (now - lastLogMs >= 200) {
    lastLogMs = now;
    if (sampleCount < MAX_SAMPLES) {
      samples[sampleCount] = { now, pitch, roll, axg, ayg, azg };
      sampleCount++;
    }
  }

  server.handleClient();
  delay(10); // ~100 Hz loop cadence
}
