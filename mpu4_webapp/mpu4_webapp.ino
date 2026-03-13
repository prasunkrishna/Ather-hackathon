#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

// ======= Wi-Fi (STA mode) =======
const char* WIFI_SSID = "Lol";     // <-- change
const char* WIFI_PASS = "789789789"; // <-- change
WebServer server(80);

// ======= MPU6050 (register-level) =======
#define MPU_ADDR 0x68
const float GYRO_SENS = 131.0f;     // ±250 dps
const float ACC_SENS  = 16384.0f;   // ±2g
const float LPF_ALPHA = 0.15f;      // you chose 0.15

// State
float pitch = 0.0f, roll = 0.0f;    // degrees (complementary filter)
float ax_f = 0, ay_f = 0, az_f = 0; // filtered accel [g]
float lx = 0, ly = 0, lz = 0;       // linear accel [g] (gravity removed)
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
unsigned long lastLoopMs = 0;
unsigned long lastLogMs  = 0;

// ======= Logging (≈9 min at 350ms with 1500 samples) =======
struct Sample {
  uint32_t ms;
  float pitch, roll, ax_f, ay_f, az_f, lx, ly, lz;
};
const size_t MAX_SAMPLES = 1500;
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
  pinMode(21, INPUT_PULLUP); // weak pull-ups (no external resistors)
  pinMode(22, INPUT_PULLUP);
  Wire.begin(21, 22);
  Wire.setClock(50000);      // stable for your wiring
  writeReg(0x6B, 0x00);      // PWR_MGMT_1: wake
  writeReg(0x1A, 0x03);      // CONFIG: DLPF ~44Hz accel / 42Hz gyro
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
<p class="muted">Updates every 350 ms. Filtered values + linear acceleration.</p>
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
<a class="btn" href="/results">View Results / Download CSV</a>
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

// ======= HTTP handlers =======
void handleIndex() { server.send_P(200, "text/html", INDEX_HTML); }

void handleData() {
  String out = "{";
  out += "\"pitch\":" + String(pitch, 4) + ",";
  out += "\"roll\":"  + String(roll, 4)  + ",";
  out += "\"ax_f\":"  + String(ax_f, 4)  + ",";
  out += "\"ay_f\":"  + String(ay_f, 4)  + ",";
  out += "\"az_f\":"  + String(az_f, 4)  + ",";
  out += "\"lx\":"    + String(lx, 4)    + ",";
  out += "\"ly\":"    + String(ly, 4)    + ",";
  out += "\"lz\":"    + String(lz, 4);
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
  html.reserve(1024);
  html += FPSTR(RESULTS_HTML_HEAD);
  html += "<div class='stat'><b>Samples:</b> " + String(n) + "</div>";
  html += "<div class='stat'><b>Duration:</b> " + String(durMs/1000.0, 2) + " s</div>";
  html += "<div class='stat'><b>Pitch min/max:</b> " + String(minP,2) + " / " + String(maxP,2) + " °</div>";
  html += "<div class='stat'><b>Roll min/max:</b> "  + String(minR,2) + " / " + String(maxR,2) + " °</div>";
  html += FPSTR(RESULTS_HTML_TAIL);
  server.send(200, "text/html", html);
}

void handleDownload() {
  String csv = "ms,pitch,roll,ax_f,ay_f,az_f,lx,ly,lz\r\n";
  size_t n = sampleCount;
  for (size_t i=0;i<n;i++){
    const Sample &s = samples[i];
    csv += String(s.ms) + "," + String(s.pitch,6) + "," + String(s.roll,6) + ","
        +  String(s.ax_f,6) + "," + String(s.ay_f,6) + "," + String(s.az_f,6) + ","
        +  String(s.lx,6)   + "," + String(s.ly,6)   + "," + String(s.lz,6)   + "\r\n";
  }
  server.send(200, "text/csv", csv);
}

// ======= Setup / Loop =======
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Init IMU...");
  mpuBegin();
  calibrateAndSeed();

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
  Serial.println("Server ready.");
}

void loop() {
  // Read IMU registers
  uint8_t d[14]; readBurst(0x3B, 14, d);
  int16_t ax = (int16_t)(d[0] << 8 | d[1]);
  int16_t ay = (int16_t)(d[2] << 8 | d[3]);
  int16_t az = (int16_t)(d[4] << 8 | d[5]);
  int16_t gx = (int16_t)(d[8] << 8 | d[9]);
  int16_t gy = (int16_t)(d[10] << 8 | d[11]);
  // int16_t gz = (int16_t)(d[12] << 8 | d[13]);

  // Convert to g, dps
  float axg = ax / ACC_SENS;
  float ayg = ay / ACC_SENS;
  float azg = az / ACC_SENS;
  float gxdps = (gx - gyroBiasX) / GYRO_SENS;
  float gydps = (gy - gyroBiasY) / GYRO_SENS;

  // dt
  unsigned long now = millis();
  float dt = (now - lastLoopMs) / 1000.0f;
  if (dt < 0.0005f) dt = 0.01f;
  lastLoopMs = now;

  // Complementary filter for pitch/roll (deg)
  float pitch_acc = atan2f(ayg, sqrtf(axg*axg + azg*azg)) * 180.0f / PI;
  float roll_acc  = atan2f(-axg, sqrtf(ayg*ayg + azg*ayg)) * 180.0f / PI;
  float pitch_gyro = pitch + gydps * dt;
  float roll_gyro  = roll  + gxdps * dt;
  const float alpha = 0.98f;
  pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
  roll  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;

  // Low-pass filter accel (in g)
  ax_f += LPF_ALPHA * (axg - ax_f);
  ay_f += LPF_ALPHA * (ayg - ay_f);
  az_f += LPF_ALPHA * (azg - az_f);

  // ======= Gravity vector via full tilt (roll φ, pitch θ), yaw not needed for gravity =======
  // Using aerospace convention: φ=roll (deg), θ=pitch (deg)
  float φ = roll * PI / 180.0f;
  float θ = pitch * PI / 180.0f;

  // Gravity components in body frame (in g units):
  // g_x =  sin(θ)
  // g_y = -sin(φ) * cos(θ)
  // g_z =  cos(φ) * cos(θ)
  float gx_g =  sinf(θ);
  float gy_g = -sinf(φ) * cosf(θ);
  float gz_g =  cosf(φ) * cosf(θ);

  // Linear acceleration (g) = filtered accel - gravity (g)
  lx = ax_f - gx_g;
  ly = ay_f - gy_g;
  lz = az_f - gz_g;

  // Log every 350 ms
  if (now - lastLogMs >= 350) {
    lastLogMs = now;
    if (sampleCount < MAX_SAMPLES) {
      samples[sampleCount++] = { now, pitch, roll, ax_f, ay_f, az_f, lx, ly, lz };
    }
  }

  server.handleClient();
  delay(10);
}
