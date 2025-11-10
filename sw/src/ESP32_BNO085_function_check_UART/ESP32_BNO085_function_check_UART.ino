/**************************************************************************
  ESP32-S3 Reverse TFT Feather
  BNO085 UART-RVC + OTA + NeoPixel + WebSocket stream + CSV logging

  - Phone connects to the ESP32's IP (same Wi-Fi) → realtime stream + buttons
  - Buttons send events to ESP32 which are appended to the CSV log
  - CSV available at http://<esp-ip>/download
 **************************************************************************/

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO08x_RVC.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include "secrets.h"

// ---------- Pins ----------
#define BNO_RX_PIN 38   // Feather RX header (wired to BNO085 SDA / sensor TX)
#define BNO_TX_PIN -1   // unused in RVC

// ---------- Globals ----------
Adafruit_BNO08x_RVC rvc;
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

WebServer         http(80);
WebSocketsServer  ws(81);                 // ws://<ip>:81
File              logFile;
char              currentLogName[48] = "/imu_log.csv";

const uint8_t LED_BRIGHT = 16;

// ---------- HTML UI (served from flash) ----------
const char PAGE_INDEX[] PROGMEM = R"HTML(
<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>IMU Stream</title>
<style>
 body{font-family:system-ui;margin:16px}
 .row{margin:8px 0} button{padding:10px 14px;margin-right:8px}
 #v{font:14px/1.35 ui-monospace,monospace;white-space:pre}
</style>
<h2>IMU Stream</h2>
<div class=row>IP: <span id=ip></span></div>
<div class=row>
  <button onclick="mark('catch')">Catch</button>
  <button onclick="mark('miss')">Miss</button>
  <button onclick="other()">Other…</button>
  <a href="/download"><button>Download CSV</button></a>
  <button onclick="rotate()">New Session</button>
</div>
<pre id=v>connecting…</pre>
<script>
const v = document.getElementById('v');
document.getElementById('ip').textContent = location.host;
let ws = new WebSocket('ws://' + location.hostname + ':81');

ws.onopen = () => v.textContent = 'connected\n';
ws.onmessage = (ev) => {
  // server sends JSON lines like: {"t":123,"yaw":...,"pitch":...,"roll":...,"ax":...}
  try {
    const d = JSON.parse(ev.data);
    if (d.type === 'ack') return;
    const line = `${d.t}\t${d.yaw.toFixed(2)}\t${d.pitch.toFixed(2)}\t${d.roll.toFixed(2)}\t${d.ax.toFixed(3)}\t${d.ay.toFixed(3)}\t${d.az.toFixed(3)}\n`;
    v.textContent += line;
    if (v.textContent.length > 4000) v.textContent = v.textContent.slice(-4000);
  } catch(e) {}
};
ws.onclose = () => v.textContent += '\n(disconnected)';

function mark(kind, note=''){
  ws.send(JSON.stringify({ type:'event', kind, note, t: Date.now() }));
}
function other(){
  const note = prompt('Describe event:') || '';
  mark('other', note);
}
function rotate(){
  ws.send(JSON.stringify({ type:'rotate' }));
}
</script>
)HTML";

// ---------- Wi-Fi / OTA ----------
static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.print("\nWiFi: "); Serial.println(WiFi.localIP());
}
static void setupOTA() {
  ArduinoOTA.setHostname(SECRET_OTA_HOSTNAME);
  if (SECRET_OTA_PASS_HASH && strlen(SECRET_OTA_PASS_HASH)==64) {
    ArduinoOTA.setPasswordHash(SECRET_OTA_PASS_HASH);
  }
  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

// ---------- Logging ----------
void openLog(const char* name) {
  if (logFile) logFile.close();
  strncpy(currentLogName, name, sizeof(currentLogName));
  logFile = LittleFS.open(currentLogName, FILE_WRITE);
  if (logFile && logFile.size()==0) {
    logFile.println("t_ms,yaw,pitch,roll,ax,ay,az,event,note");
  }
}
void rotateLog() {
  // create a simple incremented name
  static uint16_t idx = 1;
  char buf[48];
  snprintf(buf, sizeof(buf), "/imu_log_%u.csv", idx++);
  openLog(buf);
  Serial.printf("New session: %s\n", buf);
}

// ---------- HTTP ----------
void handleRoot() {
  http.setContentLength(strlen_P(PAGE_INDEX));
  http.send(200, "text/html", "");
  http.sendContent_P(PAGE_INDEX);
}
void handleDownload() {
  if (!LittleFS.exists(currentLogName)) { http.send(404, "text/plain", "log not found"); return; }
  File f = LittleFS.open(currentLogName, FILE_READ);
  http.streamFile(f, "text/csv");
  f.close();
}

void wsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  if (type == WStype_TEXT) {
    // parse tiny JSON (event or rotate)
    String msg((char*)payload, len);
    if (msg.indexOf("\"type\":\"rotate\"") >= 0) {
      rotateLog();
      ws.sendTXT(num, "{\"type\":\"ack\",\"cmd\":\"rotate\"}");
      return;
    }
    // event with optional note
    // We'll be forgiving rather than fully JSON parsing here.
    String kind = "", note = "";
    int k = msg.indexOf("\"kind\":\""); if (k>=0){ int s=k+8; int e=msg.indexOf("\"",s); kind=msg.substring(s,e); }
    int n = msg.indexOf("\"note\":\""); if (n>=0){ int s=n+8; int e=msg.indexOf("\"",s); note=msg.substring(s,e); note.replace(",", " "); }
    // timestamp in ms (ESP time)
    uint32_t t = millis();
    if (logFile) logFile.printf("%lu,,,,,,,%s,%s\n", (unsigned long)t, kind.c_str(), note.c_str());
    ws.sendTXT(num, "{\"type\":\"ack\",\"cmd\":\"event\"}");
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(100);

#ifdef NEOPIXEL_POWER
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
  pixel.begin();
  pixel.setBrightness(LED_BRIGHT);
  pixel.fill(pixel.Color(0,255,0)); pixel.show();

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }
  openLog(currentLogName);     // /imu_log.csv (default)

  connectWiFi();
  setupOTA();

  // BNO085 one-wire UART
  Serial1.begin(115200, SERIAL_8N1, BNO_RX_PIN, BNO_TX_PIN);
  if (!rvc.begin(&Serial1)) Serial.println("BNO085 RVC init failed!");
  else Serial.println("BNO085 RVC ready.");

  // HTTP + WS
  http.on("/", handleRoot);
  http.on("/download", handleDownload);
  http.begin();

  ws.begin();
  ws.onEvent(wsEvent);

  Serial.println("Open http://<ESP-IP>/ in your phone browser");
  Serial.println("WebSocket port: 81");
}

// ---------- Loop ----------
void loop() {
  ArduinoOTA.handle();
  http.handleClient();
  ws.loop();

  // Blink heartbeat
  static uint32_t t0=0; static bool on=true;
  if (millis()-t0>1000){ t0=millis(); on=!on; if(on) pixel.fill(pixel.Color(0,255,0)); else pixel.clear(); pixel.show(); }

  // Read IMU → stream & log
  #define ENABLE_LOGGING 0  // set 0 to disable all file writes for testing

  static uint32_t lastFlush = 0;
  static uint32_t lastSend  = 0;

  BNO08x_RVC_Data d;
  if (rvc.read(&d)) {
    const uint32_t t_ms = millis();

    // --- throttle WS to ~60 Hz (smoother, less jitter) ---
    if (t_ms - lastSend >= 16) {  // 16 ms ≈ 60 fps
      lastSend = t_ms;

      // compact JSON to minimize airtime
      char js[160];
      snprintf(js, sizeof(js),
        "{\"t\":%lu,\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f}",
        (unsigned long)t_ms, d.yaw, d.pitch, d.roll, d.x_accel, d.y_accel, d.z_accel);
      ws.broadcastTXT(js);
    }

  #if ENABLE_LOGGING
    if (logFile) {
      logFile.printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,,\n",
        (unsigned long)t_ms, d.yaw, d.pitch, d.roll, d.x_accel, d.y_accel, d.z_accel);

      // flush once per second instead of every line
      if (t_ms - lastFlush >= 1000) {
        lastFlush = t_ms;
        logFile.flush();
      }
    }
  #endif
  }
}
