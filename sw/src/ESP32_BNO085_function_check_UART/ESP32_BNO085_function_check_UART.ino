/**************************************************************************
  ESP32-S3 Reverse TFT Feather
  BNO085 UART-RVC + OTA + NeoPixel + WebSocket stream + CSV logging + UDP telemetry

  - HTTP (port 80): serves the web UI
  - WebSocket (port 81): real-time JSON stream + control events
  - CSV available at http://<esp-ip>/download
  - UDP (port 9001): binary telemetry unicast to the browser that connects (low jitter)
 **************************************************************************/

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO08x_RVC.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <WiFiUdp.h>
#include "secrets.h"

// ---------- Pins ----------
#define BNO_RX_PIN 38   // Feather RX header (wired to BNO085 SDA / sensor TX)
#define BNO_TX_PIN -1   // unused in RVC

// ---------- Feature toggles ----------
#define ENABLE_LOGGING 0    // 0: disable all file writes for testing
#define DIAG_TIMING    0    // 1: print timing deltas on Serial (adds jitter)
#define USE_UDP        1    // 1: enable parallel UDP telemetry (binary)

// ---------- Globals ----------
Adafruit_BNO08x_RVC rvc;
Adafruit_NeoPixel    pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

WebServer         http(80);
WebSocketsServer  ws(81);                 // ws://<ip>:81
File              logFile;
char              currentLogName[48] = "/imu_log.csv";

const uint8_t LED_BRIGHT = 16;
uint32_t lastReadMicros = 0;
uint32_t lastSendMicros = 0;
volatile uint32_t seqNo = 0;              // sequence counter for each streamed sample

// time helpers
static inline uint32_t dev_now_ms() { return millis(); }
static inline uint32_t dev_now_us() { return micros(); }

// ---------- UDP telemetry ----------
#if USE_UDP
WiFiUDP udp;
IPAddress lastClientIP;       // set by browser via WS "hello" (ip)
const uint16_t UDP_DST_PORT = 9001;  // destination port on the client
const uint16_t UDP_SRC_PORT = 9002;  // local source port we bind to

// Binary packet layout (little-endian)
#pragma pack(push, 1)
struct TelemetryPkt {
  uint32_t seq;
  uint32_t t_ms;
  uint32_t tr_us;
  uint32_t ts_us;
  float yaw, pitch, roll;
  float ax, ay, az;
};
#pragma pack(pop)
#endif

// ---------- HTML UI (served from flash) ----------
const char PAGE_INDEX[] PROGMEM = R"HTML(
<!doctype html>
<meta charset="utf-8">
<meta name=viewport content="width=device-width,initial-scale=1">
<title>IMU Stream</title>
<style>
 body{font-family:system-ui;margin:16px}
 .row{margin:8px 0} button{padding:10px 14px;margin-right:8px}
 #v{font:14px/1.35 ui-monospace,monospace;white-space:pre}
 .meta{color:#555}
 .pill{display:inline-block;padding:2px 8px;border:1px solid #ccc;border-radius:999px;margin-left:6px}
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
<div class="row meta">
  <span id=net>Syncing clock…</span>
  <span id=stats class="pill">rtt=– ms</span>
  <span id=offset class="pill">offset=– ms</span>
  <span class="pill">UDP dst port 9001</span>
</div>
<pre id=v>connecting…</pre>
<script>
const v = document.getElementById('v');
const net = document.getElementById('net');
const sRTT = document.getElementById('stats');
const sOff = document.getElementById('offset');
document.getElementById('ip').textContent = location.host;

let ws;

// Keep high-res time in ms (float), monotonic
const nowMs = () => performance.now();

// EWMA helpers
function makeEWMA(alpha, init=0, seeded=false){
  let val = init, has = seeded;
  return {
    update(x){ val = has ? (alpha*x + (1-alpha)*val) : x; has = true; return val; },
    get(){ return val; },
    seeded(){ return has; }
  };
}

const rttEwma    = makeEWMA(0.3);
const offsetEwma = makeEWMA(0.2);

// Multi-sample clock sync using Cristian's: offset = ((t0+tA)/2) - t1
async function syncClock(samples=8, timeout=500){
  return new Promise((resolve) => {
    let pending = 0;
    let results = [];
    const tryOnce = () => {
      const t0 = nowMs();
      ws.send(JSON.stringify({ type:'sync', t0 }));
      pending++;
      const to = setTimeout(()=>{
        pending--; if (pending===0 && results.length>=1) finish();
      }, timeout);

      const onMsg = (ev) => {
        let d; try{ d = JSON.parse(ev.data); } catch(e){ return; }
        if (d.type !== 'sync') return;
        ws.removeEventListener('message', onMsg);
        clearTimeout(to);
        const tA = nowMs();
        const t1 = d.t1_ms; // device millis when it processed our sync
        const rtt = tA - t0;
        const offset = ((t0 + tA) / 2) - t1; // ms
        results.push({ rtt, offset });
        pending--;
        if (results.length < samples) tryOnce(); else if (pending===0) finish();
      };
      ws.addEventListener('message', onMsg);
    };

    const finish = () => {
      results.sort((a,b)=>a.rtt-b.rtt);
      const best = results.slice(0, Math.max(1, Math.floor(results.length/2)));
      const offsets = best.map(x=>x.offset).sort((a,b)=>a-b);
      const median = offsets[Math.floor(offsets.length/2)];
      const bestRtt = best[0].rtt;
      rttEwma.update(bestRtt);
      offsetEwma.update(median);
      sRTT.textContent = `rtt=${rttEwma.get().toFixed(1)} ms`;
      sOff.textContent = `offset=${offsetEwma.get().toFixed(1)} ms`;
      net.textContent = `Clock synced (${best.length}/${results.length} used)`;
      resolve();
    };

    tryOnce();
  });
}

function openWS(){
  ws = new WebSocket('ws://' + location.hostname + ':81');

  ws.onopen = async () => {
    v.textContent = 'connected\n';
    net.textContent = 'Syncing clock…';
    // Tell the ESP our IP so it can unicast UDP to us
    try {
      // LAN-safe fallback without external fetch:
      ws.send(JSON.stringify({ type:'hello', ip: location.hostname }));
    } catch (e) {}
    await syncClock(8);
  };

  ws.onmessage = (ev) => {
    const recvAt = nowMs();
    let d;
    try { d = JSON.parse(ev.data); } catch(e) { return; }
    if (d.type === 'ack' || d.type === 'sync') return;

    // Data frame: {type:'data',seq,t,tr_us,ts_us,yaw,pitch,roll,ax,ay,az}
    const offset = offsetEwma.get();
    const estOneWayMs = recvAt - (d.t + offset);
    const rd2sd_ms    = (d.ts_us - d.tr_us) / 1000.0;

    const line = `${d.seq}\t${d.t}\t${d.yaw.toFixed(2)}\t${d.pitch.toFixed(2)}\t${d.roll.toFixed(2)}\t`+
                 `${d.ax.toFixed(3)}\t${d.ay.toFixed(3)}\t${d.az.toFixed(3)}\t`+
                 `rd_sd_ms=${rd2sd_ms.toFixed(3)}\teta_ms=${estOneWayMs.toFixed(1)}\n`;
    v.textContent += line;
    if (v.textContent.length > 4000) v.textContent = v.textContent.slice(-4000);

    // Opportunistic resync if offset drifted
    if (Math.abs(estOneWayMs) > 80) {
      syncClock(2);
    }
  };

  ws.onclose = () => {
    v.textContent += '\n(disconnected)';
    setTimeout(openWS, 1000);
  };
}
openWS();

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
  static uint16_t idx = 1;
  char buf[48];
  snprintf(buf, sizeof(buf), "/imu_log_%u.csv", idx++);
  openLog(buf);
  Serial.printf("New session: %s\n", buf);
}

// ---------- HTTP ----------
void handleRoot() {
  http.setContentLength(strlen_P(PAGE_INDEX));
  http.sendHeader("Content-Type", "text/html; charset=utf-8");
  http.send(200, "text/html", "");
  http.sendContent_P(PAGE_INDEX);
}
void handleDownload() {
  if (!LittleFS.exists(currentLogName)) { http.send(404, "text/plain", "log not found"); return; }
  File f = LittleFS.open(currentLogName, FILE_READ);
  http.streamFile(f, "text/csv");
  f.close();
}

// ---------- WebSocket ----------
void wsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  if (type == WStype_TEXT) {
    String msg((char*)payload, len);

    // Client tells us its IP for UDP unicast
    if (msg.indexOf("\"type\":\"hello\"") >= 0) {
      int k = msg.indexOf("\"ip\":\"");
      if (k >= 0) {
        int s = k + 6;
        int e = msg.indexOf("\"", s);
        if (e > s) {
          lastClientIP.fromString(msg.substring(s, e));
          Serial.print("UDP target set to "); Serial.println(lastClientIP);
        }
      }
      ws.sendTXT(num, "{\"type\":\"ack\",\"cmd\":\"hello\"}");
      return;
    }

    // Handle time sync
    if (msg.indexOf("\"type\":\"sync\"") >= 0) {
      double t0 = 0;
      int k = msg.indexOf("\"t0\":");
      if (k >= 0) {
        int s = k + 5;
        int e = msg.indexOf(",", s);
        if (e < 0) e = msg.indexOf("}", s);
        if (e > s) t0 = msg.substring(s, e).toDouble();
      }
      char js[96];
      snprintf(js, sizeof(js), "{\"type\":\"sync\",\"t0\":%.3f,\"t1_ms\":%lu}",
               t0, (unsigned long)dev_now_ms());
      ws.sendTXT(num, js);
      return;
    }

    // rotate command
    if (msg.indexOf("\"type\":\"rotate\"") >= 0) {
      rotateLog();
      ws.sendTXT(num, "{\"type\":\"ack\",\"cmd\":\"rotate\"}");
      return;
    }

    // event with optional note (append to CSV)
    String kind = "", note = "";
    int k2 = msg.indexOf("\"kind\":\""); if (k2>=0){ int s=k2+8; int e=msg.indexOf("\"",s); kind=msg.substring(s,e); }
    int n = msg.indexOf("\"note\":\""); if (n>=0){ int s=n+8; int e=msg.indexOf("\"",s); note=msg.substring(s,e); note.replace(",", " "); }
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

#if USE_UDP
  // Bind a local source port (we only send; no receive handler)
  lastClientIP = IPAddress(0,0,0,0);
  udp.begin(UDP_SRC_PORT);
  Serial.printf("UDP source port %u\n", UDP_SRC_PORT);
#endif

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
#if DIAG_TIMING
  uint32_t now = dev_now_us();
  Serial.printf("DeltaRead_us=%lu DeltaSend_us=%lu\n", now - lastReadMicros, now - lastSendMicros);
  lastReadMicros = now;
#endif

  ArduinoOTA.handle();
  http.handleClient();
  ws.loop();

  // Blink heartbeat
  static uint32_t t0=0; static bool on=true;
  if (millis()-t0>1000){ t0=millis(); on=!on; if(on) pixel.fill(pixel.Color(0,255,0)); else pixel.clear(); pixel.show(); }

  // Read IMU → stream & log
  static uint32_t lastFlush = 0;
  static uint32_t lastSend  = 0;

  BNO08x_RVC_Data d;
  if (rvc.read(&d)) {
    const uint32_t tr_us = dev_now_us();  // device time at read (us)
    const uint32_t t_ms  = dev_now_ms();  // device time at read (ms)

    // --- throttle WS to ~60 Hz (smoother, less jitter) ---
    if (t_ms - lastSend >= 16) {  // 16 ms ≈ 60 fps
      lastSend = t_ms;

      // Timestamp right before send
      const uint32_t ts_us = dev_now_us();
      const uint32_t thisSeq = ++seqNo;

      // compact JSON for WS UI
      char js[196];
      snprintf(js, sizeof(js),
        "{\"type\":\"data\",\"seq\":%lu,\"t\":%lu,\"tr_us\":%lu,\"ts_us\":%lu,"
        "\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f}",
        (unsigned long)thisSeq, (unsigned long)t_ms,
        (unsigned long)tr_us, (unsigned long)ts_us,
        d.yaw, d.pitch, d.roll, d.x_accel, d.y_accel, d.z_accel);
      ws.broadcastTXT(js);

#if USE_UDP
      // Send binary UDP packet to the last WS client that told us its IP
      if (lastClientIP != IPAddress(0,0,0,0)) {
        TelemetryPkt pkt;
        pkt.seq   = thisSeq;
        pkt.t_ms  = t_ms;
        pkt.tr_us = tr_us;
        pkt.ts_us = ts_us;
        pkt.yaw   = d.yaw;     pkt.pitch = d.pitch;   pkt.roll = d.roll;
        pkt.ax    = d.x_accel; pkt.ay    = d.y_accel; pkt.az   = d.z_accel;

        udp.beginPacket(lastClientIP, UDP_DST_PORT);
        udp.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
        udp.endPacket();
      }
#endif

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
}
