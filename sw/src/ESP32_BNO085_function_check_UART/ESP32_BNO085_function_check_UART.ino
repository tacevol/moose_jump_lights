/**************************************************************************
  Seeed studio XIAO ESP32C3
  BNO085 UART-RVC + OTA + WebSocket + Event-Window Logging + Live Plot + Health

  - HTTP (port 80): serves the web UI with a canvas plot and controls
  - WebSocket (port 81): real-time JSON stream (accel_mag) + control/events/health
  - Continuous logging into RAM ring buffer (~30 s of IMU history)
  - On "catch/miss/other" event:
      * Take a snapshot of the last PRE_WINDOW_MS from ring buffer
      * Keep only data from the current session (since "New Session")
      * Write that window to a new CSV file
      * Append an event row
      * Close file and resume logging
 **************************************************************************/

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_BNO08x_RVC.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include "secrets.h"
#include <Adafruit_NeoPixel.h>

// ---------- Pins ----------
#define BNO_RX_PIN    20   // XIAO ESP32C3 RX header (wired to BNO085 SDA / sensor TX)
#define BNO_TX_PIN    -1   // unused in RVC
#define LED_DATA_PIN   5   // XIAO D3 = GPIO5

// ---------- LEDs ----------
#define NUM_LEDS 2
Adafruit_NeoPixel pixels(
  NUM_LEDS,
  LED_DATA_PIN,
  NEO_GRB + NEO_KHZ800
);

// ---------- Feature toggles ----------
#define ENABLE_LOGGING    1    // 0: disable file writes, 1: enable event-window logging
#define ENABLE_LED_CONTROL 1   // 0: disable LED control, 1: enable real-time LED updates
#define LED_MODE          3    // 1: threshold, 2: gait pulse, 3: smooth brightness, 4: fast threshold
#define DIAG_TIMING       0    // 1: print timing deltas on Serial (adds jitter)
#define FORCE_FORMAT_FS   0    // 1: format LittleFS on boot ONCE to wipe old logs

// WiFi priority list: earlier entries are higher priority
struct WiFiCred {
  const char* ssid;
  const char* pass;
};

const WiFiCred WIFI_PRIORITY[] = {
  { WIFI_SSID_HOTSPOT, WIFI_PASS_HOTSPOT },  // #1: field hotspot
  { WIFI_SSID_HOME,    WIFI_PASS_HOME    },  // #2: home WiFi
};

const int WIFI_PRIORITY_COUNT = sizeof(WIFI_PRIORITY) / sizeof(WIFI_PRIORITY[0]);

// ---------- Globals ----------
Adafruit_BNO08x_RVC rvc;

WebServer         http(80);
WebSocketsServer  ws(81);                 // ws://<ip>:81
File              logFile;
char              currentLogName[48] = "";

// Sequence counter for each streamed sample
volatile uint32_t seqNo = 0;

// time helpers
static inline uint32_t dev_now_ms() { return millis(); }
static inline uint32_t dev_now_us() { return micros(); }

// ---------- Acceleration magnitude calculation ----------
// Fast, real-time calculation: sqrt(ax² + ay² + az²)
// Returns magnitude in m/s²
static inline float calcAccelMag(float ax, float ay, float az) {
  return sqrt(ax*ax + ay*ay + az*az);
}

// Alternative: squared magnitude (faster, use for thresholding)
// Use this when you only need to compare against thresholds
static inline float calcAccelMagSq(float ax, float ay, float az) {
  return ax*ax + ay*ay + az*az;
}

// ---------- LED Control Functions ----------
#if ENABLE_LED_CONTROL

// LED Mode 1: Simple threshold - LED brightness based on magnitude
void updateLEDsFromAccel(float accel_mag) {
  const float THRESHOLD_LOW  = 10.0f;   // m/s² - normal walking
  const float THRESHOLD_MED  = 20.0f;   // m/s² - running
  const float THRESHOLD_HIGH = 40.0f;   // m/s² - jumping
  
  uint8_t brightness = 0;
  uint32_t color = 0;
  
  if (accel_mag < THRESHOLD_LOW) {
    // Walking - dim green
    brightness = 32;
    color = pixels.Color(0, brightness, 0);  // Green
  } else if (accel_mag < THRESHOLD_MED) {
    // Running - medium yellow
    brightness = 128;
    color = pixels.Color(brightness, brightness, 0);  // Yellow
  } else if (accel_mag < THRESHOLD_HIGH) {
    // Fast running - bright orange
    brightness = 200;
    color = pixels.Color(brightness, brightness/2, 0);  // Orange
  } else {
    // Jumping - bright red
    brightness = 255;
    color = pixels.Color(brightness, 0, 0);  // Red
  }
  
  pixels.setPixelColor(0, color);
  pixels.setPixelColor(1, color);
  pixels.show();
}

// LED Mode 2: Pulse LED based on gait cadence
// This detects the periodic pattern in acceleration magnitude
void updateLEDsGaitPulse(float accel_mag) {
  static float last_mag = 0.0f;
  static uint32_t last_peak_ms = 0;
  static float peak_mag = 0.0f;
  
  uint32_t now = millis();
  
  // Detect peak (simple peak detection)
  if (accel_mag > last_mag && accel_mag > 15.0f) {
    // Rising edge, potential peak
    peak_mag = accel_mag;
  } else if (accel_mag < last_mag && last_mag > 15.0f) {
    // Falling edge after peak - flash LED
    uint32_t period = now - last_peak_ms;
    if (period > 200 && period < 1000) {  // 2-10 Hz cadence range
      // Flash LED on each step
      pixels.setPixelColor(0, pixels.Color(255, 255, 255));
      pixels.setPixelColor(1, pixels.Color(255, 255, 255));
      pixels.show();
      delay(50);  // Brief flash
      pixels.clear();
      pixels.show();
      
      last_peak_ms = now;
    }
  }
  
  last_mag = accel_mag;
}

// LED Mode 3: Smooth rainbow color change based on magnitude (with low-pass filter)
// Constant brightness, color transitions through full rainbow spectrum
void updateLEDsSmooth(float accel_mag) {
  static float filtered_mag = 0.0f;
  const float ALPHA = 0.1f;  // Low-pass filter coefficient (0-1, lower = smoother)
  
  // Exponential moving average filter
  filtered_mag = ALPHA * accel_mag + (1.0f - ALPHA) * filtered_mag;
  
  // Map magnitude to hue (0-65535 for ColorHSV, representing 0-360 degrees)
  // Adjust these values based on your dog's typical range
  const float MAG_MIN = 5.0f;   // Minimum expected magnitude
  const float MAG_MAX = 20.0f;  // Maximum expected magnitude
  
  const uint8_t BRIGHTNESS = 255;  // Constant brightness level (0-255)
  const uint8_t SATURATION = 255;  // Full saturation for vibrant colors
  
  // Map filtered magnitude to hue (0-65535 = 0-360 degrees)
  // Start at blue (240 degrees) and cycle through rainbow to red (0 degrees)
  uint16_t hue = 0;
  
  if (filtered_mag < MAG_MIN) {
    // Below minimum - blue/violet (start of rainbow)
    hue = 43690;  // 240 degrees (blue)
  } else if (filtered_mag >= MAG_MAX) {
    // Above maximum - red (end of rainbow)
    hue = 0;  // 0 degrees (red)
  } else {
    // Map from MAG_MIN to MAG_MAX across full rainbow spectrum
    // Blue (240°) -> Cyan -> Green -> Yellow -> Orange -> Red (0°)
    float normalized = (filtered_mag - MAG_MIN) / (MAG_MAX - MAG_MIN);  // 0.0 to 1.0
    
    // HSV hue: 0 = red, 65535 = also red (full circle)
    // We want: blue (240°) at 0.0, red (0°) at 1.0
    // So we go from 240° backwards through the spectrum
    float hue_degrees = 240.0f - (normalized * 240.0f);  // 240° down to 0°
    if (hue_degrees < 0.0f) hue_degrees += 360.0f;  // Wrap if needed
    
    hue = (uint16_t)(hue_degrees * 65535.0f / 360.0f);  // Convert to 16-bit hue
  }
  
  // Convert HSV to RGB (ColorHSV: hue 0-65535, saturation 0-255, value 0-255)
  uint32_t color = pixels.ColorHSV(hue, SATURATION, BRIGHTNESS);
  
  pixels.setPixelColor(0, color);
  pixels.setPixelColor(1, color);
  pixels.show();
}

// LED Mode 4: Use squared magnitude for faster thresholding (no sqrt)
void updateLEDsFastThreshold(float ax, float ay, float az) {
  // Calculate squared magnitude (faster - no sqrt)
  float mag_sq = calcAccelMagSq(ax, ay, az);
  
  // Compare against squared thresholds
  const float THRESHOLD_SQ_LOW  = 10.0f * 10.0f;   // 100
  const float THRESHOLD_SQ_HIGH = 30.0f * 30.0f;   // 900
  
  if (mag_sq < THRESHOLD_SQ_LOW) {
    pixels.setPixelColor(0, pixels.Color(0, 50, 0));    // Dim green
    pixels.setPixelColor(1, pixels.Color(0, 50, 0));
  } else if (mag_sq < THRESHOLD_SQ_HIGH) {
    pixels.setPixelColor(0, pixels.Color(100, 100, 0)); // Yellow
    pixels.setPixelColor(1, pixels.Color(100, 100, 0));
  } else {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));    // Red
    pixels.setPixelColor(1, pixels.Color(255, 0, 0));
  }
  pixels.show();
}

// Main LED update function - calls the appropriate mode
void updateLEDs(float accel_mag, float ax, float ay, float az) {
  #if LED_MODE == 1
    updateLEDsFromAccel(accel_mag);
  #elif LED_MODE == 2
    updateLEDsGaitPulse(accel_mag);
  #elif LED_MODE == 3
    updateLEDsSmooth(accel_mag);
  #elif LED_MODE == 4
    updateLEDsFastThreshold(ax, ay, az);
  #endif
}

#endif // ENABLE_LED_CONTROL

// ---------- Ring buffer for pre-event logging (RAM only) ----------
// We want a ~30s pre-event window at ~100 Hz.
const uint32_t PRE_WINDOW_MS = 30000;  // 30 seconds of history
const size_t   RING_CAP      = 3000;   // max samples in ring

struct ImuSample {
  uint32_t t_ms;
  float yaw, pitch, roll;
  float ax, ay, az;
  float accel_mag;  // acceleration magnitude (calculated on-the-fly)
};

ImuSample ringBuf[RING_CAP];
size_t    ringHead      = 0;   // next write index
bool      ringFilled    = false;

// Session & metrics
uint32_t sessionStartMs   = 0;   // when "New Session" was last pressed

uint32_t segSamples       = 0;   // IMU samples since session start
uint32_t segStartMs       = 0;   // time of first IMU sample in session
uint32_t imuGapCount      = 0;   // count of IMU dt > threshold
uint32_t imuGapMaxMs      = 0;   // max observed IMU dt
float    imuAvgPeriodMs   = 0.0f; // EWMA of IMU dt
uint32_t lastImuMs        = 0;   // time of last IMU sample
uint32_t eventWriteMsLast = 0;   // duration of last event segment write (ms)

void resetSegmentMetrics() {
  segSamples       = 0;
  segStartMs       = 0;
  imuGapCount      = 0;
  imuGapMaxMs      = 0;
  imuAvgPeriodMs   = 0.0f;
  lastImuMs        = 0;
  eventWriteMsLast = 0;
}

// ---------- HTML UI (served from flash) ----------
const char PAGE_INDEX[] PROGMEM = R"HTML(
<!doctype html>
<meta charset="utf-8">
<meta name=viewport content="width=device-width,initial-scale=1">
<title>IMU Plot</title>
<style>
  :root{--fg:#111;--muted:#666;--bg:#fff}
  body{font-family:system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif;background:var(--bg);color:var(--fg);margin:16px}
  h2{margin:8px 0 6px}
  .row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin:6px 0}
  button{padding:8px 12px;border:1px solid #ccc;border-radius:10px;background:#f6f6f6;cursor:pointer}
  button:active{transform:translateY(1px)}
  .pill{display:inline-block;padding:2px 8px;border:1px solid #ccc;border-radius:999px;color:#333;font-size:12px}
  #legend{display:flex;gap:10px;flex-wrap:wrap}
  .lg{display:flex;align-items:center;gap:6px;font-size:13px}
  .sw{width:16px;height:16px;border-radius:4px;border:1px solid #aaa;display:inline-block}
  .muted{color:var(--muted)}
  #canvas{display:block;width:100%;max-width:980px;height:420px;border:1px solid #eee;border-radius:12px;background:#fff}
  .chk{margin-right:10px}
  label{user-select:none}
  #healthBlock{display:flex;flex-direction:column;align-items:flex-start;gap:4px;margin:6px 0}
</style>

<h2>IMU Live Plot</h2>

<div class="row">
  <span>IP: <b id=ip></b></span>
  <span id=net class=pill>Syncing clock…</span>
  <span id=stats class=pill>rtt=– ms</span>
  <span id=offset class=pill>offset=– ms</span>
</div>

<canvas id="canvas" width="1200" height="460"></canvas>

<div class="row" id="legend">
  <!-- toggles generated by JS -->
</div>

<div class="row">
  <label class="chk"><input type="checkbox" id="autoscale"> Autoscale</label>
  <button id="pauseBtn">Pause</button>
  <a href="/download"><button>Download CSV</button></a>
  <button id="rotateBtn">New Session</button>
  <button id="catchBtn">Catch</button>
  <button id="missBtn">Miss</button>
  <button id="otherBtn">Other…</button>
</div>

<div id="healthBlock">
  <div id="healthOverall" class="pill">health: –</div>
  <div id="healthSamples" class="pill">samples: –</div>
  <div id="healthGaps" class="pill">gaps: –</div>
  <div id="healthAvg" class="pill">avg period: –</div>
  <div id="healthWrite" class="pill">last write: –</div>
</div>

<div class="row muted" style="font-size:13px">
  <div>Accel Magnitude: m/s^2 (±50 default)</div>
</div>

<script>
const ipEl = document.getElementById('ip');
const netEl = document.getElementById('net');
const rttEl = document.getElementById('stats');
const offEl = document.getElementById('offset');

const healthOverallEl = document.getElementById('healthOverall');
const healthSamplesEl = document.getElementById('healthSamples');
const healthGapsEl    = document.getElementById('healthGaps');
const healthAvgEl     = document.getElementById('healthAvg');
const healthWriteEl   = document.getElementById('healthWrite');

const cvs = document.getElementById('canvas');
const ctx = cvs.getContext('2d');
ipEl.textContent = location.host;

let ws;

// Series config: accel_mag
const series = [
  { key:'accel_mag', color:'#f4511e', enabled:true, kind:'acc' },
];

const BUFFER = 1200;     // ring buffer for plotting (host-side only)
const data = {};
series.forEach(s => data[s.key] = new Array(BUFFER).fill(null));
data.t = new Array(BUFFER).fill(null);
let head = 0;
let paused = false;

// Ranges
const FIXED = { acc:[0,50] };  // [min, max] for acceleration magnitude (always >= 0)
const autoscaleEl = document.getElementById('autoscale');

// UI legend + checkboxes
const legend = document.getElementById('legend');
series.forEach((s, i) => {
  const id = 'chk_'+s.key;
  const wrap = document.createElement('label');
  wrap.className = 'lg';
  wrap.innerHTML = `<input type="checkbox" id="${id}" ${s.enabled?'checked':''}>
                    <span class="sw" style="background:${s.color}"></span>${s.key}`;
  legend.appendChild(wrap);
  document.getElementById(id).addEventListener('change', (e)=>{
    s.enabled = e.target.checked;
  });
});

// Buttons
document.getElementById('pauseBtn').onclick = ()=>{
  paused = !paused;
  document.getElementById('pauseBtn').textContent = paused ? 'Resume' : 'Pause';
};
document.getElementById('rotateBtn').onclick = ()=> ws && ws.send(JSON.stringify({ type:'rotate' }));
document.getElementById('catchBtn').onclick  = ()=> ws && ws.send(JSON.stringify({ type:'event', kind:'catch', note:'', t: Date.now() }));
document.getElementById('missBtn').onclick   = ()=> ws && ws.send(JSON.stringify({ type:'event', kind:'miss',  note:'', t: Date.now() }));
document.getElementById('otherBtn').onclick  = ()=>{
  const note = prompt('Describe event:') || '';
  ws && ws.send(JSON.stringify({ type:'event', kind:'other', note, t: Date.now() }));
};

// Time sync (Cristian) with EWMA
const nowMs = ()=> performance.now();
function makeEWMA(a){ let v=0, has=false; return {u(x){v=has?(a*x+(1-a)*v):x;has=true;return v},g(){return v},ok(){return has}};}
const rttE = makeEWMA(0.3);
const offE = makeEWMA(0.2);

async function syncClock(samples=8, timeout=500){
  return new Promise((resolve)=>{
    let done=0, rs=[];
    function once(){
      const t0 = nowMs();
      ws.send(JSON.stringify({ type:'sync', t0 }));
      const to = setTimeout(()=>{ if(++done>=samples) finish(); else once(); }, timeout);
      const onMsg = (ev)=>{
        let d; try{ d=JSON.parse(ev.data); }catch{ return; }
        if (d.type!=='sync') return;
        ws.removeEventListener('message', onMsg);
        clearTimeout(to);
        const tA = nowMs(), t1 = d.t1_ms;
        rs.push({ rtt:(tA-t0), off:((t0+tA)/2 - t1) });
        if(++done>=samples) finish(); else once();
      };
      ws.addEventListener('message', onMsg);
    }
    function finish(){
      rs.sort((a,b)=>a.rtt-b.rtt);
      const keep = rs.slice(0, Math.max(1, Math.floor(rs.length/2)));
      const offs = keep.map(x=>x.off).sort((a,b)=>a-b);
      const med  = offs[Math.floor(offs.length/2)];
      rttE.u(keep[0].rtt); offE.u(med);
      rttEl.textContent = `rtt=${rttE.g().toFixed(1)} ms`;
      offEl.textContent = `offset=${offE.g().toFixed(1)} ms`;
      netEl.textContent = `Clock synced (${keep.length}/${rs.length})`;
      resolve();
    }
    once();
  });
}

function applyHealthColor(bg) {
  healthOverallEl.style.backgroundColor = bg;
  healthSamplesEl.style.backgroundColor = bg;
  healthGapsEl.style.backgroundColor    = bg;
  healthAvgEl.style.backgroundColor     = bg;
  healthWriteEl.style.backgroundColor   = bg;
}

function updateHealthUI(h) {
  // h: { segSamples, imuGapCount, imuGapMaxMs, imuAvgPeriodMs, eventWriteMsLast }
  const samples = h.segSamples ?? 0;
  const maxGap  = h.imuGapMaxMs ?? 0;
  const gaps    = h.imuGapCount ?? 0;
  const avg     = h.imuAvgPeriodMs ?? 0;
  const wlast   = h.eventWriteMsLast ?? 0;

  // Text per pill (numeric-heavy, as requested)
  healthOverallEl.textContent = `health: samples=${samples}, maxGap=${maxGap}ms`;
  healthSamplesEl.textContent = `samples: ${samples}`;
  healthGapsEl.textContent    = `gaps>25ms: ${gaps}, max=${maxGap}ms`;
  healthAvgEl.textContent     = `avg period: ${avg.toFixed(1)}ms`;
  healthWriteEl.textContent   = `last write: ${wlast}ms`;

  // Color code: green/yellow/red based on timing quality
  let bg = '#c8e6c9'; // green default
  if (maxGap > 120 || wlast > 2000) {
    bg = '#ffcdd2';   // red
  } else if (maxGap > 40 || wlast > 800) {
    bg = '#fff9c4';   // yellow
  }
  applyHealthColor(bg);
}

// WebSocket
function openWS() {
  ws = new WebSocket('ws://' + location.hostname + ':81');

  ws.onopen = async () => {
    netEl.textContent = 'Syncing clock…';
    await syncClock(8);
  };

  let lastSeq  = null;
  let lastDevT = null; // last device timestamp (ms)
  let lastHost = null; // last browser timestamp (ms, performance.now)

  ws.onmessage = (ev) => {
    let d;
    try {
      d = JSON.parse(ev.data);
    } catch {
      return;
    }

    if (d.type === 'health') {
      updateHealthUI(d);
      return;
    }

    if (d.type === 'ack' || d.type === 'sync') return;
    if (paused) return;

    if (d.type !== 'data') return; // ignore unknown types here

    const hostNow = performance.now();

    if (lastSeq !== null && d.seq !== undefined && d.t !== undefined) {
      const dSeq  = d.seq - lastSeq;
      const dDev  = d.t   - lastDevT;    // ms between device timestamps
      const dHost = hostNow - lastHost;  // ms between arrivals in browser

      // Host jitter check (for debugging streaming health)
      if (dHost > 150) {
        console.log(
          'BIG HOST GAP',
          dSeq, 'samples,',
          dHost.toFixed(1), 'ms',
          ev.target.url
        );
      }
      // Device-side spacing is monitored on the ESP already
    }

    lastSeq  = d.seq;
    lastDevT = d.t;
    lastHost = hostNow;

    // push into ring buffer for plotting (accel_mag)
    data.t[head]  = d.t;
    data.accel_mag[head] = ('accel_mag' in d) ? d.accel_mag : null;
    head = (head + 1) % BUFFER;
  };

  ws.onclose = () => setTimeout(openWS, 800);
}

openWS();

// Plotting
function getRange(kind){
  if (!autoscaleEl.checked) return FIXED[kind];
  // autoscale over visible buffer for enabled series of this kind
  let lo= Infinity, hi= -Infinity;
  series.forEach(s=>{
    if (s.kind!==kind || !s.enabled) return;
    const arr = data[s.key];
    for (let i=0;i<BUFFER;i++){
      const v = arr[i];
      if (v==null) continue;
      if (v<lo) lo=v; if (v>hi) hi=v;
    }
  });
  if (lo===Infinity) return FIXED[kind];
  if (lo===hi){ lo-=1; hi+=1; }
  // add 10% headroom
  const pad = (hi-lo)*0.1;
  lo = Math.max(0, lo-pad);  // Ensure lo >= 0 for magnitude
  return [lo, hi+pad];
}

function drawGrid(x, y, w, h, ylo, yhi, label){
  ctx.save();
  ctx.translate(x,y);
  ctx.strokeStyle = '#eee';
  ctx.lineWidth = 1;
  // axes
  for (let i=0;i<=5;i++){
    const yy = h*i/5;
    ctx.beginPath(); ctx.moveTo(0,yy); ctx.lineTo(w,yy); ctx.stroke();
  }
  ctx.fillStyle = '#999';
  ctx.font = '12px ui-monospace,monospace';
  ctx.fillText(`${label} [${ylo.toFixed(1)}..${yhi.toFixed(1)}]`, 8, 14);
  ctx.restore();
}

function mapY(val, ylo, yhi, y, h){
  const t = (val - ylo)/(yhi - ylo);
  return y + (1 - t)*h;
}

function drawSeries(arr, color, x, y, w, h, ylo, yhi){
  ctx.beginPath();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  // iterate in time order: head..end, then 0..head-1
  let drew=false;
  const step = w / (BUFFER-1);
  let idx = head, px = x, first = true;
  for (let i=0;i<BUFFER;i++){
    const v = arr[idx];
    const py = (v==null) ? null : mapY(v, ylo, yhi, y, h);
    if (py!=null){
      if (first){ ctx.moveTo(px, py); first=false; }
      else ctx.lineTo(px, py);
      drew = true;
    } else {
      // lift pen on gaps
      first = true;
    }
    px += step;
    idx = (idx+1) % BUFFER;
  }
  if (drew) ctx.stroke();
}

function render(){
  ctx.clearRect(0,0,cvs.width,cvs.height);

  const pad = 12;
  const w = cvs.width - pad*2;
  const h = cvs.height - pad*2;

  // Single panel: accel magnitude
  const [accLo, accHi] = getRange('acc');
  drawGrid(pad, pad, w, h, accLo, accHi, 'Accel Magnitude (m/s^2)');
  series.filter(s=>s.kind==='acc' && s.enabled).forEach(s=>{
    drawSeries(data[s.key], s.color, pad, pad, w, h, accLo, accHi);
  });

  requestAnimationFrame(render);
}
requestAnimationFrame(render);
</script>
)HTML";

// ---------- Wi-Fi / OTA ----------

// Multi-SSID connectWiFi from your setup
static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);  // we'll manage it ourselves here
  WiFi.persistent(false);        // don't write creds to flash

  const uint32_t CONNECT_TIMEOUT_MS = 8000;  // per-network timeout

  for (int idx = 0; idx < WIFI_PRIORITY_COUNT; ++idx) {
    const char* ssid = WIFI_PRIORITY[idx].ssid;
    const char* pass = WIFI_PRIORITY[idx].pass;

    if (!ssid || ssid[0] == '\0') {
      continue;  // skip empty slots
    }

    // Make sure any previous attempt is fully stopped
    WiFi.disconnect(true, true);   // drop connection and clear old config
    delay(200);

    Serial.printf("Trying WiFi SSID '%s'...\n", ssid);
    WiFi.begin(ssid, pass);

    uint32_t start = millis();
    wl_status_t st;

    while ((st = WiFi.status()) != WL_CONNECTED &&
           (millis() - start) < CONNECT_TIMEOUT_MS) {
      delay(200);
      Serial.print(".");
    }
    Serial.println();

    st = WiFi.status();
    if (st == WL_CONNECTED) {
      IPAddress ip = WiFi.localIP();
      Serial.print("WiFi connected to ");
      Serial.print(ssid);
      Serial.print(" | IP: ");
      Serial.println(ip);
      return;
    }

    Serial.printf("Failed to connect to '%s' (status=%d)\n", ssid, (int)st);
    // Loop will go on to next SSID and repeat the disconnect/begin cycle
  }

  Serial.println("WiFi: failed to connect to any configured network.");
}

static void setupOTA() {
  ArduinoOTA.setHostname(SECRET_OTA_HOSTNAME);
  if (SECRET_OTA_PASS_HASH && strlen(SECRET_OTA_PASS_HASH)==64) {
    ArduinoOTA.setPasswordHash(SECRET_OTA_PASS_HASH);
  }
  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

// ---------- Logging helpers ----------
void openLog(const char* name) {
  if (logFile) {
    logFile.close();
  }

  strncpy(currentLogName, name, sizeof(currentLogName));
  currentLogName[sizeof(currentLogName) - 1] = '\0';

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
  Serial.printf("New segment: %s\n", buf);
}

// ---------- HTTP ----------
void handleRoot() {
  http.setContentLength(strlen_P(PAGE_INDEX));
  http.sendHeader("Content-Type", "text/html; charset=utf-8");
  http.send(200, "text/html", "");
  http.sendContent_P(PAGE_INDEX);
}

void handleDownload() {
  if (!currentLogName[0] || !LittleFS.exists(currentLogName)) {
    http.send(404, "text/plain", "log not found");
    return;
  }
  File f = LittleFS.open(currentLogName, FILE_READ);
  http.streamFile(f, "text/csv");
  f.close();
}

// ---------- WebSocket ----------
void wsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  if (type == WStype_TEXT) {
    String msg((char*)payload, len);

    // time sync request
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

    // "New Session" from UI: clear history + reset metrics + (optionally) delete current log
    if (msg.indexOf("\"type\":\"rotate\"") >= 0) {
      sessionStartMs = dev_now_ms();  // mark start of new session

      // Clear RAM ring buffer
      ringHead   = 0;
      ringFilled = false;

      // Reset per-segment metrics
      resetSegmentMetrics();

#if ENABLE_LOGGING
      // Optional: delete the last CSV once you've downloaded it
      if (currentLogName[0] && LittleFS.exists(currentLogName)) {
        LittleFS.remove(currentLogName);
      }
      currentLogName[0] = '\0';
#endif

      ws.sendTXT(num, "{\"type\":\"ack\",\"cmd\":\"rotate\"}");
      return;
    }

    // event with optional note (flush pre-window from ring buffer to CSV)
    if (msg.indexOf("\"type\":\"event\"") >= 0) {
      String kind = "", note = "";
      int k2 = msg.indexOf("\"kind\":\"");
      if (k2 >= 0) {
        int s = k2 + 8;
        int e = msg.indexOf("\"", s);
        if (e > s) kind = msg.substring(s, e);
      }
      int n = msg.indexOf("\"note\":\"");
      if (n >= 0) {
        int s = n + 8;
        int e = msg.indexOf("\"", s);
        if (e > s) {
          note = msg.substring(s, e);
          note.replace(",", " ");  // keep CSV clean
        }
      }

      uint32_t t_event = dev_now_ms();
      uint32_t writeStart = t_event;

#if ENABLE_LOGGING
      // Create a new segment file for this event
      rotateLog();  // sets currentLogName and opens logFile

      if (logFile) {
        // Base: 30 s window
        const uint32_t baseStart =
          (t_event > PRE_WINDOW_MS) ? (t_event - PRE_WINDOW_MS) : 0;

        // Don't include samples from before the current session
        const uint32_t windowStart =
          (sessionStartMs > baseStart) ? sessionStartMs : baseStart;

        size_t count = ringFilled ? RING_CAP : ringHead;
        size_t idx   = ringFilled ? ringHead : 0;  // oldest

        for (size_t i = 0; i < count; ++i) {
          const ImuSample &s = ringBuf[idx];

          // Only keep samples in [windowStart, t_event]
          if (s.t_ms >= windowStart && s.t_ms <= t_event) {
            logFile.printf(
              "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,,\n",
              (unsigned long)s.t_ms,
              s.yaw, s.pitch, s.roll,
              s.ax, s.ay, s.az
              // Note: accel_mag is calculated from ax,ay,az, so we don't need to store it
            );
          }

          idx = (idx + 1) % RING_CAP;
        }

        // Append the event row itself
        logFile.printf(
          "%lu,,,,,,,%s,%s\n",
          (unsigned long)t_event,
          kind.c_str(),
          note.c_str()
        );

        logFile.flush();
        logFile.close();
      }
#endif

      uint32_t writeEnd = dev_now_ms();
      eventWriteMsLast  = writeEnd - writeStart;

      // Reset per-event metrics for next segment
      resetSegmentMetrics();
      // sessionStartMs is NOT reset here; that is only via "New Session"

      ws.sendTXT(num, "{\"type\":\"ack\",\"cmd\":\"event\"}");
    }
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.print("Reset reason: ");
  Serial.println(esp_reset_reason());
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());

  // ---------- LittleFS init / optional format ----------
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed, trying to format...");
    if (LittleFS.format()) {
      Serial.println("LittleFS format OK, remounting...");
      if (!LittleFS.begin()) {
        Serial.println("LittleFS remount failed after format!");
      }
    } else {
      Serial.println("LittleFS format FAILED!");
    }
  } else {
    Serial.println("LittleFS mounted OK.");
  }

#if FORCE_FORMAT_FS
  Serial.println("FORCE_FORMAT_FS=1 → formatting LittleFS...");
  if (LittleFS.format()) {
    Serial.println("Forced format OK, remounting LittleFS...");
    if (!LittleFS.begin()) {
      Serial.println("LittleFS remount failed after forced format!");
    }
  } else {
    Serial.println("Forced LittleFS format FAILED!");
  }
#endif

  if (LittleFS.begin()) {
    size_t total = LittleFS.totalBytes();
    size_t used  = LittleFS.usedBytes();
    size_t freeB = (total > used) ? (total - used) : 0;
    Serial.printf("LittleFS: total=%u bytes, used=%u, free=%u\n",
                  (unsigned)total, (unsigned)used, (unsigned)freeB);
  } else {
    Serial.println("LittleFS still not mounted, logging disabled.");
  }

  currentLogName[0] = '\0';
  resetSegmentMetrics();

  // Start initial session at boot
  sessionStartMs = dev_now_ms();

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      Serial.print("WiFi DISCONNECTED, reason=");
      Serial.println(info.wifi_sta_disconnected.reason);
    }
    if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      Serial.print("WiFi GOT IP: ");
      Serial.println(WiFi.localIP());
    }
  });

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

  pixels.begin();
  pixels.setBrightness(255);   // keep <120 for LiPo to prevent sag/brownout
  pixels.clear();
}

// ---------- Loop ----------
void loop() {
  // temp IP check
  static uint32_t lastIpPrint = 0;

  if (millis() - lastIpPrint > 5000) {
    lastIpPrint = millis();
    Serial.printf("IP: %s | Status: %d\n",
                  WiFi.localIP().toString().c_str(),
                  (int)WiFi.status()); // 3 = WL_CONNECTED
  }

  // temp LED check
  static uint32_t lastUpdate = 0;
  static uint8_t  hue        = 0;

  if (millis() - lastUpdate > 10) {
    lastUpdate = millis();
    hue++;

    uint32_t c1 = pixels.ColorHSV(hue * 256,        255, 255);
    uint32_t c2 = pixels.ColorHSV((hue + 85) * 256, 255, 255);

    pixels.setPixelColor(0, c1);
    pixels.setPixelColor(1, c2);
    pixels.clear(); // still just a test pattern; not calling show()
  }

#if DIAG_TIMING
  static uint32_t lastReadMicros = 0;
  static uint32_t lastSendMicros = 0;
  uint32_t now_us = dev_now_us();
  Serial.printf("DeltaRead_us=%lu DeltaSend_us=%lu\n",
                now_us - lastReadMicros, now_us - lastSendMicros);
  lastReadMicros = now_us;
#endif

  ArduinoOTA.handle();
  http.handleClient();
  ws.loop();

  // Health telemetry: send ~1 Hz
  static uint32_t lastHealthMs = 0;
  uint32_t nowMs = dev_now_ms();
  if (nowMs - lastHealthMs >= 1000) {
    lastHealthMs = nowMs;

    // health message: per-session metrics
    char hbuf[196];
    snprintf(hbuf, sizeof(hbuf),
      "{\"type\":\"health\",\"segSamples\":%lu,"
      "\"imuGapCount\":%lu,\"imuGapMaxMs\":%lu,"
      "\"imuAvgPeriodMs\":%.2f,\"eventWriteMsLast\":%lu}",
      (unsigned long)segSamples,
      (unsigned long)imuGapCount,
      (unsigned long)imuGapMaxMs,
      (double)imuAvgPeriodMs,
      (unsigned long)eventWriteMsLast
    );
    ws.broadcastTXT(hbuf);
  }

  // Read IMU → update ring buffer + stream
  static uint32_t lastSend = 0;   // WS throttle

  BNO08x_RVC_Data d;
  if (rvc.read(&d)) {
    const uint32_t tr_us = dev_now_us();   // device time at read (us)
    const uint32_t t_ms  = dev_now_ms();   // device time at read (ms)

    // IMU gap + metrics
    if (lastImuMs != 0) {                  // skip very first sample
      uint32_t dt = t_ms - lastImuMs;
      if (dt > 25) {
        Serial.printf("IMU GAP %lu ms\n", (unsigned long)dt);
        imuGapCount++;
      }
      if (dt > imuGapMaxMs) imuGapMaxMs = dt;

      float fdt = (float)dt;
      if (imuAvgPeriodMs == 0.0f) imuAvgPeriodMs = fdt;
      else                        imuAvgPeriodMs = 0.1f * fdt + 0.9f * imuAvgPeriodMs;
    } else {
      // first sample of this session
      segStartMs = t_ms;
    }
    lastImuMs = t_ms;
    segSamples++;

    // ---------- Ring buffer update (RAM only, no flash writes here) ----------
    ImuSample &s = ringBuf[ringHead];
    s.t_ms  = t_ms;
    s.yaw   = d.yaw;
    s.pitch = d.pitch;
    s.roll  = d.roll;
    s.ax    = d.x_accel;
    s.ay    = d.y_accel;
    s.az    = d.z_accel;
    s.accel_mag = calcAccelMag(d.x_accel, d.y_accel, d.z_accel);  // Calculate once, reuse below

    ringHead = (ringHead + 1) % RING_CAP;
    if (ringHead == 0) ringFilled = true;
    // ---------- end ring buffer update ----------

    // ---------- LED Control (real-time) ----------
    #if ENABLE_LED_CONTROL
      updateLEDs(s.accel_mag, d.x_accel, d.y_accel, d.z_accel);
    #endif

    // --- throttle WS to ~15 Hz (lighter streaming) ---
    if (t_ms - lastSend >= 67) {  // 67 ms ≈ 15 fps
      lastSend = t_ms;

      const uint32_t ts_us   = dev_now_us();
      const uint32_t thisSeq = ++seqNo;

      // Reuse accel_mag from ring buffer (already calculated above)
      // Slimmed JSON: accel_mag + timing
      char js[160];
      snprintf(js, sizeof(js),
        "{\"type\":\"data\",\"seq\":%lu,\"t\":%lu,\"tr_us\":%lu,\"ts_us\":%lu,"
        "\"accel_mag\":%.3f}",
        (unsigned long)thisSeq,
        (unsigned long)t_ms,
        (unsigned long)tr_us,
        (unsigned long)ts_us,
        s.accel_mag  // Reuse the value we just calculated
      );

      ws.broadcastTXT(js);
    }
  }
}
