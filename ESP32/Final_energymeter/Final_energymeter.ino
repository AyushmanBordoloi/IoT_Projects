/*
  ============================================================
  ESP32 Energy Meter — PZEM-004T-100A
  Web UI Dashboard (Mobile-Optimized) — FIXED WiFi Version
  ============================================================
  Connections (ESP32 → PZEM-004T):
    ESP32 VIN  → PZEM VCC (5V)
    ESP32 GND  → PZEM GND
    ESP32 D16  → PZEM TX  (PZEM's TX → ESP's RX pin)
    ESP32 D17  → PZEM RX  (PZEM's RX → ESP's TX pin)
  ============================================================
  Libraries Required (install via Arduino Library Manager):
    - PZEM004Tv30  by Olexa Prokopenko
    - ArduinoJson  by Benoit Blanchon
    - WiFi         (built-in ESP32)
    - WebServer    (built-in ESP32)
  ============================================================
  WiFi Behaviour:
    1. Tries to connect to your home/office WiFi (STA mode).
    2. If it fails within 15 seconds, it creates its OWN
       hotspot named "EnergyMeter" (password: 12345678).
    3. In AP mode, connect your phone to "EnergyMeter" WiFi,
       then open http://192.168.4.1 in your browser.
    4. In STA mode, the IP is printed to Serial Monitor.
  ============================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <PZEM004Tv30.h>
#include <ArduinoJson.h>

const char* STA_SSID     = "Ener_Smart";  
const char* STA_PASSWORD = "12345678";  

//  Fallback Access Point (hotspot created by ESP32 itself)
const char* AP_SSID      = "EnergyMeter";
const char* AP_PASSWORD  = "12345678";

// ─── PZEM Serial ───────────────────────────────────────────
#define PZEM_RX_PIN 16   // ESP32 D16 ← PZEM TX
#define PZEM_TX_PIN 17   // ESP32 D17 → PZEM RX
HardwareSerial pzemSerial(1);
PZEM004Tv30 pzem(pzemSerial, PZEM_RX_PIN, PZEM_TX_PIN);

// ─── Web Server ────────────────────────────────────────────
WebServer server(80);

// ─── Mode flag ─────────────────────────────────────────────
bool isAPMode = false;

// ─── Readings Cache ────────────────────────────────────────
struct Readings {
  float voltage   = 0;
  float current   = 0;
  float power     = 0;
  float energy    = 0;
  float frequency = 0;
  float pf        = 0;
  bool  valid     = false;
} latest;

unsigned long lastReadMs = 0;
const unsigned long READ_INTERVAL = 1500;

// ─────────────────────────────────────────────────────────────
//  HTML PAGE
// ─────────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1"/>
<title>Energy Meter</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&family=Exo+2:wght@300;400;600;700&display=swap');

  :root {
    --bg:       #060b14;
    --panel:    #0d1526;
    --border:   #162235;
    --accent:   #00cfff;
    --green:    #00ff99;
    --warn:     #ff7043;
    --yellow:   #ffd54f;
    --text:     #dce8f5;
    --muted:    #4a6080;
    --glow-b:   0 0 18px rgba(0,207,255,.3);
    --glow-g:   0 0 18px rgba(0,255,153,.3);
    --glow-w:   0 0 18px rgba(255,112,67,.3);
  }

  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Exo 2', sans-serif;
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* Animated scanline background */
  body::before {
    content: '';
    position: fixed; inset: 0; z-index: 0;
    background:
      repeating-linear-gradient(
        0deg,
        transparent,
        transparent 2px,
        rgba(0,207,255,.015) 2px,
        rgba(0,207,255,.015) 4px
      ),
      linear-gradient(135deg, #060b14 0%, #0a1020 50%, #060b14 100%);
    pointer-events: none;
  }

  .wrapper {
    position: relative; z-index: 1;
    max-width: 460px;
    margin: 0 auto;
    padding: 14px 14px 24px;
  }

  /* ── HEADER ── */
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    background: var(--panel);
    border: 1px solid var(--border);
    border-top: 2px solid var(--accent);
    border-radius: 12px;
    padding: 12px 16px;
    margin-bottom: 14px;
    box-shadow: var(--glow-b);
  }

  .brand {
    display: flex; align-items: center; gap: 11px;
  }
  .brand-icon {
    width: 40px; height: 40px;
    background: linear-gradient(135deg, var(--accent) 0%, var(--green) 100%);
    border-radius: 10px;
    display: flex; align-items: center; justify-content: center;
    font-size: 20px;
    box-shadow: 0 0 14px rgba(0,207,255,.4);
    flex-shrink: 0;
  }
  .brand-name {
    font-family: 'Orbitron', monospace;
    font-size: 15px; font-weight: 700;
    letter-spacing: 1px;
    line-height: 1;
  }
  .brand-sub {
    font-size: 10px; color: var(--muted);
    letter-spacing: 2.5px; text-transform: uppercase;
    margin-top: 3px;
  }

  .live-badge {
    display: flex; align-items: center; gap: 6px;
    background: rgba(0,255,153,.08);
    border: 1px solid rgba(0,255,153,.25);
    border-radius: 20px;
    padding: 6px 13px;
    font-size: 11px; font-weight: 700;
    color: var(--green);
    letter-spacing: 1.5px;
    transition: all .3s;
  }
  .live-badge.err { background: rgba(255,112,67,.08); border-color: rgba(255,112,67,.3); color: var(--warn); }
  .live-dot {
    width: 7px; height: 7px; border-radius: 50%;
    background: var(--green);
  }
  .live-badge.err .live-dot { background: var(--warn); }
  .live-dot.pulse { animation: blink 1.6s ease-in-out infinite; }
  @keyframes blink {
    0%,100% { opacity:1; transform:scale(1.1); }
    50%      { opacity:.3; transform:scale(.7); }
  }

  /* AP MODE NOTICE */
  .ap-notice {
    display: none;
    background: rgba(255,213,79,.07);
    border: 1px solid rgba(255,213,79,.3);
    border-radius: 10px;
    padding: 10px 14px;
    font-size: 12px;
    color: var(--yellow);
    text-align: center;
    margin-bottom: 12px;
    letter-spacing: .5px;
    line-height: 1.5;
  }

  /* ERROR BANNER */
  .err-banner {
    display: none;
    background: rgba(255,112,67,.08);
    border: 1px solid rgba(255,112,67,.3);
    border-radius: 10px;
    padding: 10px 14px;
    font-size: 12px; color: var(--warn);
    text-align: center; margin-bottom: 12px;
  }

  /* TIMESTAMP */
  .ts {
    text-align: right;
    font-size: 10px; color: var(--muted);
    font-family: 'Orbitron', monospace;
    letter-spacing: 1px;
    margin-bottom: 12px;
  }

  /* ── HERO: VOLTAGE ── */
  .hero {
    background: var(--panel);
    border: 1px solid var(--border);
    border-top: 2px solid var(--accent);
    border-radius: 14px;
    padding: 22px 20px 18px;
    text-align: center;
    margin-bottom: 12px;
    position: relative;
    overflow: hidden;
    box-shadow: var(--glow-b);
  }
  .hero::after {
    content: '';
    position: absolute;
    top: -60px; left: 50%; transform: translateX(-50%);
    width: 220px; height: 180px;
    background: radial-gradient(ellipse, rgba(0,207,255,.1) 0%, transparent 65%);
    pointer-events: none;
  }
  .hero-lbl {
    font-size: 10px; letter-spacing: 3px;
    color: var(--accent); text-transform: uppercase;
    margin-bottom: 6px;
  }
  .hero-val {
    font-family: 'Orbitron', monospace;
    font-size: 68px; font-weight: 900; line-height: 1;
    color: #fff;
    text-shadow: 0 0 40px rgba(0,207,255,.6);
    letter-spacing: -2px;
  }
  .hero-unit {
    font-family: 'Orbitron', monospace;
    font-size: 22px; color: var(--accent);
    vertical-align: super; margin-left: 4px;
  }
  .hero-note {
    font-size: 11px; color: var(--muted);
    margin-top: 6px; letter-spacing: 1px;
  }

  /* ── CARDS GRID ── */
  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
    margin-bottom: 10px;
  }

  .card {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 13px;
    padding: 14px 14px 12px;
    position: relative;
    overflow: hidden;
    transition: box-shadow .3s, border-color .3s;
  }
  .card:active { opacity: .9; }

  .c-blue  { border-top: 2px solid var(--accent); }
  .c-green { border-top: 2px solid var(--green); }
  .c-warn  { border-top: 2px solid var(--warn); }
  .c-yellow{ border-top: 2px solid var(--yellow); }

  .c-blue:hover  { box-shadow: var(--glow-b); }
  .c-green:hover { box-shadow: var(--glow-g); }
  .c-warn:hover  { box-shadow: var(--glow-w); }

  .card-ico {
    font-size: 20px; margin-bottom: 8px; display: block;
    line-height: 1;
  }
  .card-lbl {
    font-size: 9px; letter-spacing: 2.5px;
    color: var(--muted); text-transform: uppercase;
    margin-bottom: 4px;
  }
  .card-val {
    font-family: 'Orbitron', monospace;
    font-size: 24px; font-weight: 700; color: #fff; line-height: 1;
  }
  .card-unit {
    font-size: 11px; color: var(--muted); margin-left: 3px;
  }

  /* Bar */
  .bar-bg {
    height: 3px; border-radius: 3px;
    background: rgba(255,255,255,.06);
    margin-top: 10px; overflow: hidden;
  }
  .bar-fg {
    height: 100%; border-radius: 3px;
    transition: width .7s cubic-bezier(.4,0,.2,1);
  }
  .bar-blue   { background: linear-gradient(90deg, var(--accent), #80e8ff); }
  .bar-green  { background: linear-gradient(90deg, var(--green),  #b2ffd8); }
  .bar-warn   { background: linear-gradient(90deg, var(--warn),   #ffb39a); }
  .bar-yellow { background: linear-gradient(90deg, var(--yellow), #fff59d); }

  /* ── ENERGY FULL-WIDTH ── */
  .card-wide {
    grid-column: 1 / -1;
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 10px;
  }
  .card-wide .card-val { font-size: 30px; }
  .cost-box { text-align: right; flex-shrink: 0; }
  .cost-lbl { font-size: 9px; color: var(--muted); letter-spacing: 2px; text-transform: uppercase; margin-bottom: 4px; }
  .cost-val {
    font-family: 'Orbitron', monospace;
    font-size: 20px; color: var(--green);
    text-shadow: 0 0 12px rgba(0,255,153,.4);
  }
  .cost-rate { font-size: 9px; color: var(--muted); margin-top: 2px; }

  /* ── FOOTER ── */
  footer {
    text-align: center;
    font-size: 10px; color: var(--muted);
    letter-spacing: 2px; text-transform: uppercase;
    padding-top: 10px;
  }
  footer span { color: var(--accent); }

  /* ── CORNER DECORATION ── */
  .card::before {
    content: '';
    position: absolute; top: 0; right: 0;
    width: 28px; height: 28px;
    border-top: 1px solid rgba(255,255,255,.05);
    border-right: 1px solid rgba(255,255,255,.05);
    border-radius: 0 12px 0 0;
    pointer-events: none;
  }
</style>
</head>
<body>
<div class="wrapper">

  <!-- HEADER -->
  <header>
    <div class="brand">
      <div class="brand-icon">⚡</div>
      <div>
        <div class="brand-name">ENER·METER</div>
        <div class="brand-sub">PZEM-004T · ESP32</div>
      </div>
    </div>
    <div class="live-badge" id="badge">
      <div class="live-dot pulse" id="dot"></div>
      <span id="badgeTxt">LIVE</span>
    </div>
  </header>

  <!-- AP Mode Notice (shown if on hotspot) -->
  <div class="ap-notice" id="apNote">
    📶 Connected via ESP32 Hotspot<br>
    <strong>EnergyMeter</strong> · 192.168.4.1
  </div>

  <!-- Error Banner -->
  <div class="err-banner" id="errBanner">⚠ PZEM sensor not responding — check wiring</div>

  <!-- Timestamp -->
  <div class="ts" id="ts">LAST UPDATE: --:--:--</div>

  <!-- HERO: Voltage -->
  <div class="hero">
    <div class="hero-lbl">⚡ Line Voltage</div>
    <div>
      <span class="hero-val" id="voltage">---</span><span class="hero-unit">V</span>
    </div>
    <div class="hero-note">AC RMS — Single Phase</div>
  </div>

  <!-- CARDS -->
  <div class="grid">

    <!-- Current -->
    <div class="card c-blue">
      <span class="card-ico">〰</span>
      <div class="card-lbl">Current</div>
      <div>
        <span class="card-val" id="current">---</span>
        <span class="card-unit">A</span>
      </div>
      <div class="bar-bg"><div class="bar-fg bar-blue" id="barI" style="width:0%"></div></div>
    </div>

    <!-- Power -->
    <div class="card c-green">
      <span class="card-ico">🔆</span>
      <div class="card-lbl">Active Power</div>
      <div>
        <span class="card-val" id="power">---</span>
        <span class="card-unit">W</span>
      </div>
      <div class="bar-bg"><div class="bar-fg bar-green" id="barP" style="width:0%"></div></div>
    </div>

    <!-- Frequency -->
    <div class="card c-yellow">
      <span class="card-ico">📡</span>
      <div class="card-lbl">Frequency</div>
      <div>
        <span class="card-val" id="frequency">---</span>
        <span class="card-unit">Hz</span>
      </div>
      <div class="bar-bg"><div class="bar-fg bar-yellow" id="barF" style="width:0%"></div></div>
    </div>

    <!-- Power Factor -->
    <div class="card c-warn">
      <span class="card-ico">📐</span>
      <div class="card-lbl">Power Factor</div>
      <div>
        <span class="card-val" id="pf">---</span>
      </div>
      <div class="bar-bg"><div class="bar-fg bar-warn" id="barPF" style="width:0%"></div></div>
    </div>

    <!-- Energy — full width -->
    <div class="card c-green card-wide">
      <div>
        <span class="card-ico">🔋</span>
        <div class="card-lbl">Total Energy</div>
        <div>
          <span class="card-val" id="energy">---</span>
          <span class="card-unit">kWh</span>
        </div>
      </div>
      <div class="cost-box">
        <div class="cost-lbl">Est. Cost</div>
        <div class="cost-val" id="cost">₹ --</div>
        <div class="cost-rate">@ ₹8 / kWh</div>
      </div>
    </div>

  </div><!-- /grid -->

  <footer>ESP32 IoT · <span>Energy Monitor</span> · v2.0</footer>

</div>

<script>
const RATE = 8;
const p2  = n => String(n).padStart(2,'0');
const fmt = (v, d=2) => (v == null || isNaN(v)) ? '---' : (+v).toFixed(d);
const bar = (id, pct) => {
  const el = document.getElementById(id);
  if(el) el.style.width = Math.min(100, Math.max(0, pct)) + '%';
};

// Detect if we're on the ESP32 AP (192.168.4.x)
if(location.hostname === '192.168.4.1'){
  document.getElementById('apNote').style.display = 'block';
}

async function refresh(){
  try {
    const res = await fetch('/data', {cache:'no-store'});
    if(!res.ok) throw new Error('HTTP ' + res.status);
    const d = await res.json();

    const badge  = document.getElementById('badge');
    const dot    = document.getElementById('dot');
    const errB   = document.getElementById('errBanner');

    if(!d.valid){
      badge.classList.add('err');
      document.getElementById('badgeTxt').textContent = 'ERROR';
      dot.classList.remove('pulse');
      errB.style.display = 'block';
    } else {
      badge.classList.remove('err');
      document.getElementById('badgeTxt').textContent = 'LIVE';
      dot.classList.add('pulse');
      errB.style.display = 'none';
    }

    document.getElementById('voltage').textContent   = fmt(d.voltage, 1);
    document.getElementById('current').textContent   = fmt(d.current, 2);
    document.getElementById('power').textContent     = fmt(d.power, 1);
    document.getElementById('frequency').textContent = fmt(d.frequency, 1);
    document.getElementById('pf').textContent        = fmt(d.pf, 3);
    document.getElementById('energy').textContent    = fmt(d.energy, 3);

    const c = (d.energy * RATE);
    document.getElementById('cost').textContent = '₹ ' + (isNaN(c) ? '--' : c.toFixed(2));

    bar('barI',  (d.current   / 100)   * 100);
    bar('barP',  (d.power     / 22000) * 100);
    bar('barF',  ((d.frequency - 45) / 15) * 100);  // 45–60 Hz range
    bar('barPF', d.pf * 100);

    const n = new Date();
    document.getElementById('ts').textContent =
      'LAST UPDATE: ' + p2(n.getHours()) + ':' + p2(n.getMinutes()) + ':' + p2(n.getSeconds());

  } catch(e) {
    const badge = document.getElementById('badge');
    badge.classList.add('err');
    document.getElementById('badgeTxt').textContent = 'OFFLINE';
    document.getElementById('dot').classList.remove('pulse');
    document.getElementById('errBanner').style.display = 'block';
  }
}

refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>
)rawhtml";

// ─────────────────────────────────────────────────────────────
//  ROUTE HANDLERS
// ─────────────────────────────────────────────────────────────
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  StaticJsonDocument<256> doc;

  if (latest.valid) {
    doc["voltage"]   = latest.voltage;
    doc["current"]   = latest.current;
    doc["power"]     = latest.power;
    doc["energy"]    = latest.energy;
    doc["frequency"] = latest.frequency;
    doc["pf"]        = latest.pf;
  } else {
    doc["voltage"]   = nullptr;
    doc["current"]   = nullptr;
    doc["power"]     = nullptr;
    doc["energy"]    = nullptr;
    doc["frequency"] = nullptr;
    doc["pf"]        = nullptr;
  }
  doc["valid"] = latest.valid;

  String json;
  serializeJson(doc, json);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", json);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32 ENERGY METER v2.0 ===");

  // ── Step 1: Try connecting to router ──────────────────────
  Serial.println("[WiFi] Trying STA mode: " + String(STA_SSID));
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASSWORD);

  unsigned long t0 = millis();
  bool connected = false;

  while (millis() - t0 < 15000) {          // wait up to 15 seconds
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      break;
    }
    Serial.print('.');
    delay(500);
  }
  Serial.println();

  // ── Step 2: Fallback to AP mode if STA failed ─────────────
  if (!connected) {
    Serial.println("[WiFi] STA failed — switching to AP (hotspot) mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    isAPMode = true;

    IPAddress apIP = WiFi.softAPIP();
    Serial.println("[WiFi] AP started!");
    Serial.println("[WiFi] SSID    : " + String(AP_SSID));
    Serial.println("[WiFi] Password: " + String(AP_PASSWORD));
    Serial.println("[WiFi] Open on phone: http://" + apIP.toString());
  } else {
    Serial.println("[WiFi] Connected to " + String(STA_SSID));
    Serial.println("[WiFi] IP: " + WiFi.localIP().toString());
    Serial.println("[WiFi] Open on phone: http://" + WiFi.localIP().toString());
  }

  // ── Step 3: Start web server ───────────────────────────────
  server.on("/",      handleRoot);
  server.on("/data",  handleData);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[Server] HTTP server started on port 80");
  Serial.println("=================================\n");
}

// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  // ── Auto-reconnect if STA drops ───────────────────────────
  if (!isAPMode && WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Connection lost — reconnecting...");
    WiFi.reconnect();
    delay(3000);
  }

  // ── Read PZEM every 1.5 seconds ───────────────────────────
  unsigned long now = millis();
  if (now - lastReadMs >= READ_INTERVAL) {
    lastReadMs = now;

    float v  = pzem.voltage();
    float a  = pzem.current();
    float w  = pzem.power();
    float e  = pzem.energy();
    float hz = pzem.frequency();
    float pf = pzem.pf();

    if (!isnan(v) && !isnan(a) && v > 0) {
      latest.voltage   = v;
      latest.current   = a;
      latest.power     = w;
      latest.energy    = e;
      latest.frequency = hz;
      latest.pf        = pf;
      latest.valid     = true;

      Serial.printf("[PZEM] V=%.1fV  I=%.2fA  P=%.1fW  E=%.3fkWh  F=%.1fHz  PF=%.2f\n",
                    v, a, w, e, hz, pf);
    } else {
      latest.valid = false;
      Serial.println("[PZEM] Read failed — check TX/RX wiring to PZEM");
    }
  }
}
