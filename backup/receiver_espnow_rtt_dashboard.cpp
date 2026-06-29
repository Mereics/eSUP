/*
 * ESP32-S3 Mini - RECEIVER v2 FINAL (ESP-NOW + PONG + Dashboard RTT)
 * ===================================================================
 * - Primeste PING de la Transmitter
 * - Raspunde IMEDIAT cu PONG
 * - TX masoara RTT si il trimite inapoi in payload-ul urmatorului PING
 * - RX afiseaza RTT real pe dashboard
 *
 * Librarii: WebSockets by Markus Sattler
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

const char* AP_SSID     = "ESP32-Dashboard";
const char* AP_PASSWORD = "esp32test";

#define PKT_PING 0x01
#define PKT_PONG 0x02

typedef struct {
  uint8_t  type;
  uint32_t seq;
  uint32_t tx_timestamp;
  uint32_t rx_timestamp;
  float    last_rtt;      // TX pune aici ultimul RTT masurat
  uint8_t  padding[28];
} __attribute__((packed)) esp_packet_t;

// Statistici
uint32_t ping_recv    = 0;
uint32_t pong_sent    = 0;
uint32_t out_of_order = 0;
uint32_t expected_seq = 0;
uint32_t last_seq     = 0;
int      last_rssi    = 0;

float last_rtt = 0, avg_rtt = 0, min_rtt = 9999, max_rtt = 0;
uint32_t rtt_count = 0;

float    rtt_history[60]  = {};
int      rssi_history[60] = {};
uint8_t  rtt_idx = 0, rssi_idx = 0;

uint8_t TX_MAC[6] = {};
bool    tx_known  = false;

uint32_t start_time = 0;

WebServer        server(80);
WebSocketsServer wsServer(81);

// ============================================================
//  CALLBACK RECEPTIE
// ============================================================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < (int)sizeof(esp_packet_t)) return;
  esp_packet_t pkt;
  memcpy(&pkt, data, sizeof(esp_packet_t));
  if (pkt.type != PKT_PING) return;

  // Invata MAC-ul TX la primul pachet
  if (!tx_known) {
    memcpy(TX_MAC, info->src_addr, 6);
    tx_known = true;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, TX_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.print("TX MAC: ");
    for (int i = 0; i < 6; i++) { Serial.printf("%02X", TX_MAC[i]); if(i<5) Serial.print(":"); }
    Serial.println();
  }

  // RSSI
  last_rssi = info->rx_ctrl->rssi;
  rssi_history[rssi_idx] = last_rssi;
  rssi_idx = (rssi_idx + 1) % 60;

  // Ordine
  if (ping_recv == 0) expected_seq = pkt.seq;
  if (pkt.seq != expected_seq) out_of_order++;
  expected_seq = pkt.seq + 1;
  last_seq = pkt.seq;
  ping_recv++;

  // RTT trimis de TX in payload
  if (pkt.last_rtt > 0 && pkt.last_rtt < 2000) {
    float rtt = pkt.last_rtt;
    last_rtt = rtt;
    rtt_history[rtt_idx] = rtt;
    rtt_idx = (rtt_idx + 1) % 60;
    rtt_count++;
    if (rtt_count == 1) { avg_rtt = rtt; min_rtt = rtt; max_rtt = rtt; }
    else {
      avg_rtt = avg_rtt * 0.9f + rtt * 0.1f;
      if (rtt < min_rtt) min_rtt = rtt;
      if (rtt > max_rtt) max_rtt = rtt;
    }
  }

  // Trimite PONG imediat
  esp_packet_t pong = {};
  pong.type         = PKT_PONG;
  pong.seq          = pkt.seq;
  pong.tx_timestamp = pkt.tx_timestamp;
  pong.rx_timestamp = millis();
  pong.last_rtt     = 0;

  esp_err_t res = esp_now_send(TX_MAC, (uint8_t *)&pong, sizeof(esp_packet_t));
  if (res == ESP_OK) pong_sent++;

  Serial.printf("[PING #%lu] RSSI:%d | RTT:%.1fms | PONG:%s\n",
                pkt.seq, last_rssi, last_rtt, res==ESP_OK?"OK":"FAIL");
}

// ============================================================
//  JSON STATS
// ============================================================
String buildJson() {
  // Construieste array RTT history
  String rh = "[";
  for (int i = 0; i < 60; i++) {
    int idx = (rtt_idx + i) % 60;
    rh += String(rtt_history[idx], 1);
    if (i < 59) rh += ",";
  }
  rh += "]";

  String rs = "[";
  for (int i = 0; i < 60; i++) {
    int idx = (rssi_idx + i) % 60;
    rs += String(rssi_history[idx]);
    if (i < 59) rs += ",";
  }
  rs += "]";

  String j = "{";
  j += "\"rtt\":"    + String(last_rtt, 1) + ",";
  j += "\"avg\":"    + String(avg_rtt,  1) + ",";
  j += "\"min\":"    + String(min_rtt == 9999 ? 0 : min_rtt, 1) + ",";
  j += "\"max\":"    + String(max_rtt,  1) + ",";
  j += "\"ping\":"   + String(ping_recv)   + ",";
  j += "\"pong\":"   + String(pong_sent)   + ",";
  j += "\"ooo\":"    + String(out_of_order) + ",";
  j += "\"seq\":"    + String(last_seq)    + ",";
  j += "\"rssi\":"   + String(last_rssi)   + ",";
  j += "\"uptime\":" + String((millis()-start_time)/1000) + ",";
  j += "\"rtt_h\":"  + rh + ",";
  j += "\"rssi_h\":" + rs + "}";
  return j;
}

// ============================================================
//  HTML DASHBOARD
// ============================================================
const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="ro">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 RTT Dashboard</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap');
:root{--bg:#0a0e1a;--sf:#111827;--bd:#1e3a5f;--ac:#00d4ff;--g:#00ff9d;--w:#ffb800;--r:#ff4444;--tx:#c8d8e8;--dm:#4a6080}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--tx);font-family:'Rajdhani',sans-serif;padding:14px;min-height:100vh}
body::before{content:'';position:fixed;inset:0;background:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,212,255,.015) 2px,rgba(0,212,255,.015) 4px);pointer-events:none;z-index:999}
header{display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid var(--bd);padding-bottom:10px;margin-bottom:16px}
.logo{font-family:'Share Tech Mono',monospace;font-size:1rem;color:var(--ac);letter-spacing:2px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--r);display:inline-block;margin-right:6px;box-shadow:0 0 5px currentColor;transition:background .3s}
.dot.on{background:var(--g)}
.st{font-size:.75rem;font-family:'Share Tech Mono',monospace;color:var(--dm)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px;margin-bottom:16px}
.card{background:var(--sf);border:1px solid var(--bd);border-radius:8px;padding:12px}
.lbl{font-size:.65rem;color:var(--dm);text-transform:uppercase;letter-spacing:1.5px;margin-bottom:5px;font-family:'Share Tech Mono',monospace}
.val{font-size:1.75rem;font-weight:700;color:var(--ac);line-height:1;font-family:'Share Tech Mono',monospace}
.unit{font-size:.68rem;color:var(--dm);margin-top:3px}
.good{color:var(--g)}.warn{color:var(--w)}.bad{color:var(--r)}
.wide{grid-column:1/-1}
.cc{background:var(--sf);border:1px solid var(--bd);border-radius:8px;padding:12px;margin-bottom:10px}
.ct{font-size:.65rem;color:var(--dm);text-transform:uppercase;letter-spacing:1.5px;margin-bottom:8px;font-family:'Share Tech Mono',monospace}
canvas{width:100%!important;display:block}
.lb{font-family:'Share Tech Mono',monospace;font-size:.7rem;height:100px;overflow-y:auto;background:#070c14;border:1px solid var(--bd);border-radius:4px;padding:7px;margin-top:8px}
.lb p{margin-bottom:2px}.lb .ok{color:var(--g)}.lb .w{color:var(--w)}.lb .e{color:var(--r)}
.bar-bg{height:6px;background:var(--bd);border-radius:3px;margin-top:7px;overflow:hidden}
.bar-fill{height:100%;border-radius:3px;background:linear-gradient(90deg,var(--r),var(--w),var(--g));transition:width .5s}
.rtt-gauge{height:10px;background:var(--bd);border-radius:5px;margin-top:8px;overflow:hidden}
.rtt-fill{height:100%;border-radius:5px;transition:width .4s,background .4s}
.foot{text-align:center;font-size:.65rem;color:var(--dm);font-family:'Share Tech Mono',monospace;margin-top:14px;padding-top:10px;border-top:1px solid var(--bd)}
</style>
</head>
<body>
<header>
  <div class="logo">&#9711; ESP32-S3 RTT LIVE</div>
  <div><span class="dot" id="dot"></span><span class="st" id="wst">DISCONNECTED</span></div>
</header>

<div class="grid">
  <div class="card">
    <div class="lbl">RTT curent</div>
    <div class="val" id="rtt">—</div>
    <div class="unit">ms round-trip</div>
    <div class="rtt-gauge"><div class="rtt-fill" id="gRtt" style="width:0%"></div></div>
  </div>
  <div class="card">
    <div class="lbl">One-Way Est.</div>
    <div class="val good" id="ow">—</div>
    <div class="unit">ms (RTT÷2)</div>
  </div>
  <div class="card">
    <div class="lbl">RTT mediu</div>
    <div class="val" id="avg">—</div>
    <div class="unit">ms EWMA</div>
  </div>
  <div class="card">
    <div class="lbl">RTT minim</div>
    <div class="val good" id="mn">—</div>
    <div class="unit">ms best</div>
  </div>
  <div class="card">
    <div class="lbl">RTT maxim</div>
    <div class="val warn" id="mx">—</div>
    <div class="unit">ms worst</div>
  </div>
  <div class="card">
    <div class="lbl">PDR</div>
    <div class="val" id="pdr">—</div>
    <div class="unit">% livrate</div>
  </div>
  <div class="card">
    <div class="lbl">Pachete</div>
    <div class="val" id="pk">—</div>
    <div class="unit">ping total</div>
  </div>
  <div class="card">
    <div class="lbl">RSSI</div>
    <div class="val" id="rs">—</div>
    <div class="unit">dBm</div>
    <div class="bar-bg"><div class="bar-fill" id="bRs" style="width:0%"></div></div>
  </div>
  <div class="card">
    <div class="lbl">Out of order</div>
    <div class="val warn" id="ooo">—</div>
    <div class="unit">pachete</div>
  </div>
  <div class="card">
    <div class="lbl">Ultim seq</div>
    <div class="val" id="seq">—</div>
    <div class="unit">#</div>
  </div>
</div>

<div class="cc wide">
  <div class="ct">&#9642; RTT real-time (ms) — ultimele 60 ping-uri</div>
  <canvas id="cRtt" height="90"></canvas>
</div>
<div class="cc wide">
  <div class="ct">&#9642; RSSI (dBm) — ultimele 60 ping-uri</div>
  <canvas id="cRssi" height="55"></canvas>
</div>
<div class="cc wide">
  <div class="ct">&#9642; Log</div>
  <div class="lb" id="log"></div>
</div>

<div class="foot">ESP32-S3 Mini &bull; ESP-NOW RTT Monitor &bull; WS:81</div>

<script>
let ws;
const rttH=new Array(60).fill(null), rssiH=new Array(60).fill(null);

function conn(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=()=>{setSt(true);lg('Conectat','ok')};
  ws.onclose=()=>{setSt(false);lg('Reconectez...','e');setTimeout(conn,2000)};
  ws.onmessage=e=>{try{upd(JSON.parse(e.data))}catch(x){}};
}
function setSt(on){
  document.getElementById('dot').className='dot'+(on?' on':'');
  document.getElementById('wst').textContent=on?'CONNECTED':'RECONNECTING...';
}
function lc(v,g,w){return v<=g?'good':v<=w?'warn':'bad'}
function set(id,txt,cls){const e=document.getElementById(id);if(!e)return;e.textContent=txt;if(cls)e.className='val '+cls}

function upd(d){
  const rtt=d.rtt, ow=(rtt/2);
  set('rtt', rtt.toFixed(1)+' ms', lc(rtt,5,20));
  set('ow',  ow.toFixed(1)+' ms', 'good');
  set('avg', d.avg.toFixed(1)+' ms', lc(d.avg,5,20));
  set('mn',  d.min.toFixed(1)+' ms', 'good');
  set('mx',  d.max.toFixed(1)+' ms', 'warn');
  set('pk',  d.ping);
  set('seq', '#'+d.seq);
  set('ooo', d.ooo, d.ooo>0?'warn':'');

  const pdrV=d.ping>0?(100*d.pong/d.ping):0;
  set('pdr', pdrV.toFixed(1)+'%', pdrV>=95?'good':pdrV>=80?'warn':'bad');

  const rsEl=document.getElementById('rs');
  rsEl.textContent=d.rssi+' dBm';
  rsEl.className='val '+(d.rssi>-60?'good':d.rssi>-75?'warn':'bad');
  const rp=Math.min(100,Math.max(0,(d.rssi+90)/60*100));
  document.getElementById('bRs').style.width=rp+'%';

  const gf=document.getElementById('gRtt');
  const rttPct=Math.min(100,(rtt/50)*100);
  gf.style.width=rttPct+'%';
  gf.style.background=rtt<5?'var(--g)':rtt<20?'var(--w)':'var(--r)';

  // Actualizeaza istoricele
  if(d.rtt_h){d.rtt_h.forEach((v,i)=>rttH[i]=v);}
  else{rttH.push(rtt);if(rttH.length>60)rttH.shift();}
  if(d.rssi_h){d.rssi_h.forEach((v,i)=>rssiH[i]=v);}
  else{rssiH.push(d.rssi);if(rssiH.length>60)rssiH.shift();}

  draw('cRtt', rttH, 'var(--ac)', 'rgba(0,212,255,.2)', 'ms', 90);
  draw('cRssi', rssiH, 'var(--g)', 'rgba(0,255,157,.15)', 'dBm', 55);

  const cls=rtt<5?'ok':rtt<20?'w':'e';
  lg(`#${d.seq} RTT:${rtt.toFixed(1)}ms OW:${ow.toFixed(1)}ms RSSI:${d.rssi}dBm`, cls);
}

function draw(id,hist,lc,fc,unit,H){
  const cv=document.getElementById(id);
  const W=cv.offsetWidth; cv.width=W; cv.height=H;
  const valid=hist.filter(v=>v!==null&&v!==0);
  if(valid.length<2)return;
  const ctx=cv.getContext('2d');
  ctx.clearRect(0,0,W,H);
  const maxV=Math.max(...valid), minV=Math.min(...valid);
  const rng=maxV-minV||1;
  const step=W/(hist.length-1), pad=13;

  ctx.strokeStyle='#1e3a5f';ctx.lineWidth=.5;
  for(let i=0;i<=3;i++){
    const y=H-pad-(i/3)*(H-pad*2);
    ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();
    ctx.fillStyle='#4a6080';ctx.font='9px Share Tech Mono,monospace';
    ctx.fillText((minV+(i/3)*rng).toFixed(0)+unit,2,y-2);
  }

  const g=ctx.createLinearGradient(0,0,0,H);
  g.addColorStop(0,fc);g.addColorStop(1,'transparent');
  ctx.beginPath();let s=false;
  for(let i=0;i<hist.length;i++){
    if(!hist[i]){s=false;continue;}
    const x=i*step,y=H-pad-((hist[i]-minV)/rng)*(H-pad*2);
    if(!s){ctx.moveTo(x,y);s=true;}else ctx.lineTo(x,y);
  }
  const li=hist.map((v,i)=>v?i:-1).filter(i=>i>=0).pop()||0;
  ctx.lineTo(li*step,H);ctx.lineTo(0,H);ctx.fillStyle=g;ctx.fill();

  ctx.beginPath();s=false;
  for(let i=0;i<hist.length;i++){
    if(!hist[i]){s=false;continue;}
    const x=i*step,y=H-pad-((hist[i]-minV)/rng)*(H-pad*2);
    if(!s){ctx.moveTo(x,y);s=true;}else ctx.lineTo(x,y);
  }
  ctx.strokeStyle=lc;ctx.lineWidth=1.5;ctx.stroke();
}

const lb=document.getElementById('log');
function lg(msg,cls){
  const p=document.createElement('p');p.className=cls||'';
  const t=new Date().toLocaleTimeString('ro',{hour12:false});
  p.textContent=`[${t}] ${msg}`;
  lb.appendChild(p);
  if(lb.children.length>80)lb.removeChild(lb.firstChild);
  lb.scrollTop=lb.scrollHeight;
}
conn();
</script>
</body>
</html>
)rawhtml";

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("  ESP32-S3 Mini - RECEIVER v2 FINAL");
  Serial.println("========================================");

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD, 1);
  delay(500);

  Serial.printf("AP: %s | IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  Serial.print("MAC Receiver: ");
  Serial.println(WiFi.macAddress());
  Serial.println(">>> Copiaza MAC-ul de mai sus in Transmitter_v2!\n");
  Serial.println("Dashboard: http://192.168.4.1\n");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[FATAL] ESP-NOW init failed!");
    while(true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);
  String json = buildJson();
  server.on("/", []() { server.send_P(200, "text/html", DASHBOARD_HTML); });
  server.on("/stats", []() { server.send(200, "application/json", buildJson()); });
  server.begin();

  wsServer.begin();
  wsServer.onEvent([](uint8_t n, WStype_t t, uint8_t *p, size_t l){
    if (t == WStype_CONNECTED) Serial.printf("[WS] Client #%u conectat\n", n);
  });

  start_time = millis();
  Serial.println("Astept PING-uri...\n");
}

void loop() {
  server.handleClient();
  wsServer.loop();

  static uint32_t lastBC = 0;
  if (millis() - lastBC >= 300 && ping_recv > 0) {
    lastBC = millis();
    String json = buildJson();
    wsServer.broadcastTXT(json);
  }

  static uint32_t lastLog = 0;
  if (millis() - lastLog >= 10000 && ping_recv > 0) {
    lastLog = millis();
    Serial.printf("\n--- 10s --- ping:%lu pong:%lu ooo:%lu rssi:%d rtt:%.1f avg:%.1f\n\n",
                  ping_recv, pong_sent, out_of_order, last_rssi, last_rtt, avg_rtt);
  }
}
