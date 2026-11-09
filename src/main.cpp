#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPSPlus.h>
#include "protocol.h"

// ==================================================
// PIN CONFIGURATION
// ==================================================

// GPS SAM-M10Q - UART2
#define GPS_RX 17//16
#define GPS_TX 16
#define GPS_BAUD 115200

// LoRa E220-900T22D - UART1
#define LORA_RX 26
#define LORA_TX 25
#define LORA_BAUD 9600

// Pin kontrol E220  (SESUAIKAN dengan wiring kamu)
#define LORA_M0  32     // ESP32 -> M0
#define LORA_M1  33     // ESP32 -> M1
#define LORA_AUX 25     // E220 AUX -> ESP32 (input)

// LED indikator
#define LED_PIN 23

// ==================================================
// WIFI ACCESS POINT
// ==================================================

#define AP_SSID     "Lora Testing"
#define AP_PASSWORD ""            // kosong = AP terbuka, minimal 8 karakter kalau mau dipassword

// ==================================================
// TIMING
// ==================================================

const unsigned long TX_INTERVAL        = 2000;   // jeda kirim paket GPS
const unsigned long ACK_TIMEOUT        = 1500;   // batas tunggu feedback dari RX
const uint8_t       MAX_MISSED_ACK     = 3;      // ACK hilang berturut-turut -> link dianggap putus
const unsigned long HANDSHAKE_INTERVAL = 2000;   // jeda kirim "start"
const uint8_t       HS_TRIES_BEFORE_HW = 10;     // gagal handshake sekian kali -> cek hardware lagi
const unsigned long HW_RETRY_INTERVAL  = 3000;   // jeda ulang cek hardware kalau gagal
const unsigned long LED_BLINK_MS       = 250;

// ==================================================
// OBJECT
// ==================================================

TinyGPSPlus gps;
HardwareSerial GPS(2);
HardwareSerial LoRa(1);
WebServer server(80);
FrameParser parser;

// ==================================================
// STATE
// ==================================================

enum LinkState : uint8_t {
    ST_HW_CHECK,     // cek modul LoRa (AUX, M0/M1, baca config)
    ST_HANDSHAKE,    // kirim "start", tunggu RX menjawab
    ST_CONNECTED     // terhubung, kirim data + tunggu feedback
};

LinkState state = ST_HW_CHECK;
String hwMessage = "belum dicek";

uint8_t  seqCounter  = 0;
uint8_t  missedAcks  = 0;
uint8_t  hsTries     = 0;
bool     waitingAck  = false;
uint8_t  pendingSeq  = 0;

unsigned long sentAt     = 0;
unsigned long lastTx     = 0;
unsigned long lastHsTx   = 0;
unsigned long lastHwTry  = 0;
unsigned long lastAckAt  = 0;
unsigned long rttMs      = 0;

uint32_t pktSent  = 0;
uint32_t pktAcked = 0;
uint32_t pktLost  = 0;
uint16_t remoteRxCount = 0;

// ==================================================
// HALAMAN WEB (disimpan di flash)
// ==================================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>EEPISAT - Lora Testing</title>
<style>
:root{--bg:#f3f4f6;--card:#fff;--tx:#111827;--mut:#6b7280;--bd:#e5e7eb;--ok:#16a34a;--warn:#d97706;--bad:#dc2626}
@media(prefers-color-scheme:dark){:root{--bg:#0e1116;--card:#161b22;--tx:#e6edf3;--mut:#8b949e;--bd:#262c36}}
*{box-sizing:border-box}
body{margin:0;padding:14px;background:var(--bg);color:var(--tx);font:15px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
h1{font-size:18px;margin:0 0 2px}
h2{font-size:12px;color:var(--mut);text-transform:uppercase;letter-spacing:.06em;margin:18px 0 8px}
.sub{color:var(--mut);font-size:12px;margin-bottom:12px}
.grid{display:grid;gap:10px;grid-template-columns:repeat(auto-fit,minmax(140px,1fr))}
.card{background:var(--card);border:1px solid var(--bd);border-radius:10px;padding:10px 12px}
.k{color:var(--mut);font-size:12px}
.v{font-size:19px;font-weight:600;margin-top:2px;overflow-wrap:anywhere}
.badge{display:inline-block;margin-top:4px;padding:3px 10px;border-radius:99px;color:#fff;font-size:13px;font-weight:600;background:var(--mut)}
.ok{background:var(--ok)}.warn{background:var(--warn)}.bad{background:var(--bad)}
a{color:inherit;font-size:13px}
#err{display:none;margin-top:12px;color:var(--bad);font-size:13px}
</style>
</head>
<body>
<h1>EEPISAT Telemetry</h1>
<div class="sub">AP: Lora Testing &middot; <span id="up">-</span></div>

<div class="grid">
  <div class="card"><div class="k">LoRa link</div><span id="lora" class="badge">-</span><div class="k" id="hw" style="margin-top:6px"></div></div>
  <div class="card"><div class="k">GPS</div><span id="gps" class="badge">-</span></div>
</div>

<h2>Posisi</h2>
<div class="grid" id="pos"></div>
<p><a id="map" href="#" target="_blank" style="display:none">Buka di Google Maps (butuh internet di HP)</a></p>

<h2>Link LoRa</h2>
<div class="grid" id="lnk"></div>

<div id="err">Koneksi ke ESP32 terputus, mencoba lagi...</div>

<script>
const POS=[['lat','Latitude'],['lon','Longitude'],['alt','Altitude (m)'],['spd','Kecepatan (km/h)'],['sats','Satelit'],['hdop','HDOP'],['crs','Arah (&deg;)'],['utc','Waktu UTC']];
const LNK=[['sent','Paket terkirim'],['acked','ACK diterima'],['lost','ACK hilang'],['loss','Packet loss'],['rtt','RTT'],['ackago','ACK terakhir'],['rxc','Diterima RX'],['miss','Missed ACK'],['sta','Client AP']];
const $=id=>document.getElementById(id);
function build(id,a){$(id).innerHTML=a.map(x=>'<div class="card"><div class="k">'+x[1]+'</div><div class="v" id="f_'+x[0]+'">-</div></div>').join('');}
build('pos',POS);build('lnk',LNK);
const put=(k,v)=>{$('f_'+k).textContent=(v===null||v===undefined||v==='')?'-':v;};
function badge(id,t,c){const e=$(id);e.textContent=t;e.className='badge '+c;}
async function tick(){
  try{
    const d=await (await fetch('/data',{cache:'no-store'})).json();
    $('err').style.display='none';
    const m={CONNECTED:['TERHUBUNG','ok'],HANDSHAKE:['MENUNGGU RX','warn'],HW_CHECK:['MODUL ERROR','bad']}[d.state]||[d.state,'bad'];
    badge('lora',m[0],m[1]);
    $('hw').textContent=d.state==='HW_CHECK'?d.hw:'';
    if(d.chars===0)badge('gps','TIDAK ADA DATA','bad');
    else if(d.fix)badge('gps','FIX','ok');
    else badge('gps','MENCARI SATELIT','warn');
    put('lat',d.lat);put('lon',d.lon);put('alt',d.alt);put('spd',d.spd);
    put('sats',d.sats);put('hdop',d.hdop);put('crs',d.crs);
    put('utc',(d.date&&d.time)?d.date+' '+d.time:d.time);
    const mp=$('map');
    if(d.fix){mp.href='https://maps.google.com/?q='+d.lat+','+d.lon;mp.style.display='';}else{mp.style.display='none';}
    put('sent',d.sent);put('acked',d.acked);put('lost',d.lost);
    const tot=d.acked+d.lost;
    put('loss',tot?(d.lost*100/tot).toFixed(1)+' %':null);
    put('rtt',(d.state==='CONNECTED'&&d.rtt>0)?d.rtt+' ms':null);
    put('ackago',d.ackago>=0?(d.ackago/1000).toFixed(1)+' s lalu':null);
    put('rxc',d.rxc);put('miss',d.miss+' / '+d.maxmiss);put('sta',d.sta);
    $('up').textContent='uptime '+d.up+' s';
  }catch(e){$('err').style.display='block';}
}
tick();setInterval(tick,1000);
</script>
</body>
</html>
)rawliteral";

// ==================================================
// HELPER UMUM
// ==================================================

void readGPS() {
    while (GPS.available() > 0) {
        gps.encode(GPS.read());
    }
}

// LED: kedip saat HW check / handshake, nyala terus saat terhubung
void updateLed() {
    static unsigned long lastToggle = 0;
    static bool level = false;

    if (state == ST_CONNECTED) {
        digitalWrite(LED_PIN, HIGH);
        level = true;
        return;
    }

    if (millis() - lastToggle >= LED_BLINK_MS) {
        lastToggle = millis();
        level = !level;
        digitalWrite(LED_PIN, level ? HIGH : LOW);
    }
}

// Dipanggil di semua tempat yang menunggu supaya GPS, web, dan LED tetap jalan
void background() {
    readGPS();
    server.handleClient();
    updateLed();
}

void waitMs(uint32_t ms) {
    uint32_t t = millis();
    while (millis() - t < ms) {
        background();
        delay(1);
    }
}

bool waitAux(int level, uint32_t timeoutMs) {
    uint32_t t = millis();
    while (millis() - t < timeoutMs) {
        if (digitalRead(LORA_AUX) == level) return true;
        background();
        delay(1);
    }
    return digitalRead(LORA_AUX) == level;
}

// Mode E220:  M0 M1 = 0 0 normal | 1 1 config
void setLoraMode(bool m0, bool m1) {
    digitalWrite(LORA_M0, m0 ? HIGH : LOW);
    digitalWrite(LORA_M1, m1 ? HIGH : LOW);
    waitMs(50);                 // modul butuh waktu pindah mode (AUX turun lalu naik)
    waitAux(HIGH, 1000);        // tunggu modul siap
    waitMs(20);
}

const char* stateText() {
    switch (state) {
        case ST_HW_CHECK:  return "HW_CHECK";
        case ST_HANDSHAKE: return "HANDSHAKE";
        default:           return "CONNECTED";
    }
}

// ==================================================
// STEP 1 : CEK HARDWARE LORA (AUX + M0/M1 + BACA CONFIG)
// ==================================================

bool checkLoraHardware() {

    static const uint32_t UART_BAUD_TBL[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
    static const float    AIR_RATE_TBL[8]  = {2.4, 2.4, 2.4, 4.8, 9.6, 19.2, 38.4, 62.5};

    // 1) Mode normal, AUX harus HIGH (self-check modul selesai)
    setLoraMode(false, false);

    if (digitalRead(LORA_AUX) != HIGH) {
        hwMessage = "AUX tidak HIGH (cek power dan wiring AUX)";
        return false;
    }

    // 2) Kotak-katik M0/M1 -> masuk config mode, lalu baca register modul
    setLoraMode(true, true);

    while (LoRa.available()) LoRa.read();

    const uint8_t readCmd[3] = { 0xC1, 0x00, 0x08 };    // baca 8 register E220
    LoRa.write(readCmd, sizeof(readCmd));

    uint8_t resp[11];
    size_t n = 0;
    uint32_t t = millis();
    while (n < sizeof(resp) && (millis() - t) < 600) {
        if (LoRa.available()) resp[n++] = LoRa.read();
        else background();
    }

    // 3) Balik ke mode normal
    setLoraMode(false, false);

    if (n < 3 || resp[0] != 0xC1) {
        hwMessage = "Modul tidak menjawab config (cek TX/RX dan M0/M1)";
        return false;
    }

    // 4) Tampilkan hasil baca config
    if (n >= 11 && resp[2] == 0x08) {
        uint8_t reg0 = resp[5];
        uint8_t reg3 = resp[8];
        uint32_t modBaud = UART_BAUD_TBL[reg0 >> 5];

        Serial.println("[HW] Config modul LoRa:");
        Serial.printf("     Address   : 0x%02X%02X\n", resp[3], resp[4]);
        Serial.printf("     Channel   : %u (%.3f MHz)\n", resp[7], 850.125 + resp[7]);
        Serial.printf("     UART      : %lu bps\n", (unsigned long)modBaud);
        Serial.printf("     Air rate  : %.1f kbps\n", AIR_RATE_TBL[reg0 & 0x07]);
        Serial.printf("     Mode TX   : %s\n", (reg3 & 0x40) ? "FIXED (harus diubah ke Transparent!)" : "Transparent");
        Serial.printf("     RSSI byte : %s\n", (reg3 & 0x80) ? "ON" : "OFF");

        if (modBaud != LORA_BAUD) {
            hwMessage = "Baud modul tidak sama dengan LORA_BAUD";
            return false;
        }
    }

    return true;
}

// ==================================================
// KIRIM FRAME LORA
// ==================================================

bool sendFrame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len) {
    if (!waitAux(HIGH, 200)) return false;      // modul masih sibuk

    uint8_t buf[6 + PROTO_MAX_PAYLOAD];
    size_t n = protoBuild(buf, type, seq, payload, len);
    LoRa.write(buf, n);                          // satu kali write = satu paket udara
    return true;
}

void sendGpsPacket() {

    GpsPayload p;
    bool fix = gps.location.isValid();

    p.lat_e6 = fix ? (int32_t)lround(gps.location.lat() * 1e6) : 0;
    p.lon_e6 = fix ? (int32_t)lround(gps.location.lng() * 1e6) : 0;
    p.alt_cm = gps.altitude.isValid() ? (int32_t)lround(gps.altitude.meters() * 100.0) : 0;

    long sp = gps.speed.isValid() ? lround(gps.speed.kmph() * 100.0) : 0;
    if (sp < 0) sp = 0;
    if (sp > 65535) sp = 65535;
    p.speed_x100 = (uint16_t)sp;

    p.sats  = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
    p.flags = fix ? 0x01 : 0x00;

    uint8_t seq = ++seqCounter;

    if (sendFrame(PKT_GPS, seq, (const uint8_t*)&p, sizeof(p))) {
        pendingSeq = seq;
        waitingAck = true;
        sentAt = millis();
        pktSent++;

        Serial.printf("[TX] #%u fix=%d lat=%.6f lon=%.6f alt=%.2f spd=%.2f sat=%u\n",
                      seq, fix ? 1 : 0,
                      p.lat_e6 / 1e6, p.lon_e6 / 1e6,
                      p.alt_cm / 100.0, p.speed_x100 / 100.0, p.sats);
    } else {
        Serial.println("[TX] LoRa sibuk (AUX LOW), paket dilewati");
    }
}

// ==================================================
// TERIMA FEEDBACK DARI RX
// ==================================================

void handleLoraRx() {

    while (LoRa.available()) {

        if (!parser.feed(LoRa.read())) continue;

        unsigned long now = millis();

        // --- RX menjawab "start" ---
        if (parser.type == PKT_START_ACK && state == ST_HANDSHAKE) {
            state = ST_CONNECTED;
            missedAcks = 0;
            waitingAck = false;
            lastAckAt = now;
            lastTx = now - TX_INTERVAL;          // langsung kirim data pertama
            Serial.println("[LoRa] RX menjawab START -> TERHUBUNG");
        }

        // --- RX memberi feedback untuk paket data ---
        else if (parser.type == PKT_GPS_ACK && state == ST_CONNECTED &&
                 waitingAck && parser.seq == pendingSeq) {

            waitingAck = false;
            missedAcks = 0;
            pktAcked++;
            lastAckAt = now;
            rttMs = now - sentAt;

            if (parser.len >= 3) {
                remoteRxCount = (uint16_t)parser.payload[1] | ((uint16_t)parser.payload[2] << 8);
            }

            Serial.printf("[LoRa] ACK #%u OK  RTT=%lu ms  RX_count=%u\n",
                          parser.seq, rttMs, remoteRxCount);
        }
    }
}

void checkAckTimeout() {

    if (waitingAck && (millis() - sentAt) > ACK_TIMEOUT) {

        waitingAck = false;
        pktLost++;
        missedAcks++;

        Serial.printf("[LoRa] ACK #%u TIMEOUT (%u/%u)\n", pendingSeq, missedAcks, MAX_MISSED_ACK);

        if (missedAcks >= MAX_MISSED_ACK) {
            state = ST_HANDSHAKE;
            hsTries = 0;
            lastHsTx = millis() - HANDSHAKE_INTERVAL;     // kirim "start" segera
            Serial.println("[LoRa] LINK PUTUS -> ulang handshake");
        }
    }
}

// ==================================================
// WEB SERVER
// ==================================================

static void kv(String& j, const char* key, const String& val, bool quoted = false) {
    if (j.length() > 1) j += ',';
    j += '"';
    j += key;
    j += "\":";
    if (quoted) { j += '"'; j += val; j += '"'; }
    else        { j += val; }
}

static String jnum(bool ok, double v, int dec) {
    return ok ? String(v, dec) : String("null");
}

void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {

    bool fix = gps.location.isValid();

    char tbuf[12] = "";
    char dbuf[12] = "";
    if (gps.time.isValid()) {
        snprintf(tbuf, sizeof(tbuf), "%02u:%02u:%02u",
                 (unsigned)gps.time.hour(), (unsigned)gps.time.minute(), (unsigned)gps.time.second());
    }
    if (gps.date.isValid()) {
        snprintf(dbuf, sizeof(dbuf), "%04u-%02u-%02u",
                 (unsigned)gps.date.year(), (unsigned)gps.date.month(), (unsigned)gps.date.day());
    }

    unsigned long now = millis();

    String j;
    j.reserve(700);
    j += '{';

    kv(j, "state",  stateText(), true);
    kv(j, "hw",     hwMessage, true);
    kv(j, "chars",  String((unsigned long)gps.charsProcessed()));
    kv(j, "fix",    fix ? "true" : "false");
    kv(j, "lat",    jnum(fix, gps.location.lat(), 6));
    kv(j, "lon",    jnum(fix, gps.location.lng(), 6));
    kv(j, "alt",    jnum(gps.altitude.isValid(), gps.altitude.meters(), 2));
    kv(j, "spd",    jnum(gps.speed.isValid(), gps.speed.kmph(), 2));
    kv(j, "sats",   gps.satellites.isValid() ? String((unsigned long)gps.satellites.value()) : String("null"));
    kv(j, "hdop",   jnum(gps.hdop.isValid(), gps.hdop.hdop(), 2));
    kv(j, "crs",    jnum(gps.course.isValid(), gps.course.deg(), 1));
    kv(j, "time",   tbuf, true);
    kv(j, "date",   dbuf, true);

    kv(j, "sent",    String((unsigned long)pktSent));
    kv(j, "acked",   String((unsigned long)pktAcked));
    kv(j, "lost",    String((unsigned long)pktLost));
    kv(j, "rtt",     String(rttMs));
    kv(j, "ackago",  lastAckAt ? String(now - lastAckAt) : String("-1"));
    kv(j, "rxc",     String((unsigned long)remoteRxCount));
    kv(j, "miss",    String((unsigned long)missedAcks));
    kv(j, "maxmiss", String((unsigned long)MAX_MISSED_ACK));
    kv(j, "sta",     String((unsigned long)WiFi.softAPgetStationNum()));
    kv(j, "up",      String(now / 1000));

    j += '}';

    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", j);
}

// ==================================================
// SETUP
// ==================================================

void setup() {

    // USB Serial
    Serial.begin(115200);

    // LED + pin kontrol LoRa
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    pinMode(LORA_M0, OUTPUT);
    pinMode(LORA_M1, OUTPUT);
    pinMode(LORA_AUX, INPUT_PULLDOWN);      // kalau modul tidak terpasang, AUX terbaca LOW -> gagal
    digitalWrite(LORA_M0, LOW);
    digitalWrite(LORA_M1, LOW);

    // GPS UART2
    GPS.setRxBufferSize(1024);
    GPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

    // LoRa UART1
    LoRa.setRxBufferSize(512);
    LoRa.begin(LORA_BAUD, SERIAL_8N1, LORA_RX, LORA_TX);

    Serial.println();
    Serial.println("================================");
    Serial.println("EEPISAT GPS TELEMETRY");
    Serial.println("================================");

    // WiFi Access Point + web server
    WiFi.mode(WIFI_AP);
    if (strlen(AP_PASSWORD) >= 8) WiFi.softAP(AP_SSID, AP_PASSWORD);
    else                          WiFi.softAP(AP_SSID);

    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.begin();

    Serial.print("AP  : ");
    Serial.println(AP_SSID);
    Serial.print("Web : http://");
    Serial.println(WiFi.softAPIP());
    Serial.println("Memulai cek modul LoRa...");

    lastHwTry = millis() - HW_RETRY_INTERVAL;     // cek hardware langsung di loop pertama
}

// ==================================================
// LOOP
// ==================================================

void loop() {

    background();
    handleLoraRx();

    unsigned long now = millis();

    switch (state) {

        // ------------------------------------------
        // CEK HARDWARE
        // ------------------------------------------
        case ST_HW_CHECK:
            if (now - lastHwTry >= HW_RETRY_INTERVAL) {

                Serial.println("[HW] Cek modul LoRa...");
                bool ok = checkLoraHardware();
                lastHwTry = millis();

                if (ok) {
                    hwMessage = "OK";
                    state = ST_HANDSHAKE;
                    hsTries = 0;
                    lastHsTx = millis() - HANDSHAKE_INTERVAL;
                    Serial.println("[HW] Modul LoRa OK -> handshake dengan RX");
                } else {
                    Serial.print("[HW] GAGAL: ");
                    Serial.println(hwMessage);
                }
            }
            break;

        // ------------------------------------------
        // HANDSHAKE: kirim "start" (biner), tunggu jawaban RX
        // ------------------------------------------
        case ST_HANDSHAKE:
            if (now - lastHsTx >= HANDSHAKE_INTERVAL) {
                lastHsTx = now;

                if (hsTries >= HS_TRIES_BEFORE_HW) {
                    Serial.println("[LoRa] RX tidak menjawab, cek ulang hardware");
                    state = ST_HW_CHECK;
                    lastHwTry = now - HW_RETRY_INTERVAL;
                    break;
                }

                hsTries++;
                if (sendFrame(PKT_START, ++seqCounter, START_WORD, sizeof(START_WORD))) {
                    Serial.printf("[LoRa] kirim START (%u/%u), menunggu RX...\n", hsTries, HS_TRIES_BEFORE_HW);
                }
            }
            break;

        // ------------------------------------------
        // TERHUBUNG: kirim data, wajib ada feedback
        // ------------------------------------------
        case ST_CONNECTED:
            checkAckTimeout();

            if (state == ST_CONNECTED && (now - lastTx) >= TX_INTERVAL) {
                lastTx = now;
                sendGpsPacket();

                if (!gps.location.isValid()) {
                    Serial.println("[GPS] menunggu posisi valid (paket tetap dikirim untuk cek link)");
                }
            }
            break;
    }
}