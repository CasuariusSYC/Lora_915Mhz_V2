#pragma once
#include <Arduino.h>

// ==================================================
// EEPISAT LORA PROTOCOL  (dipakai bersama oleh TX dan RX)
// ==================================================
//
// Frame biner:
//   [0xAA][TYPE][SEQ][LEN][PAYLOAD x LEN][CRC16_H][CRC16_L]
//   CRC16-CCITT (poly 0x1021, init 0xFFFF) dihitung atas TYPE..PAYLOAD
//
// Alur:
//   TX --PKT_START ("start" = 73 74 61 72 74)--> RX
//   TX <--PKT_START_ACK ------------------------ RX   => link terhubung
//   TX --PKT_GPS --------------------------------> RX   (tiap TX_INTERVAL)
//   TX <--PKT_GPS_ACK --------------------------- RX   => link masih hidup
//
// ==================================================

#define PROTO_SOF         0xAA
#define PROTO_MAX_PAYLOAD 64

enum PacketType : uint8_t {
    PKT_START     = 0x01,   // TX -> RX : handshake
    PKT_GPS       = 0x02,   // TX -> RX : data telemetri
    PKT_START_ACK = 0x81,   // RX -> TX : jawaban handshake
    PKT_GPS_ACK   = 0x82    // RX -> TX : feedback tiap paket data
};

static const uint8_t START_WORD[5] = { 's', 't', 'a', 'r', 't' };
static const uint8_t START_OK[2]   = { 'O', 'K' };

// Payload PKT_GPS (16 byte, little-endian)
struct __attribute__((packed)) GpsPayload {
    int32_t  lat_e6;        // latitude  * 1e6
    int32_t  lon_e6;        // longitude * 1e6
    int32_t  alt_cm;        // altitude dalam cm
    uint16_t speed_x100;    // kecepatan km/h * 100
    uint8_t  sats;          // jumlah satelit
    uint8_t  flags;         // bit0 = GPS fix valid
};

// Payload PKT_GPS_ACK (3 byte): [status][rxCount_L][rxCount_H]
//   status  = 0 -> OK
//   rxCount = jumlah paket GPS yang sudah diterima RX

// --------------------------------------------------
// CRC16-CCITT
// --------------------------------------------------
inline uint16_t protoCrc16(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// --------------------------------------------------
// Bangun frame, return panjang total (6 + len)
// out minimal 6 + PROTO_MAX_PAYLOAD byte
// --------------------------------------------------
inline size_t protoBuild(uint8_t* out, uint8_t type, uint8_t seq,
                         const uint8_t* payload, uint8_t len) {
    out[0] = PROTO_SOF;
    out[1] = type;
    out[2] = seq;
    out[3] = len;
    if (len) memcpy(out + 4, payload, len);
    uint16_t crc = protoCrc16(out + 1, 3 + len);
    out[4 + len] = (uint8_t)(crc >> 8);
    out[5 + len] = (uint8_t)(crc & 0xFF);
    return 6 + len;
}

// --------------------------------------------------
// Parser frame byte-per-byte (non-blocking)
// feed() return true kalau satu frame valid (CRC cocok) selesai
// --------------------------------------------------
class FrameParser {
public:
    uint8_t type = 0;
    uint8_t seq  = 0;
    uint8_t len  = 0;
    uint8_t payload[PROTO_MAX_PAYLOAD];

    bool feed(uint8_t b) {
        uint32_t now = millis();
        if (state != 0 && (now - lastByte) > 300) state = 0;   // frame menggantung -> reset
        lastByte = now;

        switch (state) {
            case 0:
                if (b == PROTO_SOF) state = 1;
                break;
            case 1:
                type = b;
                state = 2;
                break;
            case 2:
                seq = b;
                state = 3;
                break;
            case 3:
                len = b;
                if (len > PROTO_MAX_PAYLOAD) {
                    state = 0;
                } else {
                    idx = 0;
                    state = len ? 4 : 5;
                }
                break;
            case 4:
                payload[idx++] = b;
                if (idx >= len) state = 5;
                break;
            case 5:
                crcH = b;
                state = 6;
                break;
            case 6: {
                state = 0;
                uint8_t tmp[3 + PROTO_MAX_PAYLOAD];
                tmp[0] = type;
                tmp[1] = seq;
                tmp[2] = len;
                if (len) memcpy(tmp + 3, payload, len);
                uint16_t rx = ((uint16_t)crcH << 8) | b;
                return rx == protoCrc16(tmp, 3 + len);
            }
        }
        return false;
    }

private:
    uint8_t  state    = 0;
    uint8_t  idx      = 0;
    uint8_t  crcH     = 0;
    uint32_t lastByte = 0;
};
