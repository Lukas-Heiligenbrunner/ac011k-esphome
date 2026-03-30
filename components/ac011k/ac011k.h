// components/ac011k/ac011k.h
// ESPHome external component for the EN+ / Autoaid AC011K EV charger.
// Implements the PrivComm binary serial protocol spoken between the ESP32
// and the GD32 co-processor that controls the charging hardware.
//
// Reference: software/src/modules/ac011k/ac011k.cpp in this repository.
//
// Wiring (confirmed from ecactus_firmware_dump.bin):
//   UART1 RX <- GPIO 34  (from GD32, input-only pin)
//   UART1 TX -> GPIO 32  (to GD32)
//   Green LED  = GPIO 25  (active LOW)
//   Red LED    = GPIO 33  (active LOW)
//   Button SW3 = GPIO ?? (GPIO 32 is TX — button pin not yet traced)

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <cstring>
#include <ctime>

namespace esphome {
namespace ac011k {

static const char *const TAG = "ac011k";

static inline uint16_t get_u16(const uint8_t *buf, int i) {
    return (uint16_t)(buf[i] | ((uint16_t)buf[i + 1] << 8));
}

// Log up to 32 bytes of a frame as hex + a decoded one-liner at DEBUG level.
// dir: "TX" or "RX"
static void log_frame(const char *dir, const uint8_t *frame, size_t len) {
    // Hex dump (max 64 bytes shown)
    char hex[3 * 64 + 1];
    size_t show = len < 64 ? len : 64;
    for (size_t i = 0; i < show; i++)
        snprintf(hex + 3 * i, 4, "%02X ", frame[i]);
    if (show < len)
        snprintf(hex + 3 * show, 4, "...");
    else
        hex[3 * show > 0 ? 3 * show - 1 : 0] = '\0';

    uint8_t cmd = len > 4 ? frame[4] : 0;
    uint8_t seq = len > 5 ? frame[5] : 0;
    uint16_t plen = len > 7 ? (uint16_t)(frame[6] | ((uint16_t)frame[7] << 8)) : 0;

    // One-liner decode
    char desc[128] = "";
    if (len >= 8) {
        const uint8_t *d = frame + 8;  // payload bytes (after header)
        switch (cmd) {
            // ── GD32 → ESP32 ───────────────────────────────────────────────
            case 0x02:
                snprintf(desc, sizeof(desc), "InfoSync SN=%.16s HW=%.12s FW=%.8s",
                         (const char *)d, (const char *)(d + 35), (const char *)(d + 83));
                break;
            case 0x03:
                snprintf(desc, sizeof(desc), "StatusUpdate sub=0x%02X evse=%d", d[0], d[1]);
                break;
            case 0x04:
                snprintf(desc, sizeof(desc), "Heartbeat evse=%d", d[0]);
                break;
            case 0x05:
                snprintf(desc, sizeof(desc), "RFIDCard %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                         d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);
                break;
            case 0x06:
                snprintf(desc, sizeof(desc), "RemoteStartAck flag=0x%02X (%s)",
                         d[64], d[64] == 0x40 ? "stop-ack" : "start-ack");
                break;
            case 0x07:
                snprintf(desc, sizeof(desc), "ChargingApproval buf[72]=0x%02X (%s)",
                         d[64], d[64] == 0 ? "start?" : "stop?");
                break;
            case 0x08:
                if (plen >= 70)
                    snprintf(desc, sizeof(desc),
                             "MeterData evse=%d P=%dW V=%d/%d/%dV I=%d/%d/%d A",
                             d[69], (int)get_u16(frame, 96),
                             (int)(get_u16(frame, 100)/10), (int)(get_u16(frame, 102)/10), (int)(get_u16(frame, 104)/10),
                             (int)(get_u16(frame, 106)/10), (int)(get_u16(frame, 108)/10), (int)(get_u16(frame, 110)/10));
                else
                    snprintf(desc, sizeof(desc), "MeterData (short plen=%d)", plen);
                break;
            case 0x09:
                snprintf(desc, sizeof(desc), "ChargingStopped reason=%d energy=%dWh",
                         d[69], (int)get_u16(frame, 96));
                break;
            case 0x0A:
                snprintf(desc, sizeof(desc), "CtrlAck sub=0x%02X type=0x%02X", d[1], d[0]);
                break;
            case 0x0C:
                snprintf(desc, sizeof(desc), "ACCtrlAck sub=0x%02X val=%d", d[1],
                         plen >= 4 ? (d[2] | (d[3] << 8)) : (plen >= 2 ? d[2] : 0));
                break;
            case 0x0D:
                snprintf(desc, sizeof(desc), "LimitAck");
                break;
            case 0x0E:
                snprintf(desc, sizeof(desc), "ClockAlignedExtData");
                break;
            case 0x0F:
                snprintf(desc, sizeof(desc), "ScheduleRequest gun=%d", plen > 0 ? d[0] : 0);
                break;
            // ── ESP32 → GD32 ───────────────────────────────────────────────
            case 0xA2:
                snprintf(desc, sizeof(desc), "InfoSyncAck");
                break;
            case 0xA3:
                snprintf(desc, sizeof(desc), "StatusAck time=20%02d-%02d-%02d %02d:%02d:%02d",
                         d[1],d[2],d[3],d[4],d[5],d[6]);
                break;
            case 0xA4:
                snprintf(desc, sizeof(desc), "HeartbeatAck time=20%02d-%02d-%02d %02d:%02d:%02d",
                         d[1],d[2],d[3],d[4],d[5],d[6]);
                break;
            case 0xA5:
                snprintf(desc, sizeof(desc), "CardAuthAck result=0x%02X", d[32]);
                break;
            case 0xA6:
                snprintf(desc, sizeof(desc), "RemoteTransaction flag=0x%02X (%s)",
                         d[64], d[64] == 0x30 ? "START" : (d[64] == 0x40 ? "STOP" : "?"));
                break;
            case 0xA7:
                snprintf(desc, sizeof(desc), "TransactionApprove flag=0x%02X (%s) txn=%.6s",
                         d[32], d[32] == 0x10 ? "STOP" : "START", (const char *)d);
                break;
            case 0xA8:
                snprintf(desc, sizeof(desc), "MeterAck time=20%02d-%02d-%02d %02d:%02d:%02d",
                         d[1],d[2],d[3],d[4],d[5],d[6]);
                break;
            case 0xA9:
                snprintf(desc, sizeof(desc), "TransactionAck");
                break;
            case 0xAA:
                if (plen >= 2) {
                    uint8_t sub = d[1];
                    uint16_t dlen = plen >= 4 ? (uint16_t)(d[2] | ((uint16_t)d[3] << 8)) : 0;
                    const char *type_s = (d[0] == 0x18) ? "SET" : "GET";
                    if (sub == 0x12)
                        snprintf(desc, sizeof(desc), "CtrlCmd %s SetReset reason=%d", type_s, dlen > 0 ? d[4] : 0);
                    else if (sub == 0x02 && d[0] == 0x18 && dlen == 6)
                        snprintf(desc, sizeof(desc), "CtrlCmd SET SetRTC 20%02d-%02d-%02d %02d:%02d:%02d",
                                 d[4],d[5],d[6],d[7],d[8],d[9]);
                    else if (sub == 0x52)
                        snprintf(desc, sizeof(desc), "CtrlCmd SET SetPhase phases=%d", dlen > 0 ? d[4] : 0);
                    else if (sub == 0x3E)
                        snprintf(desc, sizeof(desc), "CtrlCmd %s ClkAlignedInt=%ds", type_s,
                                 dlen >= 4 ? (int)(d[4]|(d[5]<<8)|(d[6]<<16)|(d[7]<<24)) : 0);
                    else if (sub == 0x08)
                        snprintf(desc, sizeof(desc), "CtrlCmd %s HBTimeout=%ds", type_s,
                                 dlen >= 2 ? (int)(d[4]|(d[5]<<8)) : 0);
                    else
                        snprintf(desc, sizeof(desc), "CtrlCmd %s sub=0x%02X dlen=%d", type_s, sub, dlen);
                }
                break;
            case 0xAC:
                if (plen >= 2) {
                    uint8_t sub = d[1];
                    uint16_t dlen = plen >= 4 ? (uint16_t)(d[2] | ((uint16_t)d[3] << 8)) : 0;
                    int val = dlen == 1 ? d[4] : (dlen >= 2 ? (int)(d[4]|(d[5]<<8)) : -1);
                    const char *names[] = {"","","","","","","","","MinCurr","S2OpenStop","S2OpenLock","RemoteStart","OfflineStop","OfflineEnergy"};
                    const char *name = sub < 14 ? names[sub] : "?";
                    snprintf(desc, sizeof(desc), "ACCtrl sub=0x%02X (%s) val=%d", sub, name, val);
                }
                break;
            case 0xAF:
                // payload[17] = amps; frame[8..] = payload[1..], so amps at d[16]
                snprintf(desc, sizeof(desc), "SmartCurrCtl limit=%dA", plen >= 17 ? d[16] : 0);
                break;
            default:
                snprintf(desc, sizeof(desc), "unknown");
                break;
        }
    }

    ESP_LOGD(TAG, "%s cmd=0x%02X seq=%d len=%d  %s", dir, cmd, seq, plen, desc);
    ESP_LOGV(TAG, "%s raw: %s", dir, hex);
}

// ── CRC-16/ARC: init=0x0000, poly=0xA001 (reflected 0x8005)
//    Matches crc16_modbus() in ac011k.cpp. Note: init=0x0000, NOT 0xFFFF.
static uint16_t privcomm_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001u) : (crc >> 1);
    }
    return crc;
}

// ── Frame parser states ───────────────────────────────────────────────────────
enum PcState : uint8_t {
    PC_MAGIC = 0, PC_VERSION, PC_ADDR, PC_CMD, PC_SEQ, PC_LEN, PC_PAYLOAD, PC_CRC
};

// ── Init-sequence command payloads (first byte = command code) ────────────────
static const uint8_t kSetRemoteStart[]        = {0xAC, 0x11, 0x0B, 0x01, 0x00, 0x00};
static const uint8_t kSetS2OpenStop[]         = {0xAC, 0x11, 0x09, 0x01, 0x00, 0x00};
static const uint8_t kSetS2OpenLock[]         = {0xAC, 0x11, 0x0A, 0x01, 0x00, 0x00};
static const uint8_t kSetOfflineStop[]        = {0xAC, 0x11, 0x0C, 0x01, 0x00, 0x00};
static const uint8_t kClockAlignedInterval[]  = {0xAA, 0x18, 0x3E, 0x04, 0x00, 10, 0, 0, 0};
static const uint8_t kSetOfflineEnergy[]      = {0xAC, 0x11, 0x0D, 0x04, 0x00, 0xB8, 0x0B, 0x00, 0x00};
static const uint8_t kSetGunTime[]            = {0xAA, 0x18, 0x3F, 0x04, 0x00, 30, 0, 0, 0};
static const uint8_t kSetSmartparam[]         = {0xAA, 0x18, 0x25, 0x0E, 0x00,
                                                  0x05,0,0,0, 0x05,0,0,0, 0,0x03,0,0,0, 0x02};
static const uint8_t kGetRtc[]               = {0xAA, 0x10, 0x02, 0x00, 0x00};
static const uint8_t kSetReset[]             = {0xAA, 0x18, 0x12, 0x01, 0x00, 3};
static const uint8_t kSetHbTimeout[]         = {0xAA, 0x18, 0x08, 0x02, 0x00, 240, 0x00}; // 240s
static const uint8_t kInit15[]              = {0xAA, 0x18, 0x09, 0x01, 0x00, 0x00};
static const uint8_t kClearChargingProfile[] = {0xAA, 0x18, 0x24, 0x05, 0x00,
                                                 0xFF,0xFF,0xFF,0xFF, 0x55};
static const uint8_t kGetMaxCurrLimit[]     = {0xAA, 0x10, 0x0B, 0x00, 0x00};

// ── Charging control ──────────────────────────────────────────────────────────
// StartChargingA6: WARP charger identity string + start flag 0x30 at byte 65
static const uint8_t kStartChargingA6[74] = {
    0xA6,
    'W','A','R','P',' ','c','h','a','r','g','e','r',' ','f','o','r',' ','E','N','+',
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0x30,  // start flag
    0,0,0,0,0,0,0,0
};

// StopChargingA6 template: bytes 33-38 = transaction number (ASCII), byte 65 = 0x40 stop flag
// Flag must be at template[65] so that d[64]=frame[72] == 0x40 (same offset as start flag 0x30).
static const uint8_t kStopChargingA6Template[75] = {
    0xA6,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    '0','0','0','0','0','0',  // bytes 33-38: transaction number (patched in setup())
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0x40,  // stop flag at template[65] → d[64]=frame[72]
    0,0,0,0,0,0,0,0,0
};

// TransactionAck: sent in response to charging-stop notification (cmd 0x09)
static const uint8_t kTransactionAck[34] = {
    0xA9,
    'W','A','R','P',' ','c','h','a','r','g','e','r',' ','f','o','r',' ','E','N','+',
    0,0,0,0,0,0,0,0,0,0,0,0,0
};

// CardAuthAck: 0x50 = unknown card (allow offline charging without RFID auth).
// Change byte 33 to 0x40 to accept all cards, 0xD0 to decline all.
static const uint8_t kCardAuthAck[38] = {
    0xA5,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0x50,
    0,0,0,0
};

// ── Component ─────────────────────────────────────────────────────────────────
class AC011KComponent : public Component, public uart::UARTDevice {
public:
    // ── Sensor setters (called from Python-generated registration code) ───────
    void set_power_sensor(sensor::Sensor *s)          { s_power_ = s; }
    void set_voltage_l1_sensor(sensor::Sensor *s)     { s_voltage_l1_ = s; }
    void set_voltage_l2_sensor(sensor::Sensor *s)     { s_voltage_l2_ = s; }
    void set_voltage_l3_sensor(sensor::Sensor *s)     { s_voltage_l3_ = s; }
    void set_current_l1_sensor(sensor::Sensor *s)     { s_current_l1_ = s; }
    void set_current_l2_sensor(sensor::Sensor *s)     { s_current_l2_ = s; }
    void set_current_l3_sensor(sensor::Sensor *s)     { s_current_l3_ = s; }
    void set_energy_session_sensor(sensor::Sensor *s) { s_energy_sess_ = s; }
    void set_energy_total_sensor(sensor::Sensor *s)   { s_energy_total_ = s; }
    void set_evse_status_sensor(sensor::Sensor *s)          { s_evse_status_ = s; }
    void set_evse_state_sensor(text_sensor::TextSensor *s)  { ts_evse_state_ = s; }
    void set_phases_sensor(sensor::Sensor *s)               { s_phases_ = s; }

    void set_plugged_binary_sensor(binary_sensor::BinarySensor *s)  { bs_plugged_ = s; }
    void set_charging_binary_sensor(binary_sensor::BinarySensor *s) { bs_charging_ = s; }

    // ── Public control API (call from YAML lambdas via id(ac011k_hub)) ────────

    // Start a charging session. Cable must be plugged in (EVSE status 2).
    // Only send A6 — do NOT send AF here. The GD32 will send a 0x0F ScheduleRequest
    // before sending 0x07 ChargingApproval, and we respond to that with AF.
    // (Warp firmware reference: bs_evse_start_charging(), FW >= 1.1.258 path)
    void start_charging() {
        ESP_LOGI(TAG, "start_charging()");
        send_frame(kStartChargingA6, sizeof(kStartChargingA6), seq_++);
    }

    // Stop an ongoing charging session.
    void stop_charging() {
        ESP_LOGI(TAG, "stop_charging()");
        send_frame(stop_a6_, sizeof(stop_a6_), seq_++);
    }

    // Set the current limit in amperes (6–16 A for AC011K hardware).
    // Sends 0xAF proactively with own seq (warp firmware: bs_evse_set_max_charging_current).
    // The GD32 also picks up the limit via the 0x0F ScheduleRequest handler below.
    void set_current_limit(uint8_t amps) {
        if (amps < 6)  amps = 6;
        if (amps > 16) amps = 16;
        current_limit_a_ = amps;
        ESP_LOGI(TAG, "set_current_limit(%d A)", amps);
        send_charging_limit(amps, seq_++);
    }

    uint8_t get_current_limit() const { return current_limit_a_; }

    // Switch between 1-phase and 3-phase charging (cmdAACtrlSetChgphase, AA 18 52).
    // NOTE: Phase switching does NOT work reliably. The GD32 seems to accept the
    // command (0x0A ack received) but then ignores it — the charger always operates
    // in 3-phase mode regardless. Kept here for future investigation.
    // DO NOT call this function from normal charging control paths.
    void set_phases(uint8_t phases) {
        if (phases != 1 && phases != 3) return;
        phases_ = phases;
        uint8_t cmd[6] = {0xAA, 0x18, 0x52, 0x01, 0x00, phases};
        send_frame(cmd, sizeof(cmd), seq_++);
        ESP_LOGI(TAG, "set_phases(%d) [WARNING: not functional on this hardware]", phases);
    }

    uint8_t get_phases() const { return phases_; }

    // ── ESPHome lifecycle ─────────────────────────────────────────────────────
    void setup() override {
        // Build mutable stop command with transaction number embedded as ASCII.
        memcpy(stop_a6_, kStopChargingA6Template, sizeof(stop_a6_));
        snprintf((char *)stop_a6_ + 33, 7, "%06d", (int)tx_num_);

        // StartChargingA7 / StopChargingA7: confirm approval from GD (cmd 0x07)
        memset(start_a7_, 0, sizeof(start_a7_));
        start_a7_[0] = 0xA7;
        snprintf((char *)start_a7_ + 1, 7, "%06d", (int)tx_num_);

        memset(stop_a7_, 0, sizeof(stop_a7_));
        stop_a7_[0]  = 0xA7;
        snprintf((char *)stop_a7_ + 1, 7, "%06d", (int)tx_num_);
        stop_a7_[33] = 0x10;  // stop confirmation flag

        ESP_LOGI(TAG, "Sending PrivComm init sequence (txnum=%d)", tx_num_);
        send_frame(kSetRemoteStart,       sizeof(kSetRemoteStart),       seq_++);
        send_frame(kSetS2OpenStop,        sizeof(kSetS2OpenStop),        seq_++);
        send_frame(kSetS2OpenLock,        sizeof(kSetS2OpenLock),        seq_++);
        send_frame(kSetOfflineStop,       sizeof(kSetOfflineStop),       seq_++);
        send_frame(kClockAlignedInterval, sizeof(kClockAlignedInterval), seq_++);
        send_frame(kSetOfflineEnergy,     sizeof(kSetOfflineEnergy),     seq_++);
        send_frame(kSetGunTime,           sizeof(kSetGunTime),           seq_++);
        send_frame(kSetSmartparam,        sizeof(kSetSmartparam),        seq_++);
        send_frame(kGetRtc,               sizeof(kGetRtc),               seq_++);
        send_frame(kSetReset,             sizeof(kSetReset),             seq_++);
        send_frame(kSetHbTimeout,         sizeof(kSetHbTimeout),         seq_++);
        send_frame(kInit15,               sizeof(kInit15),               seq_++);
        send_frame(kSetGunTime,           sizeof(kSetGunTime),           seq_++);  // sent twice in original
        send_frame(kClearChargingProfile, sizeof(kClearChargingProfile), seq_++);
        send_frame(kGetMaxCurrLimit,      sizeof(kGetMaxCurrLimit),      seq_++);
    }

    void loop() override {
        uint8_t b;
        while (available() > 0 && !frame_ready_) {
            if (!read_byte(&b)) break;
            rx_buf_[rx_ptr_++] = b;

            switch (rx_state_) {
                case PC_MAGIC:
                    rx_ptr_ = 1;
                    if (b == 0xFA) { rx_state_ = PC_VERSION; }
                    else { rx_ptr_ = 0; }
                    break;
                case PC_VERSION:
                    if (b == 0x03) { rx_state_ = PC_ADDR; }
                    else { rx_state_ = PC_MAGIC; rx_ptr_ = 0; }
                    break;
                case PC_ADDR:
                    if (b != 0x00) { rx_state_ = PC_MAGIC; rx_ptr_ = 0; break; }
                    if (rx_ptr_ == 4) rx_state_ = PC_CMD;
                    break;
                case PC_CMD:
                    rx_cmd_   = b;
                    rx_state_ = PC_SEQ;
                    break;
                case PC_SEQ:
                    rx_seq_   = b;
                    rx_state_ = PC_LEN;
                    break;
                case PC_LEN:
                    if (rx_ptr_ == 8) {
                        rx_len_   = get_u16(rx_buf_, 6);
                        rx_state_ = PC_PAYLOAD;
                    }
                    break;
                case PC_PAYLOAD:
                    if (rx_ptr_ == rx_len_ + 8u) {
                        rx_state_ = PC_CRC;
                    } else {
                        // Bug in some GD firmware: 0x09 packets can be truncated.
                        // Detect the start of a new frame arriving inside the payload.
                        if (rx_ptr_ >= 4 &&
                            rx_buf_[rx_ptr_-4] == 0xFA && rx_buf_[rx_ptr_-3] == 0x03 &&
                            rx_buf_[rx_ptr_-2] == 0x00 && rx_buf_[rx_ptr_-1] == 0x00) {
                            ESP_LOGW(TAG, "Truncated cmd_0x%02X, processing early", rx_cmd_);
                            frame_ready_ = true;
                            rx_state_ = PC_CMD;
                            rx_ptr_   = 4;
                        }
                    }
                    break;
                case PC_CRC:
                    if (rx_ptr_ == rx_len_ + 10u) {
                        uint16_t got  = get_u16(rx_buf_, rx_len_ + 8);
                        uint16_t calc = privcomm_crc16(rx_buf_, rx_len_ + 8);
                        if (got == calc) {
                            frame_ready_ = true;
                        } else {
                            ESP_LOGE(TAG, "CRC error cmd_0x%02X: got %04X calc %04X",
                                     rx_cmd_, got, calc);
                        }
                        rx_state_ = PC_MAGIC;
                    }
                    break;
            }

            if (rx_ptr_ >= sizeof(rx_buf_)) {
                ESP_LOGE(TAG, "RX buffer overflow, resetting parser");
                rx_state_ = PC_MAGIC;
                rx_ptr_   = 0;
            }
        }

        if (frame_ready_) {
            frame_ready_ = false;
            log_frame("RX", rx_buf_, rx_len_ + 10);
            handle_frame(rx_cmd_, rx_seq_, rx_buf_, rx_len_);
        }
    }

    float get_setup_priority() const override { return setup_priority::DATA; }

private:
    // ── Sensor pointers (nullptr = not configured in YAML) ────────────────────
    sensor::Sensor *s_power_       = nullptr;
    sensor::Sensor *s_voltage_l1_  = nullptr;
    sensor::Sensor *s_voltage_l2_  = nullptr;
    sensor::Sensor *s_voltage_l3_  = nullptr;
    sensor::Sensor *s_current_l1_  = nullptr;
    sensor::Sensor *s_current_l2_  = nullptr;
    sensor::Sensor *s_current_l3_  = nullptr;
    sensor::Sensor *s_energy_sess_ = nullptr;
    sensor::Sensor *s_energy_total_= nullptr;
    sensor::Sensor *s_evse_status_             = nullptr;
    sensor::Sensor *s_phases_                 = nullptr;
    text_sensor::TextSensor *ts_evse_state_   = nullptr;
    binary_sensor::BinarySensor *bs_plugged_  = nullptr;
    binary_sensor::BinarySensor *bs_charging_ = nullptr;

    // ── Protocol state ────────────────────────────────────────────────────────
    uint8_t  rx_buf_[1024] = {};
    uint16_t rx_ptr_       = 0;
    uint16_t rx_len_       = 0;
    uint8_t  rx_cmd_       = 0;
    uint8_t  rx_seq_       = 0;
    PcState  rx_state_     = PC_MAGIC;
    bool     frame_ready_  = false;

    uint8_t  seq_              = 1;      // outgoing sequence number
    uint8_t  current_limit_a_  = 16;    // amperes
    uint8_t  phases_           = 3;     // 1 or 3
    uint32_t tx_num_           = 100000; // transaction number (embedded in A6/A7 cmds)

    // Mutable command buffers with transaction number patched in at setup()
    uint8_t stop_a6_[75]  = {};
    uint8_t start_a7_[35] = {};
    uint8_t stop_a7_[35]  = {};

    // ── Frame dispatcher ──────────────────────────────────────────────────────
    void handle_frame(uint8_t cmd, uint8_t seq, const uint8_t *buf, uint16_t /*len*/) {
        switch (cmd) {

            case 0x02:  // SN / HW / FW info (triggered by kSetReset)
                // buf[8]  = serial number string
                // buf[43] = hardware model string  (e.g. "AC011K-AU-25")
                // buf[91] = firmware version string (e.g. "1.1.538")
                ESP_LOGI(TAG, "GD info  SN=%.32s  HW=%.32s  FW=%.16s",
                         (const char *)buf + 8,
                         (const char *)buf + 43,
                         (const char *)buf + 91);
                send_ack(cmd, seq);  // reply with 0xA2
                break;

            case 0x03:  // Status update from GD
                // buf[9] = EVSE status code (1-9)
                update_status(buf[9]);
                send_time_ack(0xA3, 0x10, 8, seq);
                break;

            case 0x04:  // Heartbeat / time request (~60s interval)
                // buf[8] = EVSE status code
                update_status(buf[8]);
                send_time_ack(0xA4, 0x01, 8, seq);
                break;

            case 0x05:  // RFID card presented
                ESP_LOGI(TAG, "RFID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                         buf[8],buf[9],buf[10],buf[11],buf[12],buf[13],buf[14],buf[15]);
                send_frame(kCardAuthAck, sizeof(kCardAuthAck), seq);
                break;

            case 0x06:  // Ack for our A6 start/stop request
                ESP_LOGD(TAG, "A6 %s ack", (buf[72] == 0x40) ? "stop" : "start");
                break;

            case 0x07:  // GD asking: "May I proceed with start/stop?"
                if (buf[72] == 0) {
                    ESP_LOGI(TAG, "Start charging approval -> A7 confirm");
                    send_frame(start_a7_, sizeof(start_a7_), seq);
                } else {
                    ESP_LOGI(TAG, "Stop charging approval -> A7 confirm");
                    send_frame(stop_a7_, sizeof(stop_a7_), seq);
                }
                break;

            case 0x08: {  // ClockAlignedData — meter readings, ~10s interval
                // buf[77] = EVSE status (< 10 = valid status message)
                if (buf[77] < 10) {
                    update_status(buf[77]);
                    publish_meter(buf);
                }
                send_time_ack(0xA8, 0x40, 12, seq);
                break;
            }

            case 0x09:  // Charging session ended
                ESP_LOGI(TAG, "Charging stopped, reason=%d  meter=%d Wh",
                         buf[77], get_u16(buf, 96));
                send_frame(kTransactionAck, sizeof(kTransactionAck), seq);
                break;

            case 0x0A:  // Ack for AA control commands (config, time, etc.)
                ESP_LOGD(TAG, "AA ctrl ack type=0x%02X", buf[9]);
                break;

            case 0x0C:  // Ack for AC control commands
                ESP_LOGD(TAG, "AC ctrl ack");
                break;

            case 0x0D:  // Ack for charging limit (AD/AF) commands
                ESP_LOGD(TAG, "Charging limit ack");
                break;

            case 0x0E:  // ClockAlignedExtData — extended meter/session report (no reply required)
                ESP_LOGD(TAG, "ClockAlignedExtData (extended meter block)");
                break;

            case 0x0F:  // GD requests current charging schedule — reply with received seq
                // Warp firmware: sendChargingLimit1(allowed_charging_current, seq)
                // Must mirror back the received seq, NOT use own seq++
                ESP_LOGD(TAG, "GD requests charging schedule, replying %d A", current_limit_a_);
                send_charging_limit(current_limit_a_, seq);
                break;

            default:
                ESP_LOGD(TAG, "Unhandled cmd=0x%02X seq=0x%02X len=%d", cmd, seq, rx_len_);
                break;
        }
    }

    // ── Status → sensor/binary_sensor updates ────────────────────────────────
    // EVSE status codes:
    //   1=Available  2=Preparing(plugged)  3=Charging  4=Suspended by charger
    //   5=Suspended by EV  6=Finishing  7=Reserved  8=Unavailable  9=Fault
    void update_status(uint8_t status) {
        if (s_evse_status_ != nullptr)
            s_evse_status_->publish_state(status);
        if (ts_evse_state_ != nullptr) {
            const char *names[] = {"", "Available", "Preparing", "Charging",
                                   "Suspended by Charger", "Suspended by EV",
                                   "Finishing", "Reserved", "Unavailable", "Fault"};
            ts_evse_state_->publish_state(status < 10 ? names[status] : "Unknown");
        }
        if (bs_plugged_ != nullptr)
            bs_plugged_->publish_state(status >= 2 && status <= 6);
        if (bs_charging_ != nullptr)
            bs_charging_->publish_state(status == 3);
    }

    // ── Meter data from cmd 0x08 ──────────────────────────────────────────────
    // All multi-byte values are little-endian uint16.
    // Voltages and currents are stored ×10 in the protocol.
    //
    // Byte offsets (from start of raw RX frame, FA=0):
    //   [77]      EVSE status
    //   [84..85]  Session energy (Wh)       → kWh
    //   [88..89]  Total energy (Wh)         → kWh (resets on GD reboot)
    //   [96..97]  Charging power (W)
    //   [100..101] L1 voltage ×10
    //   [102..103] L2 voltage ×10
    //   [104..105] L3 voltage ×10
    //   [106..107] L1 current ×10
    //   [108..109] L2 current ×10
    //   [110..111] L3 current ×10
    //   [113..114] Charging time (minutes)
    void publish_meter(const uint8_t *buf) {
        if (s_power_       != nullptr) s_power_       ->publish_state(get_u16(buf, 96));
        if (s_voltage_l1_  != nullptr) s_voltage_l1_  ->publish_state(get_u16(buf, 100) / 10.0f);
        if (s_voltage_l2_  != nullptr) s_voltage_l2_  ->publish_state(get_u16(buf, 102) / 10.0f);
        if (s_voltage_l3_  != nullptr) s_voltage_l3_  ->publish_state(get_u16(buf, 104) / 10.0f);
        if (s_current_l1_  != nullptr) s_current_l1_  ->publish_state(get_u16(buf, 106) / 10.0f);
        if (s_current_l2_  != nullptr) s_current_l2_  ->publish_state(get_u16(buf, 108) / 10.0f);
        if (s_current_l3_  != nullptr) s_current_l3_  ->publish_state(get_u16(buf, 110) / 10.0f);
        if (s_energy_sess_ != nullptr) s_energy_sess_ ->publish_state(get_u16(buf, 84)  / 1000.0f);
        if (s_energy_total_!= nullptr) s_energy_total_->publish_state(get_u16(buf, 88)  / 1000.0f);
        // Count active phases from voltages. Only updates during active charging.
        {
            uint8_t p = 0;
            if (get_u16(buf, 100) > 700)  p++;  // L1
            if (get_u16(buf, 102) > 2100) p++;  // L2
            if (get_u16(buf, 104) > 2100) p++;  // L3
            if (p > 0) {
                phases_ = p;
                if (s_phases_ != nullptr) s_phases_->publish_state(p);
            }
        }
    }

    // ── ChargingLimit via cmd 0xAF (firmware >= 1.1.258, all current hardware) ─
    // seq: use received seq for 0x0F replies; use seq_++ for proactive sends.
    void send_charging_limit(uint8_t amps, uint8_t seq) {
        uint8_t pl[286] = {};
        pl[0]  = 0xAF;
        pl[1]  = 0x00;
        fill_time(&pl[2]);
        pl[8]  = 0x80;
        pl[9]  = 0x51;
        pl[10] = 0x01;
        pl[11] = 0x00;
        pl[12] = 0x01;
        pl[17] = amps;
        send_frame(pl, sizeof(pl), seq);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    // Fill 6 bytes with UTC time in GD format: year-2000, month, day, hour, min, sec.
    void fill_time(uint8_t *dt) {
        time_t now_t = ::time(nullptr);
        if (now_t > 1000000000L) {
            struct tm t;
            gmtime_r(&now_t, &t);
            dt[0] = (uint8_t)(t.tm_year - 100);
            dt[1] = (uint8_t)(t.tm_mon + 1);
            dt[2] = (uint8_t)(t.tm_mday);
            dt[3] = (uint8_t)(t.tm_hour);
            dt[4] = (uint8_t)(t.tm_min);
            dt[5] = (uint8_t)(t.tm_sec);
        } else {
            // NTP not yet synced; use a plausible fallback
            dt[0] = 25; dt[1] = 1; dt[2] = 1;
            dt[3] =  0; dt[4] = 0; dt[5] = 0;
        }
    }

    // Build and write a complete PrivComm frame.
    //
    // payload layout: [0]=cmd  [1..size-1]=data
    // Frame:  FA 03 00 00  [cmd]  [seq]  [len_lo len_hi]  [data...]  [crc_lo crc_hi]
    // len field  = size - 1  (data bytes, excluding the cmd byte)
    // CRC covers bytes 0..(size+6) = size+7 bytes total before the CRC
    // Total frame = size + 9 bytes
    void send_frame(const uint8_t *payload, size_t size, uint8_t seq) {
        uint8_t frame[512];
        frame[0] = 0xFA;
        frame[1] = 0x03;
        frame[2] = 0x00;
        frame[3] = 0x00;
        frame[4] = payload[0];
        frame[5] = seq;
        uint16_t plen = (uint16_t)(size - 1);
        frame[6] = plen & 0xFF;
        frame[7] = plen >> 8;
        if (size > 1)
            memcpy(frame + 8, payload + 1, size - 1);
        uint16_t crc = privcomm_crc16(frame, size + 7);
        frame[size + 7] = crc & 0xFF;
        frame[size + 8] = crc >> 8;
        log_frame("TX", frame, size + 9);
        write_array(frame, size + 9);
    }

    // Simple ACK for cmd 0x02 (info sync): FA 03 00 00 [cmd^0xA0] [seq] 01 00 00 [crc]
    void send_ack(uint8_t cmd, uint8_t seq) {
        uint8_t frame[11] = {0xFA, 0x03, 0x00, 0x00,
                              (uint8_t)(cmd ^ 0xA0u), seq,
                              0x01, 0x00, 0x00, 0x00, 0x00};
        uint16_t crc = privcomm_crc16(frame, 9);
        frame[9]  = crc & 0xFF;
        frame[10] = crc >> 8;
        write_array(frame, 11);
    }

    // Time ACK for heartbeat (0x04→0xA4), status (0x03→0xA3), power data (0x08→0xA8).
    // payload_len = 8  → {cmd, action, y, m, d, H, M, S}
    // payload_len = 12 → same + four zero bytes (used for 0x08 ack)
    void send_time_ack(uint8_t cmd, uint8_t action, uint8_t payload_len, uint8_t seq) {
        uint8_t pl[12] = {};
        pl[0] = cmd;
        pl[1] = action;
        fill_time(&pl[2]);
        send_frame(pl, payload_len, seq);
    }
};

}  // namespace ac011k
}  // namespace esphome
