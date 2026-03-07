# PrivComm Protocol Specification

**AC011K Wallbox — ESP32 ↔ GD32 Serial Communication**

This document describes the proprietary binary serial protocol ("PrivComm") used between
the ESP32 application processor and the GD32 charging controller on the EN+ / Autoaid
AC011K wallbox hardware.

---

## 1. Physical Layer

| Parameter | Value |
|-----------|-------|
| Interface | UART (Serial2 on ESP32) |
| RX pin (ESP32 input) | GPIO 26 |
| TX pin (ESP32 output) | GPIO 27 |
| Baud rate | 115200 |
| Frame format | 8N1 |
| RX buffer (ESP32) | 1024 bytes |

---

## 2. Frame Format

Every message — in both directions — follows the same envelope:

```
Offset  Size  Field
──────  ────  ──────────────────────────────────────────────────
  0      1    Magic      = 0xFA  (always)
  1      1    Version    = 0x03  (always)
  2      2    Address    = 0x0000  (always, little-endian)
  4      1    CMD        command code
  5      1    SEQ        sequence number (increments per sender)
  6      2    LEN        payload length in bytes, little-endian
                         = (total data bytes) - 1
                         i.e. number of bytes AFTER the CMD byte
  8    LEN+1  DATA       CMD byte is NOT counted in LEN;
                         bytes [8 .. 8+LEN] are the payload
 8+LEN+1  2   CRC        CRC-16 over bytes [0 .. 8+LEN], little-endian
```

Total frame length = `LEN + 10` bytes
(4 header + 1 CMD + 1 SEQ + 2 LEN + (LEN+1) data - 1 CMD overlap + 2 CRC → simplifies to LEN+10)

> **Note on LEN:** The firmware defines `LEN = datasize - 1` where `datasize` is the full
> payload array length including the leading CMD byte. So for a payload of N bytes
> (including CMD), `LEN = N - 1`.

### 2.1 CRC Algorithm

**CRC-16/ARC** (also called CRC-16/IBM or CRC-16/LHA):

| Parameter | Value |
|-----------|-------|
| Init value | `0x0000` |
| Polynomial | `0xA001` (bit-reversed 0x8005) |
| Input reflection | yes |
| Output reflection | yes |
| Final XOR | none |

> This is **not** standard Modbus CRC (which uses init=0xFFFF).
> Despite the function being named `crc16_modbus` in the source, it uses init=0x0000.

```c
uint16_t privcomm_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001u) : (crc >> 1);
    }
    return crc;
}
```

CRC covers **all bytes from offset 0 through the last data byte** (i.e. `LEN + 8` bytes),
and is appended as two bytes in little-endian order.

### 2.2 Direction Convention

The **GD32 always initiates** frames. The ESP32 only sends in response to a received frame,
using the same SEQ number as the incoming frame (unless it is proactively sending a control
command). Proactive ESP32→GD32 commands use a locally-maintained, incrementing sequence
number starting at 1.

---

## 3. Command Overview

### 3.1 GD32 → ESP32 (Unsolicited)

| CMD  | Name | Description | ESP32 Response |
|------|------|-------------|----------------|
| 0x02 | InfoSync | GD32 boots / resets; sends SN, HW model, FW version | 0xA2 simple ACK |
| 0x03 | StatusUpdate | EVSE status change event | 0xA3 + UTC time |
| 0x04 | Heartbeat | ~60 s keep-alive; includes current EVSE status | 0xA4 + UTC time |
| 0x05 | RFIDCard | NFC/RFID card presented at reader | 0xA5 card auth ACK |
| 0x06 | RemoteStartAck | GD32 acknowledged our 0xA6 start/stop request | (no reply) |
| 0x07 | ChargingApproval | GD32 asks "may I start/stop?" | 0xA7 approve |
| 0x08 | MeterData | Energy meter snapshot, ~10 s interval | 0xA8 + UTC time |
| 0x09 | ChargingStopped | Session ended (remote stop or EV disconnect) | 0xA9 transaction ACK |
| 0x0A | CtrlAck | Response to 0xAA config/control commands | (logged only) |
| 0x0B | UpdateAck | Response to 0xAB GD firmware flash commands | next flash step |
| 0x0C | ACCtrlAck | Response to 0xAC config commands | (logged only) |
| 0x0D | LimitAck | Response to 0xAD / 0xAF current-limit commands | (logged only) |
| 0x0E | ChargingParamRpt | GD32 reports charging parameters | (logged only) |
| 0x0F | ScheduleRequest | GD32 asks for current charging schedule | 0xAF charging limit |

### 3.2 ESP32 → GD32 (Commands)

| CMD  | Name | Description |
|------|------|-------------|
| 0xA2 | InfoSyncAck | Simple ACK for 0x02 |
| 0xA3 | StatusAck | UTC time in response to 0x03 |
| 0xA4 | HeartbeatAck | UTC time in response to 0x04 |
| 0xA5 | CardAuthAck | RFID card authorization result |
| 0xA6 | RemoteTransaction | Request start or stop of a charging session |
| 0xA7 | TransactionApprove | Confirm GD32's start/stop approval |
| 0xA8 | MeterAck | UTC time in response to 0x08 |
| 0xA9 | TransactionAck | Acknowledge session-ended notification |
| 0xAA | CtrlCommand | Configuration and control (various sub-commands) |
| 0xAB | UpdateCommand | GD32 firmware flash operations |
| 0xAC | ACCtrlCommand | AC hardware control settings |
| 0xAD | SmartChargeCtl | OCPP-style charging profile (legacy, FW < 1.1.258) |
| 0xAF | SmartCurrCtl | Direct current limit command (FW >= 1.1.258) |

---

## 4. Received Frame Details (GD32 → ESP32)

All byte offsets are absolute positions within the raw receive buffer (0 = first byte = 0xFA).

### 4.1 CMD 0x02 — InfoSync (SN / HW / FW)

Sent spontaneously by GD32 after every reset or boot. Triggered by sending `SetReset`
(0xAA sub-command 0x12).

| Offset | Content |
|--------|---------|
| 8..39  | Serial number (null-terminated ASCII, 32 bytes) |
| 43..74 | Hardware model string, e.g. `AC011K-AU-25` (32 bytes) |
| 91..106| GD firmware version string, e.g. `1.1.538` (16 bytes) |

Known hardware models: `AC011K-AU-25`, `AC011K-AE-25`, `AC011K-AU-25-STL`, `AC011K-AE-25-STL`
Known firmware versions (working): `1.0.1435`, `1.1.27`, `1.1.212`, `1.1.258`, `1.1.460`,
`1.1.525`, `1.1.538`, `1.1.653`, `1.1.805`, `1.1.812`, `1.1.888`, `1.2.653`

**ESP32 must reply with 0xA2 ACK** (see §5.1).

Example raw frame:
```
FA 03 00 00  02  26  7D 00
53 4E 31 30 30 35 32 31 30 31 31 39 33 35 37 30   ; SN: "SN10052101193570"
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 24 D1 00
41 43 30 31 31 4B 2D 41 55 2D 32 35              ; HW: "AC011K-AU-25"
00 ...
31 2E 31 2E 32 37 00 ...                         ; FW: "1.1.27"
...
D9 25   ; CRC
```

### 4.2 CMD 0x03 — StatusUpdate

Sent whenever EVSE status changes.

| Offset | Content |
|--------|---------|
| 8 | Sub-status byte (purpose not fully determined) |
| 9 | **EVSE status code** (see §6) |
| 22..27 | Timestamp: year-2000, month, day, hour, min, sec |

**ESP32 must reply with 0xA3** (`sendTime(0xA3, 0x10, 8, seq)`).

### 4.3 CMD 0x04 — Heartbeat

Sent approximately every 60 seconds.

| Offset | Content |
|--------|---------|
| 8 | **EVSE status code** (see §6) |
| 12 | Extra value (purpose unknown) |

**ESP32 must reply with 0xA4** (`sendTime(0xA4, 0x01, 8, seq)`).

### 4.4 CMD 0x05 — RFID Card

| Offset | Content |
|--------|---------|
| 8..15 | Card UID, 8 bytes (each byte printed as 2 hex chars) |

**ESP32 must reply with 0xA5 CardAuthAck** (see §5.4).

### 4.5 CMD 0x06 — Remote Transaction Ack

GD32's acknowledgement of our 0xA6 start/stop request.

| Offset | Content |
|--------|---------|
| 72 | 0x40 = stop was acknowledged, 0x30 = start was acknowledged |

No reply required.

### 4.6 CMD 0x07 — Charging Approval

GD32 asks for permission to proceed with starting or stopping.

| Offset | Content |
|--------|---------|
| 72 | 0x00 = requesting start approval, non-zero = requesting stop approval |

**ESP32 must reply with 0xA7** using the matching start or stop approve frame (see §5.5).

### 4.7 CMD 0x08 — Meter Data (ClockAlignedData)

Sent every ~10 seconds (configurable via `ClockAlignedDataInterval`).
Only valid if `buf[77] < 10` (otherwise it is an RFID-type sub-message).

| Offset | Content | Unit | Scale |
|--------|---------|------|-------|
| 77 | EVSE status code | — | 1 |
| 84..85 | Session energy | Wh | raw ÷ 1000 = kWh |
| 86..87 | (unknown) | — | — |
| 88..89 | Total energy (resets on GD reboot) | Wh | raw ÷ 1000 = kWh |
| 90..91 | (unknown) | — | — |
| 92..93 | (unknown) | — | — |
| 94..95 | (unknown) | — | — |
| 96..97 | Charging power | W | 1 |
| 98..99 | (unknown) | — | — |
| 100..101 | L1 voltage | V | raw ÷ 10 |
| 102..103 | L2 voltage | V | raw ÷ 10 |
| 104..105 | L3 voltage | V | raw ÷ 10 |
| 106..107 | L1 current | A | raw ÷ 10 |
| 108..109 | L2 current | A | raw ÷ 10 |
| 110..111 | L3 current | A | raw ÷ 10 |
| 113..114 | Charging time | min | 1 |

All multi-byte values are **little-endian uint16**.

A voltage > 70 V (raw > 700) is used to determine that a phase is physically connected.

**ESP32 must reply with 0xA8** (`sendTime(0xA8, 0x40, 12, seq)`).

### 4.8 CMD 0x09 — Charging Stopped

| Offset | Content |
|--------|---------|
| 77 | Stop reason: 1 = Remote, 3 = EV disconnected |
| 78..79 | (unknown uint16) |
| 80..85 | Start timestamp: year-2000, month, day, hour, min, sec |
| 86..91 | Stop timestamp: year-2000, month, day, hour, min, sec |
| 92..95 | (unknown) |
| 96..97 | Final meter value (Wh) |

> **Known firmware bug:** Some GD firmware versions send 0x09 frames with a truncated
> payload and no CRC. The parser must detect an embedded next-frame magic `FA 03 00 00`
> to recover.

**ESP32 must reply with 0xA9 TransactionAck** (see §5.6).

### 4.9 CMD 0x0A — Control Command Ack

Response to 0xAA commands. Sub-type at `buf[9]`:

| Sub-type | Meaning |
|----------|---------|
| 0x02 | Time answer (GetRTC / SetRTC response); RTC data at buf[12..17] |
| 0x08 | Heartbeat timeout confirmation; timeout value at buf[12..13] |
| 0x09 | Start power mode set confirmation |
| 0x0B | GetMaxCurrLimit answer; max current at buf[12] (e.g. 160 = 16.0 A) |
| 0x12 | SetReset confirmation |
| 0x24 | ClearChargingProfile confirmation |
| 0x25 | SetSmartparam confirmation |
| 0x3E | ClockAlignedDataInterval confirmation; interval at buf[12..13] |
| 0x3F | SetGunTime confirmation |

### 4.10 CMD 0x0F — Schedule Request

GD32 asks the ESP32 for the current charging schedule. Sent after receiving an 0xAD
smart-charge command.

**ESP32 must reply with 0xAF** (see §5.9).

---

## 5. Sent Frame Details (ESP32 → GD32)

### 5.1 CMD 0xA2 — InfoSync ACK

Simple one-byte-payload ACK. Used for commands where no data needs to be returned.

```
Payload (1 byte): 0x00
Full frame: FA 03 00 00  A2  [seq]  01 00  00  [crc_lo crc_hi]
```

The same pattern applies to all simple ACKs: payload = `{cmd XOR 0xA0, seq, 0x01, 0x00, 0x00}`.

### 5.2 CMD 0xA3 — Status ACK

Reply to CMD 0x03. Carries UTC time so the GD32 can stay in sync.

```
Payload (8 bytes):
  [0] = 0xA3   (cmd)
  [1] = 0x10   (action)
  [2] = year - 2000
  [3] = month (1-12)
  [4] = day
  [5] = hour
  [6] = minute
  [7] = second
```

### 5.3 CMD 0xA4 — Heartbeat ACK

Reply to CMD 0x04. Same structure as 0xA3 but with action byte 0x01.

```
Payload (8 bytes):
  [0] = 0xA4
  [1] = 0x01
  [2..7] = UTC time (year-2000, month, day, hour, min, sec)
```

### 5.4 CMD 0xA5 — Card Auth ACK

Reply to CMD 0x05 (RFID card presented).

```
Payload (38 bytes):
  [0]  = 0xA5
  [1..31] = 0x00 (padding)
  [32] = auth result:
           0x40 = accept all cards
           0x50 = unknown card (allow offline charging without RFID auth)
           0xD0 = decline all cards
  [33..37] = 0x00 (padding)
```

For offline operation without RFID enforcement, use `0x50`.

### 5.5 CMD 0xA6 — Remote Transaction Request

Sent by ESP32 to initiate or terminate a charging session.

**Start charging (74 bytes):**

```
[0]     = 0xA6
[1..20] = Identity string "WARP charger for EN+" (ASCII, 20 chars)
[21..64]= 0x00 (padding, 44 bytes)
[65]    = 0x30  (start flag)
[66..73]= 0x00 (padding)
```

**Stop charging (75 bytes):**

```
[0]     = 0xA6
[1..32] = 0x00 (padding, 32 bytes)
[33..38]= transaction number as zero-padded 6-digit ASCII, e.g. "100000"
[39..64]= 0x00 (padding, 26 bytes)
[65]    = 0x40  (stop flag)
[66..74]= 0x00 (padding)
```

The transaction number must match the one used in the corresponding 0xA7 frames.

### 5.6 CMD 0xA7 — Transaction Approve

Reply to CMD 0x07 (GD32 asking for charging approval).

**Start approve (35 bytes):**

```
[0]     = 0xA7
[1..6]  = transaction number as zero-padded 6-digit ASCII, e.g. "100000"
[7..34] = 0x00
```

**Stop approve (35 bytes):**

```
[0]     = 0xA7
[1..6]  = transaction number as zero-padded 6-digit ASCII
[7..32] = 0x00
[33]    = 0x10  (stop confirmation flag)
[34]    = 0x00
```

### 5.7 CMD 0xA8 — Meter Data ACK

Reply to CMD 0x08. Carries UTC time. Uses a 12-byte payload (4 extra zero bytes vs. 0xA3/0xA4).

```
Payload (12 bytes):
  [0] = 0xA8
  [1] = 0x40   (action)
  [2..7] = UTC time (year-2000, month, day, hour, min, sec)
  [8..11] = 0x00
```

### 5.8 CMD 0xA9 — Transaction ACK

Reply to CMD 0x09 (session ended).

```
Payload (34 bytes):
  [0]     = 0xA9
  [1..20] = Identity string "WARP charger for EN+" (ASCII)
  [21..33]= 0x00
```

### 5.9 CMD 0xAF — Charging Current Limit (FW >= 1.1.258)

Sets the maximum allowed charging current. Supported on all current hardware.
Also sent in response to CMD 0x0F (schedule request).

```
Payload (286 bytes):
  [0]    = 0xAF
  [1]    = 0x00
  [2..7] = UTC time (year-2000, month, day, hour, min, sec)
  [8]    = 0x80
  [9]    = 0x51
  [10]   = 0x01
  [11]   = 0x00
  [12]   = 0x01
  [13..16]= 0x00
  [17]   = current limit in amperes (e.g. 16 for 16 A)
  [18..285]= 0x00
```

The minimum valid current is 6 A; maximum is 16 A for the AC011K hardware.

### 5.10 CMD 0xAD — Smart Charge Control (FW < 1.1.258, legacy)

Two variants exist in the original firmware (`sendChargingLimit2` and `sendChargingLimit3`).
Only relevant for the very early GD firmware version 1.1.212. All current hardware uses
0xAF instead.

### 5.11 CMD 0xAA — Configuration / Control Commands

General-purpose command for configuring GD32 parameters. Sub-commands are embedded in the
payload starting at offset 8 in the full frame. The init sequence (sent on every ESP32 boot)
uses the following sub-commands in order:

| Payload | Description |
|---------|-------------|
| `AC 11 0B 01 00 00` | SetRemoteStart = 0 (disable remote start via OCPP server) |
| `AC 11 09 01 00 00` | SetS2OpenStop = 0 |
| `AC 11 0A 01 00 00` | SetS2OpenLock = 0 |
| `AC 11 0C 01 00 00` | SetOfflineStop = 0 |
| `AA 18 3E 04 00 0A 00 00 00` | ClockAlignedDataInterval = 10 s |
| `AC 11 0D 04 00 B8 0B 00 00` | SetOfflineEnergy = 3000 Wh |
| `AA 18 3F 04 00 1E 00 00 00` | SetGunTime = 30 s |
| `AA 18 25 0E 00 05 00 00 00 05 00 00 00 00 03 00 00 00 02` | SetSmartparam |
| `AA 10 02 00 00` | GetRTC |
| `AA 18 12 01 00 03` | SetReset (triggers 0x02 InfoSync from GD32) |
| `AA 18 08 02 00 F0 00` | SetHeartbeatTimeout = 240 s |
| `AA 18 09 01 00 00` | Init15 (set start power mode) |
| `AA 18 3F 04 00 1E 00 00 00` | SetGunTime (sent twice) |
| `AA 18 24 05 00 FF FF FF FF 55` | ClearChargingProfile (connectorId=0) |
| `AA 10 0B 00 00` | GetMaxCurrLimit |

---

## 6. EVSE Status Codes

The GD32 reports a status code in commands 0x03, 0x04, and 0x08:

| Code | Name | IEC 61851 State | Charger State |
|------|------|-----------------|---------------|
| 1 | Available | A (not connected) | Not plugged in |
| 2 | Preparing | B (connected, not charging) | Waiting for release |
| 3 | Charging | C (charging) | Charging |
| 4 | Suspended by charger | B | Ready to charge |
| 5 | Suspended by EV | B | Ready to charge |
| 6 | Finishing | B | Waiting for release |
| 7 | Reserved | EF (error) | Error |
| 8 | Unavailable | EF (error) | Error |
| 9 | Fault | EF (error) | Error |

---

## 7. Startup / Init Sequence

On every ESP32 boot, the following must be done once before normal operation:

1. Open Serial2 at 115200 8N1, RX=GPIO26, TX=GPIO27.
2. Send all `0xAA` / `0xAC` init commands listed in §5.11 (in order).
3. The `SetReset` command triggers the GD32 to send a `0x02 InfoSync` frame.
4. Extract serial number, hardware model and firmware version from `0x02`.
5. Patch the transaction number into the A6/A7 command buffers (`sprintf("%06d", txNum)`).
6. Begin normal receive loop.

---

## 8. Charging Session Flow

### 8.1 Start Charging

```
ESP32                                GD32
  │                                   │
  │   [cable plugged → status=2]      │
  │ <──── 0x03 StatusUpdate ─────────┤
  │ ────► 0xA3 + UTC time ───────────►│
  │                                   │
  │ ────► 0xA6 StartChargingA6 ──────►│  (flag byte = 0x30)
  │ ────► 0xAF ChargingLimit ─────────►│  (set current limit)
  │                                   │
  │ <──── 0x06 RemoteStart Ack ───────┤
  │                                   │
  │ <──── 0x07 ChargingApproval ──────┤  (buf[72]=0x00 = start)
  │ ────► 0xA7 StartApprove ─────────►│
  │                                   │
  │ <──── 0x03 StatusUpdate ──────────┤  (status=3 Charging)
  │ ────► 0xA3 + UTC time ───────────►│
  │                                   │
  │ <──── 0x08 MeterData ─────────────┤  (every ~10 s)
  │ ────► 0xA8 + UTC time ───────────►│
```

### 8.2 Stop Charging

```
ESP32                                GD32
  │                                   │
  │ ────► 0xA6 StopChargingA6 ───────►│  (flag byte = 0x40)
  │                                   │
  │ <──── 0x06 RemoteStop Ack ────────┤
  │                                   │
  │ <──── 0x07 ChargingApproval ──────┤  (buf[72]!=0 = stop)
  │ ────► 0xA7 StopApprove ──────────►│
  │                                   │
  │ <──── 0x09 ChargingStopped ───────┤
  │ ────► 0xA9 TransactionAck ───────►│
  │                                   │
  │ <──── 0x03 StatusUpdate ──────────┤  (status=1 or 2)
  │ ────► 0xA3 + UTC time ───────────►│
```

### 8.3 Change Current Limit During Charging

Send `0xAF SmartCurrCtl` at any time with the new ampere value in payload byte `[17]`.
The GD32 responds with `0x0D LimitAck` and then `0x0F ScheduleRequest`, which triggers
another 0xAF reply.

---

## 9. Transaction Number

A transaction number is maintained by the ESP32 (initial value: 100000). It is embedded
as a 6-character zero-padded ASCII string into:

- `0xA6 StopChargingA6` at bytes `[33..38]`
- `0xA7 StartChargingA7` at bytes `[1..6]`
- `0xA7 StopChargingA7` at bytes `[1..6]`

This number identifies the current session. In the original firmware it is stored
persistently and incremented per session. For simple ESPHome use a fixed value (e.g. 100000)
works fine since the GD32 does not validate it strictly.

---

## 10. UTC Time Format

Several commands embed a 6-byte UTC timestamp:

| Offset | Field | Notes |
|--------|-------|-------|
| +0 | Year | years since 2000 (e.g. 26 for 2026) |
| +1 | Month | 1–12 |
| +2 | Day | 1–31 |
| +3 | Hour | 0–23 |
| +4 | Minute | 0–59 |
| +5 | Second | 0–59 |

If NTP is not yet synced, a plausible fallback (e.g. 2022-01-01 00:00:00) must be used —
the GD32 will reject obviously invalid timestamps.

---

## 11. GPIO Reference (AC011K Board)

| Function | GPIO | Logic |
|----------|------|-------|
| Green LED | GPIO 25 | Active LOW |
| Red LED | GPIO 33 | Active LOW |
| Button SW3 | GPIO 32 | Active LOW (pull-up) |
| UART RX (from GD32) | GPIO 26 | — |
| UART TX (to GD32) | GPIO 27 | — |

---

## 12. Known Quirks and Edge Cases

1. **Truncated 0x09 frames:** Some GD firmware versions send `0x09 ChargingStopped`
   without a complete payload and without a valid CRC. The parser must detect a new
   frame header (`FA 03 00 00`) appearing mid-payload and process the current (partial)
   frame early.

2. **CRC init = 0x0000:** Despite the function name `crc16_modbus`, the init value is
   `0x0000` (not `0xFFFF`). Using the wrong init will cause all CRC checks to fail.

3. **Firmware-version-dependent start command:**
   - FW 1.1.212: Send `0xA6` StartChargingA6 then `0xAD` ChargingLimit3 (legacy).
   - FW >= 1.1.258 (all current hardware): Send `0xA6` StartChargingA6 then `0xAF` ChargingLimit.

4. **Reset-to-full-current on disconnect (FW 1.1.212 only):** After a session ends,
   send `ChargingLimit3` at 16 A to restore the full current range for the next session.

5. **0x08 sub-message:** If `buf[77] >= 10` the frame is not meter data but a different
   sub-type. Value `0x10` contains an RFID card number at bytes `[40..47]`.

6. **LEN field:** `LEN = (payload array length) - 1`. The CMD byte is included in the
   payload array but excluded from LEN. This means LEN = number of bytes strictly after
   the CMD byte.

7. **ESP32 TX buffer:** The original firmware uses a 1024-byte `PrivCommTxBuffer` with
   the fixed header `FA 03 00 00` pre-loaded at `[0..3]`. The SEQ is always copied from
   the most recent received frame's SEQ into `TxBuffer[5]` before building a reply.
