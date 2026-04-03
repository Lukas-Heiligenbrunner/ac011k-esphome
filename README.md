# AC011K ESPHome Firmware

ESPHome custom component and configuration for the **Ecactus AC011K** EV wallbox charger. Replaces the proprietary cloud firmware with local Home Assistant integration.

---

## What it does

- Full charging control (start, stop, current limit 6–16 A) via Home Assistant
- Real-time sensor readout: power, per-phase voltage & current, session and total energy
- EVSE status tracking (available, preparing, charging, suspended, fault, …)
- **Solar auto-charging mode**: automatically starts and stops charging based on excess solar power, using a configurable current to match available surplus from a Shelly 3EM grid meter
- Home battery awareness: treats home battery charge power as available for the car, but never draws from the battery to charge the car
- Auto-start on cable plug-in (optional)
- OTA firmware updates, local web UI, encrypted Home Assistant API

---

## How it works

The AC011K contains two processors:

- **ESP32** — the application/networking processor. The stock firmware runs the Ecactus cloud stack here. This project replaces that entirely with ESPHome.
- **GD32** — the dedicated charging controller. It manages the CP pilot signal, contactor, metering, and all safety interlocks. It is **not** replaced.

The ESP32 and GD32 communicate over an internal UART (GPIO 32 TX / GPIO 34 RX, 115200 8N1) using a proprietary binary protocol called **PrivComm**. For full protocol details see [PRIVCOMM_PROTOCOL.md](PRIVCOMM_PROTOCOL.md).

### Solar auto-charging

A 10-second ESPHome interval reads the grid power from a Shelly 3EM and computes:

```
available_W = charger_power - grid_power - 500 W_buffer + battery_power
target_A    = available_W / (230 V × phases)
```

- `grid_power` positive = importing; negative = exporting surplus solar
- `battery_power` positive = battery charging (those watts are available for the car); negative = battery discharging (subtracted to prevent using battery power for the car)
- 500 W safety buffer keeps a margin before loading the grid
- Hysteresis: 60 s before starting, 240 s before stopping (avoids rapid cycling on cloud cover)

---

## Supported hardware

Tested on an **Ecactus AC011K-AE-35** (3-phase). The EN+ and Autoaid branded versions of the AC011K appear to be the same hardware with a different label and are likely compatible, but have not been tested.

Single-phase variants likely share the same PrivComm protocol but are also untested. Note that the GD32 phase-switching command is non-functional on this hardware — the charger always operates in 3-phase mode regardless (see known issues).

---

## Ethernet

The IP101 PHY hardware is present on the board and the GPIO pins are identified (MDC=GPIO23, MDIO=GPIO18, power=GPIO5). The Ethernet interface is implemented in `ac011k.yaml` but commented out in favour of WiFi. To switch, uncomment the `ethernet:` block and comment out the `wifi:` section.

---

## Credits

The PrivComm protocol was originally reverse-engineered by the **warp-more-hardware** project:

> [https://github.com/warp-more-hardware/esp32-firmware](https://github.com/warp-more-hardware/esp32-firmware)

That work provided the frame format, command codes, and payload layouts that made this ESPHome port possible. The GD32 PV-mode bug and max-current-limit cap were discovered during development of this project.

---

## Known issues / TODO

### Not working

- **Phase switching** — The GD32 accepts the `0xAA`/`0x52` SetPhase command (returns an ACK) but silently ignores it. The charger always operates in 3-phase mode. Root cause unknown; possibly a GD32 firmware limitation. Single-phase charging is therefore not available.

### Partially working / caveats

- **Total energy counter** — The `energy_total` sensor value is stored on the GD32 and resets whenever the GD32 reboots (which happens on every ESP32 reboot due to the init sequence). Home Assistant's long-term statistics compensate for this, but the raw sensor value is not persistent.

### TODO

- **1-phase charging mode** — Blocked by the GD32 ignoring the phase-switch command. If a working method is found (e.g. a different sub-command or a GD32 firmware update), the YAML `installed_phases` select is already wired up to feed the auto-charge calculation.
- **RFID card management** — Cards are currently accepted unconditionally. A whitelist/blacklist mechanism could be implemented by storing authorised card UIDs in flash and checking them in the `0x05` handler.
- **Session energy persistence** — Session energy resets on cable unplug. A running total accumulated in ESPHome globals and pushed to HA could provide a more reliable lifetime counter independent of GD32 reboots.
- **CP pilot current readout** — The `0x0E` (ChargingParamRpt) frame contains the CP duty cycle (and thus the vehicle's advertised max current). This is currently only logged, not exposed as a sensor.