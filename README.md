# Heltec V3 LoRa-to-MQTT base station

This Arduino sketch turns a Heltec Wireless Stick Lite V3 into a receive-only
bridge for a small mailbox sensor. It receives fixed five-byte LoRa packets,
validates their node and message identifiers, and publishes the decoded state
as retained JSON over Wi-Fi to an MQTT broker.

## Hardware and dependencies

- Heltec Wireless Stick Lite V3 (ESP32 with SX1262 radio)
- Arduino IDE or another ESP32-compatible Arduino build environment
- Heltec's library/board support providing `LoRaWan_APP.h`
- `PubSubClient`
- `ArduinoJson`

The sender must use the same radio settings as the sketch: 868 MHz, 125 kHz
bandwidth, spreading factor 8, coding rate 4/8, an eight-symbol preamble, and
CRC. The 868 MHz configuration is intended for Italy; confirm that frequency,
power, and duty-cycle settings comply with the rules where the device is used.

## Packet format

The receiver accepts exactly five bytes:

| Byte | Field | Accepted or interpreted value |
| ---: | --- | --- |
| 0 | Node ID | Must be `0x01` |
| 1 | Message type | Must be `0x01` |
| 2 | Hatch state | `1` is open; other values are published as closed |
| 3 | Door state | `1` is open; other values are published as closed |
| 4 | Battery | Percentage supplied by the sender |

Packets with another length, node ID, or message type are rejected and counted
in the serial diagnostics.

## Configuration and upload

1. Install the board support and libraries listed above.
2. Open `heltec_V3-base-station.ino` and replace the placeholder Wi-Fi and MQTT
   values in the configuration section.
3. Select the Heltec Wireless Stick Lite V3 board and the correct serial port.
4. Compile and upload the sketch.
5. Open the serial monitor at 115200 baud to inspect connection and packet
   diagnostics.

The configurable values are:

| Constant | Purpose | Default |
| --- | --- | --- |
| `WIFI_SSID` | Wi-Fi network name | Placeholder |
| `WIFI_PASSWORD` | Wi-Fi password | Placeholder |
| `MQTT_BROKER` | MQTT hostname or address | `192.168.1.100` |
| `MQTT_PORT` | MQTT TCP port | `1883` |
| `MQTT_USER` | MQTT username | Placeholder |
| `MQTT_PASSWORD` | MQTT password | Placeholder |
| `MQTT_TOPIC` | Retained publication topic | `home/mailbox/status` |

Do not commit real credentials. This sketch stores them in the firmware and
connects to MQTT without TLS, so use it only on a trusted network or adapt the
network client and configuration handling for your threat model.

## MQTT payload

A valid packet produces retained JSON similar to:

```json
{
  "node_id": 1,
  "hatch_open": false,
  "door_open": false,
  "battery_percent": 87,
  "rssi": -72,
  "timestamp": 123456
}
```

`timestamp` is the ESP32's `millis()` value (milliseconds since boot), not a
wall-clock timestamp. The retained flag lets a new MQTT subscriber immediately
receive the most recently published state.

## Runtime behavior

- The radio remains in continuous receive mode and is restarted after every
  completed or failed receive event.
- Wi-Fi connectivity is checked every 30 seconds.
- MQTT connectivity is checked every 15 seconds, while `mqttClient.loop()` runs
  continuously.
- Packet counters are printed every 60 seconds when packet diagnostics are
  enabled.

## License

Licensed under the Apache License, Version 2.0. See [LICENSE.txt](LICENSE.txt).
