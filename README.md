# Water Tank Sensor

Water level monitoring for the main water tank at Essência. An ultrasonic sensor on the tank measures the water level and sends it over LoRa to a receiver, which publishes it to the [DataHub](https://github.com/Essencia-life/DataHub) (Grafana dashboards and alerting, Neon archive).

```
 tank                                    house (WiFi)
┌────────────────────┐   LoRa 868 MHz   ┌────────────────────┐   HTTPS    ┌─────────┐
│ SENDER             │ ───────────────► │ RECEIVER           │ ─────────► │ DataHub │
│ ESP32 + ultrasonic │   every 30 s     │ ESP32, display     │  every 3 m │         │
└────────────────────┘                  └────────────────────┘            └─────────┘
```

Both devices are the same board running the same firmware (`src/main.cpp`). The PlatformIO environment you build decides whether it becomes the sender or the receiver.

## Hardware

| Part | Used for |
| --- | --- |
| 2× Heltec WiFi LoRa 32 V3 (ESP32-S3, SX1262, 0.96" OLED) | one sender, one receiver |
| JSN-SR04T-v3.3 waterproof ultrasonic sensor | sender only |
| 2× 868 MHz LoRa antenna | **required on both boards** |
| 2× USB-C cable (data, not charge-only) and a 5 V supply | flashing and power |

> **Attach the antenna before powering a board.** Transmitting without an antenna can damage the radio chip.

LoRa is configured for the EU 868 MHz band. Boards for other regions need a different frequency band in `setup()`.

### Wiring of the sender

| Sensor | Board pin |
| --- | --- |
| Trig | GPIO 4 |
| Echo | GPIO 5 |
| VCC / GND | supply and ground as specified for your sensor board |

The ESP32-S3 inputs are 3.3 V only, never connect a 5 V echo signal directly. The display is the on-board OLED (I²C: SDA 17, SCL 18, reset 21, powered via `Vext`), nothing to wire.

How the sensor sits in the tank and which distances the software uses:

<img width="434" height="220" alt="Watertank_Sketch_Mounting_and_Distances" src="https://github.com/user-attachments/assets/f64ce825-2ba4-4d1e-ad7b-b6d5713ad396" />

## How it works

### Sender

- Takes up to 10 ultrasonic measurements per cycle (1 s) and averages 5 valid ones, then smooths the result (80 % old value, 20 % new value).
- Converts the distance to percent: **0 %** at 130 cm distance (water pipe level), **100 %** at 30 cm distance (the sensor's dead zone is about 22 cm, so full is defined below it). The tank geometry is set by constants at the top of `src/main.cpp`.
- Sends status and level over LoRa every 30 s and shows distance, percent and status on its display.

Sensor status values (sent as a number):

| Value | Status | Meaning |
| --- | --- | --- |
| 0 | OK | valid measurement |
| 1 | Timeout | no echo: object too far away, or sensor covered |
| 2 | Dead zone | water very close to the sensor, still usable |
| 3 | Drift error | jump of more than 10 cm between cycles (lid opened or closed, sensor moved) |
| 4 | Init | just started, no valid measurement yet |
| 5 | Out of range | water appears deeper than the tank (e.g. lid open) |

### Receiver

- Listens for LoRa packets and shows status, level and the age of the last packet on its display. After 10 minutes without a packet it shows `??`.
- Connects to WiFi (the SSID is the constant `wifi_ssid` in `src/main.cpp`, currently an open network without password) and gets the time from `pool.ntp.org`.
- Every 3 minutes it posts the water level to the DataHub:

  ```
  POST https://<SERVER_HOST>/api/ingest/lora-tank
  x-api-key: <SERVER_API_KEY>

  {"deviceId":"essencia-water-tank-01","timestamp":"2026-10-23T11:00:00Z","waterLevelPercent":68.4}
  ```

- Nothing is sent while the sensor status is not OK or dead zone, so a sensor fault shows up as missing data and never as a false 0 %.
- If the DataHub or WiFi is unreachable, readings wait in a queue of 64 entries (about 3 hours at one reading per 3 minutes) and are sent oldest first. The queue lives in RAM and is lost on reboot or power loss.

## Build and flash

### Requirements

- [PlatformIO](https://platformio.org/) as the VS Code extension "PlatformIO IDE", or the command line tool (`pip install platformio`). Internet access is needed for the first build, it downloads the ESP32 toolchain and the libraries listed in `platformio.ini` (Heltec ESP32 LoRa v3, U8g2) automatically.
- A USB-C **data** cable. If the computer does not list a serial port for the board, install the USB-serial (CP210x) driver for your operating system.

### Environments

| Environment | Board becomes | Build flags |
| --- | --- | --- |
| `wassertank_sender` | sender (at the tank, with the ultrasonic sensor) | `IS_SENDER` |
| `wassertank_receiver` | receiver (WiFi, uploads to the DataHub) | `IS_RECEIVER`, `SERVER_HOST`, `SERVER_API_KEY` |

The role is set only by the environment. You do not have to change any `#define` in the code.

### Configure the receiver

In `platformio.ini`, section `[env:wassertank_receiver]`, set the DataHub host (without `https://` and without path) and its API key:

```ini
build_flags =
    -D IS_RECEIVER
    -D SERVER_HOST=\"datahub.example.com\"
    -D SERVER_API_KEY=\"your-api-key\"
```

**Do not commit the real API key.** Either keep it only in your local copy and discard the change afterwards (`git checkout platformio.ini`), or read it from an environment variable, which PlatformIO supports: `-D SERVER_API_KEY=\"${sysenv.WATERTANK_API_KEY}\"`.

The WiFi network is set in `src/main.cpp` (`wifi_ssid`). The current code connects to an open network only, there is no password setting.

### Flash with the command line

```sh
git clone https://github.com/Essencia-life/WaterTankSensor.git
cd WaterTankSensor

pio run -e wassertank_sender                  # build only
pio run -e wassertank_sender -t upload        # build and flash the sender
pio run -e wassertank_receiver -t upload      # build and flash the receiver
pio device monitor -b 115200                  # serial log
```

Connect only the board you want to flash. With several boards plugged in, list the ports with `pio device list` and pick one: `pio run -e wassertank_receiver -t upload --upload-port <port>` (for example `COM5` on Windows or `/dev/ttyUSB0` on Linux).

### Flash with VS Code

1. Open the project folder in VS Code with the PlatformIO extension installed.
2. Choose the environment in the blue status bar at the bottom ("Switch PlatformIO Project Environment"): `wassertank_sender` or `wassertank_receiver`.
3. Click the arrow (Upload) in the status bar. The plug icon opens the serial monitor.

<img width="866" height="398" alt="PlatformIO status bar in VS Code" src="https://github.com/user-attachments/assets/72fe7bc0-0625-456b-9df8-0bd6ac3ff305" />

The screenshot shows the sender environment selected. Switch to the receiver environment before flashing the receiver.

### If the upload fails

If the board is not found or the upload cannot connect, put it into download mode by hand: hold the `PRG` button, press and release `RST`, then release `PRG`, and start the upload again.

### First start

The serial monitor (115200 baud) shows the log. After flashing:

- **Sender display:** distance in cm, level in percent, sensor status and `LORA: OK`.
- **Receiver display:** `Watertank Level (R)`, the sensor status from the last packet, level and `WiFi: OK` once WiFi is connected and the time is synchronized. The first packet arrives within 30 s.
- Start the sender first or the receiver first, the order does not matter.

Frequency, spreading factor, coding rate and sync word must be identical on both boards. They are, as long as both are built from the same source. When you change them, flash both boards.

## Settings in `src/main.cpp`

| Setting | Default | Meaning |
| --- | --- | --- |
| `trigPin`, `echoPin` | 4, 5 | ultrasonic sensor pins |
| `distance_max_depth_watertank` | 150 cm | distance from sensor to tank bottom |
| `height_watertank_0percent` | 20 cm | water height that counts as 0 % |
| `height_watertank_100percent` | 120 cm | water height that counts as 100 % |
| `lora_send_interval` | 30 s | how often the sender transmits |
| `server_push_interval` | 3 min | how often the receiver posts to the DataHub |
| `upload_queue_capacity` | 64 | readings kept while offline |
| `wifi_ssid` | `STARLINK` | WiFi network of the receiver |

## Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| No serial port, upload cannot connect | charge-only cable, missing driver, or board needs download mode (see above) |
| Receiver shows `??` | no LoRa packets for 10 minutes: sender off, antennas missing or too far apart, sender shows `LORA: Err` |
| Receiver shows `WiFi: Err` | network not reachable, or time not synchronized yet (NTP needs internet) |
| Serial log: `Server push failed: HTTP 401` | wrong `SERVER_API_KEY` |
| Serial log: `Server push failed: HTTP 400` | the DataHub rejected the payload, check the DataHub logs |
| Serial log: `Upload queue full; dropping oldest upload` | DataHub or WiFi was unreachable for more than about 3 hours |
| Status `Timeout` or `Out of Range` | sensor covered, lid open, or mounting does not match the tank geometry constants |
| Status `Drift` | water surface or sensor moved by more than 10 cm between two cycles |

## Repository

- `src/main.cpp` – firmware for sender and receiver (`#ifdef IS_SENDER` / `#ifdef IS_RECEIVER`)
- `platformio.ini` – the two build environments
- `src/main2.txt` – old display test, not part of the build
