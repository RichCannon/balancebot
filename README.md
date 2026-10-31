# Two-Wheel Self-Balancing Robot

A two-wheeled self-balancing robot built on the **ESP32-S3**. An IMU measures
the tilt, sensor fusion estimates the angle, and a PID controller drives two DC
motors through an H-bridge to keep the robot upright — the classic inverted
pendulum on wheels.

The firmware is written in C++ on top of **ESP-IDF** and structured into small,
isolated components so each part (sensor, motors, control, UI) can be tested and
replaced independently.

## Features

- 6-axis IMU over SPI with **complementary** and **1D Kalman** sensor fusion
  (selectable, both computed for comparison).
- PID balance controller running at 200 Hz.
- Dual DC motor control via a TB6612FNG H-bridge (LEDC PWM + direction GPIO).
- Optional WiFi web interface for start/stop, live PID tuning, and automatic
  setpoint calibration.
- Fall protection, motor dead-zone compensation, and integral anti-windup.

---

## Hardware

| Component | Role |
|---|---|
| **ESP32-S3** (DevKitC-1) | Main controller |
| **MPU-6500** | 6-axis IMU (accelerometer + gyroscope) |
| **TB6612FNG** | Dual H-bridge motor driver |
| **Mini360 DC-DC (HM) buck converter** | Steps the battery voltage down to a stable logic/supply rail |
| **2S 18650 Li-ion pack** | Main power (motor supply) |
| DC gear motors × 2 | Drive wheels |

### Pin mapping

| Function | ESP32-S3 pin |
|---|---|
| IMU SPI MOSI | GPIO 11 |
| IMU SPI MISO | GPIO 13 |
| IMU SPI CLK | GPIO 12 |
| IMU SPI CS | GPIO 10 |
| Motor A PWM | GPIO 4 |
| Motor A IN1 / IN2 | GPIO 6 / 5 |
| Motor B IN1 / IN2 | GPIO 15 / 16 |
| Motor B PWM | GPIO 17 |
| Driver STBY | GPIO 7 |

> Power: the 2S pack feeds the TB6612FNG `VM` (motor rail) and the Mini360 buck
> converter, which provides the logic supply. All grounds are tied together.

<img width="1130" height="431" alt="image" src="https://github.com/user-attachments/assets/c17f92c4-346c-40e3-a9c4-f2b27f5e0460" />

---

## Software

### Code structure

```
components/
├── config/     # All constants (pins, sensor ranges, filters, PID defaults)
│               # + shared runtime control state (control_state_t)
├── mpu/        # MPU-6500 driver: SPI, configuration, calibration, sensor fusion
├── motor/      # TB6612FNG driver: LEDC PWM + GPIO direction
├── balance/    # Hardware init + the balance control loop (PID)
└── web/        # Optional WiFi + HTTP control UI (compiled out when disabled)
src/
└── main.cpp    # Entry point: init hardware, start tasks
```

Everything tunable lives in `components/config/config.h`; the runtime-tunable
values (PID gains, setpoint, etc.) live in `control_state_t`.

### Main functions

**Entry point** (`main.cpp`)
- `app_main()` — boot delay → `balance_init()` → optional `web_init()` →
  `balance_start()`.

**Balance** (`components/balance`)
- `balance_init()` — starts the SPI bus, initializes the IMU and the motors.
- `balance_start()` — creates the balance control task.
- `balance_task()` — the 200 Hz loop: read IMU → fuse angle → PID → drive motors,
  with dead-band, fall protection, and setpoint calibration.
- `fusion_test_start()` — diagnostic task that streams gyro / accel / both fused
  angles in serial-plotter format (used on the filter-testing branch).

**IMU** (`components/mpu`)
- `mpu_init()` — adds the SPI device and configures the sensor (ranges, DLPF,
  sample rate).
- `mpu_calib()` — averages the gyro at rest to remove bias, sets the initial
  angle.
- `mpu_read_sensor()` — one SPI burst of accel/gyro data.
- `mpu_update()` — denominator-free sensor fusion: computes the accelerometer
  angle, runs the complementary and Kalman filters, and exposes the fused
  `pitch` / `roll` and their rates.

**Motors** (`components/motor`)
- `motor_init()` — configures direction GPIOs and the LEDC PWM.
- `motor_drive()` — set speed (−MAX…+MAX) with direction.
- `motor_brake()` / `motor_coast()` — short-brake or freewheel.

**Web** (`components/web`)
- `web_init()` — connects to WiFi and starts the HTTP control server.

### How balancing works

1. The IMU provides the tilt rate (gyro) and the gravity direction
   (accelerometer).
2. Sensor fusion blends them into a clean, drift-free angle.
3. The PID controller turns the angle error into a motor command:
   - **P** — proportional push (the "spring"),
   - **D** — derivative damping using the gyro rate (the "shock absorber"),
   - **I** — optional, with anti-windup.
4. Both wheels drive together to move the base back under the center of mass.

---

## Build & Run

- **IDE:** Visual Studio Code + **PlatformIO** (PlatformIO IDE extension)
- **Framework:** ESP-IDF
- **Toolchain / target:** ESP32-S3 (DevKitC-1)

```bash
pio run                 # build
pio run -t upload       # flash
pio device monitor      # serial monitor (115200 baud)
```

The web interface can be disabled (to save ~630 KB of flash) by setting
`Config::Web::ENABLED = false` in `components/config/config.h`. WiFi credentials
are set in the same file.

---

## Demo videos

<!-- TODO: add video links -->


https://github.com/user-attachments/assets/99ba4e16-8c25-4141-89a6-ad4949a87350

https://github.com/user-attachments/assets/a83020f9-2e72-45c0-bb2e-db484af614bb



## Images

<img width="500" height="auto" alt="IMG_1612" src="https://github.com/user-attachments/assets/5efe7726-fa12-46e6-ac31-811bdf4b2d00" />

<img width="500" height="auto" alt="IMG_1613" src="https://github.com/user-attachments/assets/d1d19244-5b63-442e-bd14-983fb38cc683" />

<img width="500" height="auto" alt="IMG_1611" src="https://github.com/user-attachments/assets/13245a23-57b0-4ba9-b266-9d8af6cea8e9" />

<img width="500" height="auto" alt="image" src="https://github.com/user-attachments/assets/1112c1b5-1a88-4f69-b74d-2f4ada80f79a" />
