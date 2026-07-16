# mecanum_base (Pico firmware)

RP2040/Pico C++ micro-ROS firmware that drives a 4-wheel mecanum base — motor control, quadrature-encoder odometry, and an MPU9250 IMU — bridged to ROS 2 over USB.

## Overview

This is the low-level motion firmware for the VLM-Police-Patrol robot. It runs on a Raspberry Pi Pico (RP2040), implements mecanum inverse/forward kinematics, open-loop feedforward motor control over two TB6612 drivers, 4x quadrature-encoder decoding, and readout of an MPU9250 IMU on I2C1. It exposes itself as a ROS 2 node (`mecanum_base`) via micro-ROS over USB CDC serial, talking to a `micro_ros_agent` running on the RDK X5 host. A connection state machine handles agent (re)connect, resetting odometry and re-probing the IMU on each connect, and braking the motors on disconnect.

## micro-ROS interface

- **Node name:** `mecanum_base`
- **Transport:** USB CDC serial (custom `pico_serial_transport_*` from libmicroros). The host runs `micro_ros_agent serial --dev /dev/ttyACM0`. Baud is not set explicitly in firmware (USB CDC). LED on GP25: fast 100 ms blink = waiting for agent, solid = connected.
- **Subscribed:**
  - `cmd_vel` — `geometry_msgs/Twist`. Uses `linear.x` (forward), `linear.y` (left), `angular.z` (CCW); fed straight into the inverse kinematics and motor drive.
- **Published:**
  - `odom` — `nav_msgs/Odometry` @ 20 Hz. frame `odom` → child `base_link`; integrated pose (x, y, yaw as quaternion z/w), body-frame twist (vx, vy, wz), diagonal covariance (pose 1e-3/1e-3/1e-2, twist 1e-3/1e-3/1e-2). Timestamps epoch-synced via `rmw_uros_sync_session`.
  - `wheel_speeds` — `std_msgs/Float32MultiArray` @ 20 Hz. 4 floats `[FL, RL, RR, FR]` in rad/s.
  - `imu/data_raw` — `sensor_msgs/Imu` @ 50 Hz (only created if the IMU is detected). frame `imu_link`; `linear_acceleration` [m/s^2] and `angular_velocity` [rad/s]. `orientation_covariance[0] = -1` (no orientation, per REP 145); gyro/accel diagonal covariances 4e-4 and 4e-2.

## Hardware

4 motors driven by two TB6612 chips (STBY on GP14, HIGH enables both). Pin map as wired (array order FL=0, RL=1, RR=2, FR=3 = TB6612 channels A/B/C/D):

| Wheel | PWM | IN1 | IN2 | ENC_A | ENC_B |
|-------|-----|-----|-----|-------|-------|
| FL (A) | GP2 | GP3 | GP4 | GP15 | GP16 |
| RL (B) | GP5 | GP7 | GP6 | GP17 | GP18 |
| RR (C) | GP8 | GP10 | GP9 | GP20 | GP19 |
| FR (D) | GP11 | GP12 | GP13 | GP22 | GP21 |

Notes: RL and RR have IN1/IN2 swapped vs silkscreen (deliberate). PWM runs at wrap 999 (~125 kHz). Encoders are full 4x quadrature decoded in a shared GPIO edge ISR using a state-transition table; encoder inputs use internal pull-ups.

**MPU9250 IMU:** I2C1, SDA=GP26, SCL=GP27, 400 kHz fast mode, address `0x68` (AD0 low). Configured for ±2 g accel and ±250 dps gyro, DLPF ~41 Hz, ~200 Hz internal rate. Per-transfer I2C timeout (2 ms) guards against a stuck bus. Detection is by WHO_AM_I (accepts 0x71/0x73/0x70); if absent at boot it is re-probed on each agent connect, and IMU publishing is simply disabled if never found.

### Chassis geometry constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `WHEEL_R` | 0.04 m | Wheel radius |
| `LX` | 0.0625 m | Half wheelbase, center ↔ front/rear axle (full 125 mm) |
| `LY` | 0.100 m | Half track width, center ↔ left/right wheel (full 200 mm) |
| `ENC_CPR` | 1560.0 | Counts/rev = 13 pulses/rev × 4 (quadrature) × 30:1 gear |

## Kinematics

**Inverse (cmd_vel → wheels):** with `k = LX + LY`,
`FL = (vx - vy - k·wz)/WHEEL_R`, `RL = (vx + vy - k·wz)/WHEEL_R`, `RR = (vx - vy + k·wz)/WHEEL_R`, `FR = (vx + vy + k·wz)/WHEEL_R`. Each wheel speed maps to PWM duty via feedforward (`|rad_s|/MAX_W`, clamped) and direction pins; zero commands active-brake.

**Forward (encoders → odom):** encoder deltas → per-wheel rad/s, then
`vx = R/4·(FL+RL+RR+FR)`, `vy = R/4·(-FL+RL-RR+FR)`, `wz = R/(4·(LX+LY))·(-FL-RL+RR+FR)`. Body velocity is rotated by current heading and integrated into world-frame pose.

## Safety limits

| Constant | Value | Effect |
|----------|-------|--------|
| `MAX_W` | 20.0 rad/s | Max wheel angular speed; sets 100% PWM duty. Wheel commands above this saturate (duty clamped to 1.0) — caps effective output. |
| `WD_SECS` | 0.5 s | cmd_vel watchdog. If no `cmd_vel` arrives within this window, the odom timer calls `all_brake()` (all motors active-braked). |

On agent disconnect the firmware also calls `all_brake()` in `destroy_entities()`. **There is NO hardware e-stop yet (planned)** — braking is entirely firmware/software driven.

## Build & flash

Built with the Raspberry Pi Pico SDK (v2.2.0, `PICO_BOARD=pico`) plus the pre-built micro-ROS static library. `CMakeLists.txt` includes `pico_sdk_import.cmake` and `libmicroros/libmicroros.cmake`, links `pico_stdlib`, `hardware_pwm/gpio/irq/i2c`, and `microros`, and enables USB stdio (UART disabled). Output is `mecanum_base.uf2`.

One-time setup of the micro-ROS library (`libmicroros/` = a clone of [`micro_ros_raspberrypi_pico_sdk`](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk), vendored, ~177 MB — do not edit; matched to your ROS 2 distro, e.g. `-DMICRO_ROS_DISTRO=humble`):

```bash
git clone --recursive https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git libmicroros
cd libmicroros && mkdir build && cd build
cmake .. -DMICRO_ROS_DISTRO=humble && make
```

Then build the firmware:

```bash
mkdir build && cd build
cmake .. && make
```

Flash: hold **BOOTSEL** while plugging in the Pico (it mounts as a mass-storage device), then copy `build/mecanum_base.uf2` onto it. `build/` is gitignored.

## Runtime / troubleshooting

- On the RDK X5 host, start the agent: `micro_ros_agent serial --dev /dev/ttyACM0`. The LED goes solid once connected.
- The Pico occasionally needs a physical USB **unplug/replug** before it enumerates and the agent connects. See known issues in [`../../docs/technical.md`](../../docs/technical.md).
- If `imu/data_raw` is missing, the IMU was not detected at connect time — check I2C1 wiring/power; it is re-probed on each reconnect (no reset needed).

## Files

- `mecanum_base.cpp` — the complete firmware (kinematics, motor/encoder control, IMU, micro-ROS node + connection state machine).
- `CMakeLists.txt` — Pico SDK + libmicroros build producing `mecanum_base.uf2`.
- `pico_sdk_import.cmake` — standard Pico SDK locator.
- `.gitignore` — excludes `build/`.
- `libmicroros/` and `build/` are excluded from git and not part of the source (`libmicroros/` is the vendored micro-ROS dependency; `build/` is generated).

---

See the root [`../../../README.md`](../../../README.md) and [`../../../docs/technical.md`](../../../docs/technical.md) for the full robot and system documentation.
