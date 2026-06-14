# motor_controller — MiniBot 1 kHz firmware (L0)

The **only hard-real-time** part of the stack. Runs on the MCU (STM32/Teensy) on
the firmware team's PCB. It owns the 1 kHz loop and the safety watchdog; the
Jetson never closes a loop this fast (Linux is not hard-real-time — blueprint §9).

## Responsibilities

1. Receive a CRC-checked **impedance command** (`MotorCmd`) from the Jetson's
   `ros2_control` serial hardware interface.
2. Run the **PD / MIT-impedance law** at 1 kHz: `τ = kp·(q*−q) + kd·(qd*−qd) + τ_ff`,
   clamped to `tau_max` and the joint limits.
3. **Watchdog**: if the bus goes silent past the timeout, zero torque
   independently of the Jetson.
4. Stream measured **state** (`MotorState`) back — including `eff` from
   **current sense**, which the residual-RL observation vector requires (blueprint §9).

## Layout

```text
include/   protocol.h (host/MCU wire format) · pd_controller.h · watchdog.h
src/       pd_controller.c · watchdog.c (portable) · main.c (MCU loop skeleton)
test/      test_pd_controller.c (host unit tests)
```

## Build

```bash
# Host unit tests of the control math (what CI runs):
cmake -S . -B build -DBUILD_HOST_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure

# MCU image (needs an arm-none-eabi toolchain + the BSP):
cmake -S . -B build-mcu -DBUILD_MCU_FIRMWARE=ON \
      -DCMAKE_TOOLCHAIN_FILE=../../deploy/toolchains/arm-none-eabi.cmake
```

The wire format (`protocol.h`) mirrors the existing `soccer-firmware` repo
(CRC16-CCITT, MIT-style command), so the host and MCU agree byte-for-byte.
