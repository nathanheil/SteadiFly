SteadiFly – Fixed-Wing Flight Stabilizer

SteadiFly is a real-time flight stabilization system developed at Cal Poly SLO as part of EE 329 (Microcontroller-Based Systems Design). The project demonstrates how embedded systems can enable autonomous stability in fixed-wing aircraft by combining low-level hardware control with advanced filtering and feedback algorithms.

At its core, SteadiFly runs on an STM32 Nucleo-L4A6ZG microcontroller, programmed in bare-metal C without the STM32 HAL to ensure precise timing, low overhead, and full hardware control. The system continuously monitors aircraft orientation through an MPU6050 6-DOF IMU, fusing gyroscope and accelerometer data with a custom Kalman filter. These filtered estimates of pitch, roll, and yaw feed into independent PID controllers, which compute corrective outputs to maintain stable flight. Control signals are mapped to servo actuation via custom PWM drivers, enabling direct manipulation of ailerons, rudder, and elevator.

A UART-based interface was implemented for two key purposes:

Real-time PID tuning using interrupt-driven digit parsing, allowing gain adjustments mid-flight without recompiling code.

Telemetry and logging through PuTTY and MATLAB, enabling detailed analysis of sensor data and control responses.

The software is organized into well-defined modules (I2C.c, PID.c, PWM.c, UART.c, delay.c), each handling a specific subsystem. This modular design supported rapid debugging and iterative testing, especially during challenges like calibrating gyroscope bias, configuring I²C timing at 100 kHz from a 4 MHz clock, and mapping PID outputs into servo-friendly ranges.

Results & Lessons Learned

Achieved stable pitch, roll, and yaw correction in a test rig and flight scenarios.

Kalman filter tuning significantly reduced noise, improving orientation accuracy.

Real-time UART tuning saved hours of development time by eliminating repeated reflashing.

Key challenges included IMU calibration, low-level I²C configuration, and servo output scaling.

Future extensions include GPS-based waypoint navigation and more advanced autonomous modes.

Documentation

Full technical details, schematics, and analysis are available in the project report:
SteadiFly Report (PDF)
