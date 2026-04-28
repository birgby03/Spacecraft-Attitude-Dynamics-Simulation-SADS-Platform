# Spacecraft-Attitude-Dynamics-Simulation-SADS-Platform
Platform for attitude control spacecraft attitude-control research and validation. It uses a Raspberry Pi 5 onboard computer to interface with an IMU, reaction wheels, and a ground station. Uses PID logic to perform simple attitude control.

# SADS FSW Overview:

How to generate an executable file after making changes to the software:

First, ensure that there is one and only one file in the SADS_FSW folder named control_task.cpp.
This GitHub repository includes all three contol task iterations in the flight software folder so please be sure to remove the unwanted versions before building any new code.

Run:
```bash
mkdir -p build
cd build
cmake ..
make -j4
./sads_fsw
```

Open:
- http://localhost:8080
- or http://<pi-ip>:8080


Threads:
- IMU: 100 Hz (MTi-03 over USB serial)
- Estimator: 100 Hz (identity copy)
- Control: 20 Hz (zero commanded torque)
- Telemetry: 5 Hz (HTTP/AJAX)
