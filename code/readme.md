# Code

In this folder you can find all of the code used on the robot.

##Folder contents:
- `pico2pi5-bridge` Contains the code for the UART bridge between the Pi Pico 2 Zero and the Pi 5 SBC
- `open-challenge-cv` Contains the code for the Open Challenge with OpenCV (Camera support)
- `open-challenge-nocv` Contains the code for the Open Challenge without OpenCV (analog, using raw ToF & IMU logic, fallback in case the camera module fails)
- `obstacle-challenge` Contains the Obstacle Challenge code

### Digital robot structure:
Pi 5 SBC:
- OS: <a href="https://downloads.raspberrypi.com/raspios_arm64/images/raspios_arm64-2026-06-19/2026-06-18-raspios-trixie-arm64.img.xz">Debian 13 Trixie arm64 64bit</a>
- TFT LCD: 2.4" with <a href="https://github.com/goodtft/LCD-show">LCD-Show2.4</a> via GPIO pins
- PC Connectivity: RealVNC & OpenSSH Server via LAN/WiFi with custom dual monitor for VNC/LCD separation 

Pi Pico 2 Zero MCU:
- Bootloader: <a href="https://github.com/earlephilhower/arduino-pico">Arduino for PICO</a>
- Connectivity to Pi 5 SBC: UART
- Connectivity to sensors: I2C via multiplexer

### Bridge Interface:
The bridge interface consists of a Python3 graphical and CLI script which is displayed on the Pi 5's LCD display and/or runs in the background. It translates OpenCV data from instructions like `steer: 100` to code that the Pi Pico 2 understands and the same thing in reverse. The full UART comms protocol can be found in the main README of the project. 


