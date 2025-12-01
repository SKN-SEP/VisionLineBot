# LineBot
Line follower is an autonomous robot, that tracks the black line. Robot works thanks to the line sensor (qtr8 from pololu). The sensors detect diffrence between colors. This projects combines automatics, micrcontroller programming and 3d printing.

### Table of contents 
- [Introduction](#introduction)
- [PID Algorithm](#pid-algorithm)
- [Installation](#installation)
- [Assembly](#assembly)
- [License](#license)

## Introduction
...

## PID Algorithm 
...

## Installation 
1. Download <a href="https://www.arduino.cc/en/software/">Arduino IDE</a> and install QTRSensors library.
2. Clone github repository to your computer:
  ```
  https://github.com/SKN-SEP/LineBot.git
  ```
3. Connect your arduino to computer and upload the program.

## Assembly 
In order to build this robot, you need all the parts listed in the table below. The corresponding STL models are in the models folder. The skills required to assemble this robot include 3D printing, soldering, and cable management.
| Name                 | Description                                                                 | Image                                   |
|----------------------|-----------------------------------------------------------------------------|-----------------------------------------|
| Arduino Uno          | Microcontroller board for controlling sensors and motors                    | ![Arduino Uno](./assets/arduino-uno.jpg) |
| Cables 15 cm         | Short cables for connecting sensors, motors, power, and controller          | ![Cables](./assets/cables-15cm.jpg)     |
| QTR‑8 Pololu         | IR reflectance sensor array (for line tracking / floor detection)           | ![QTR‑8 Pololu](./assets/qtr8.png)      |
| DC Motor 1           | One of the two DC motors — drives wheel / movement                         | ![DC Motor](./assets/dc-motor1.jpg)     |
| DC Motor 2           | Second DC motor — drives wheel / movement                                  | ![DC Motor](./assets/dc-motor2.jpg)     |
| Chassis (AliExpress) | The base frame / body of the robot where motors, electronics, and wheels mount | ![Chassis](./assets/chassis.jpg)       |
| Li‑ion Batteries ×3  | Three Li‑ion batteries — main power supply for motors / controller          | ![Li-ion Batteries](./assets/lion-batteries.jpg) |
| Battery Slot (for 3) | Battery holder/slot for the 3 Li‑ion batteries                             | ![Battery Slot](./assets/battery-slot.jpg) |
| DC Switch            | Power switch controlling overall power on/off for the robot                | ![DC Switch](./assets/dc-switch.jpg)     |
| L298N Motor Driver   | Dual H‑bridge driver to control the two DC motors (direction & speed)       | ![L298N Driver](./assets/l298n.jpg)      |
| Ultrasonic Sensor    | Detection sensor, used to check if there are obstacles on the way       | ![L298N Driver](./assets/ultrasonic.png)      |
| Filament (for 3D printing) | Thermoplastic filament used if you 3D‑print custom parts (e.g. mounts, holders, covers) | ![3D Print Filament](./assets/filament.jpg) |


## License
This project is licensed under the MIT License. See the LICENSE file for details.
