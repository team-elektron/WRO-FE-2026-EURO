# Elektron Team - WRO Future Engineers 2026 - European Open

## Introduction

We are Team Elektron from the High School of Electrical Engineering in Sarajevo, Bosnia & Herzegovina.
This is the public repo for our team and the work we have done on our robot for the Open Championship in Zagreb.


### Robot Gallery

<p float="left">
  <img src="robot-photos/right.jpg" width="350"/>
  &nbsp;
  <img src="robot-photos/Pics/servo-and-tof.jpg" width="350"/>
</p>


### Team Photo

<p float="left">
  <img src="team-photos/normal.jpg" width="350"/>
  &nbsp;
</p>

## Folder Content
* `team-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `robot-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `videos` contains the video.md file with the link to a video where driving demonstration exists
* `schematics` contains schematic diagrams of the driving base.
* `code` contains code of control software for all components which were programmed to participate in the competition.
* `3d-models` is for the files for models used by 3D printers to produce the vehicle elements.

## Navigation

- <a href="#introduction">Introduction</a>
- <a href="#team-photo">Team Photo</a>
- <a href="#robot-gallery">Robot Gallery</a>
- <a href="#technical--mechanical-specifications">Technical & Mechanical Specifications</a>
  - <a href="#mechanics--drive">Mechanics & Drive</a>
  - <a href="#electronics">Electronics</a>
  - <a href="#size">Size</a>
- <a href="#how-does-it-work">How does it work?</a>
  - <a href="#driving-system">Driving System</a>
  - <a href="#steering-system">Steering System</a>
  - <a href="#power-and-power-delivery">Power & Power delivery system</a>
  - <a href="#the-brain">The brain (SBC, MCU)</a>
  - <a href="#sensors">Sensors</a>
- <a href="#bill-of-materials">Bill of Materials</a>
- <a href="#the-assembly-process-and-future-improvements">The assembly process and future improvements</a>
  - <a href="#pcb-assembly">PCB Assembly</a>
  - <a href="#3d-print--assembly">3d Print & Assembly</a>
- <a href="#code-explanation">Code Explanation</a>
  - <a href="#raspberry-pi-pico-2---pi-5-communications-bridge">Raspberry Pi Pico 2 <-> Pi 5 Communications bridge</a>
  - <a href="#open-challenge">Open Challenge</a>
  - <a href="#obstacle-challenge">Obstacle Challenge</a>



## Technical & Mechanical Specifications

### Mechanics & Drive
- Fully custom built and 3d printed robot base
- High torque, medium speed driving system
- Servo steering system

### Electronics
- 5Ah 2S battery pack (Li-Po)
- Time-of-Flight distance sensors
- Raspberry Pi 5 8GB SBC
- Raspberry Pi Pico 2 Zero MCU
- Full-axis IMU sensor
- Addressable LED indicators
- 2.4in TFT Touch Control LCD
- Raspberry Pi Camera Module 2 160 degree FOV

### Size
- 205x135x65mm (LxWxH) without camera mount, 205x135x~150mm (LxWxH) with camera mount
- ~490g total weight

<p float="left">
  <img src="robot-photos/Pics/length.png" width="150"/>
  &nbsp;
  <img src="robot-photos/Pics/width.jpg" width="300"/>
  &nbsp;
  <img src="robot-photos/Pics/weight.jpg" width="280"/>
</p>



## How does it work?

### Driving system
The main driving motors are 2 N30 motors in a 1:150 configuration, then the power is transferred to the wheels with a 1:1 gear ratio, the motors feature hall effect sensors. 

<p float="left">
  <img src="robot-photos/Pics/driving-base.jpg" width="400"/>
  &nbsp;
  <img src="robot-photos/Pics/driving-base-3dprint.jpg" width="400"/>
</p>

We have tested a couple different configurations (mainly 1:3 and 1:1) out of which, 3:1 turned out to work the best for our needs as these motors spin quite fast but are quite low torque. For extra torque, we decided to include 2 motors instead of one. They are connected to the DRV8833 H-Bridge IC on separate channels **BUT** the motors are connected onto one shared shaft together. The separte channels are exclusively to distribute load evenly on the IC.

We are using a 110mm long and 3mm in diameter steel shaft and plastic gears.

Wheels are 3D printed, each wheel being 30mm in diameter without the outer rubber bits.

<p float="left">
  <img src="robot-photos/Pics/wheel.png" width="350"/>
  &nbsp;
</p>

These specific motors have 2 hall effect sensors each, one for speed and one for direction, however we are only using one set from one motor, the other one stays disconnected and should only be used in case that something goes wrong with the other motor.

Speed control is done via a PID algorithm handled by the Pi Pico 2.

The motors are controlled by a DRV8833 IC which is current-limited to 1A per channel.

<p float="left">
  <img src="robot-photos/Pics/drv8833.jpg" width="250"/>
  &nbsp;
  <img src="robot-photos/Pics/driving-mech-assembled.jpg" width="450"/>
</p>

### Steering system
The steering system is controlled by one main MG90S servo from Waveshare.

Last year, we had issues with our steering system on the Open Championship in Ljubljana which was caused by a fake "metal" servo. Hence why we opted for higher quality full metal gear servos, and the MG90S was a great option as we didn't really need more power than that.

Steering power is transferred to the wheels with a steering shaft and poles on which we tied some rubber bands to help center the steering mechanism.
With this configuration, we are able to achieve a steering angle of 50 degrees which fits our needs nicely.

<p float="left">
  <img src="robot-photos/Pics/steering-base.jpg" width="400"/>
  &nbsp;
  <img src="robot-photos/Pics/servo-and-tof.jpg" width="400"/>
</p>

### Power and power delivery
We had originally thought about using a 3S battery system like we did last year, however, we switched to a 2S system now to save on space and weight. With some clever engineering tricks and overcomplication the power delivery side, that was super easy to do.

<p float="left">
  <img src="robot-photos/Pics/initial-assembly.jpeg" width="350"/>
  &nbsp;
</p>

The main power source are 2 5360105 Li-Po 3.7v 5Ah cells connected in series, they each have their own integrated BMS for safety.
We have decided to continue to charge our cells with a USB-C charger, but this year we have switched to a much more efficient IP2326 module for charging (and also because our previous battery had a dedicated BMS as it was built from bare 18650 cells and had balancing included which the 2 separate battery cells don't).

The actual power delivery part is split up into a couple different stages, one buck regulator for the Pi 5 exclusively and 2 smaller LDO regulators for the Pi Pico 2, sensors and another for the MG90S servo.

The main power regulator IC is a TPS54623RHLR. It's a pretty recent chip with not so much documentation available from other hobbyists but due to its very attractive price and great performance claims, we had to give it a shot. After designing a circuit that works for our needs in Ti's WEBENCH design utility (which are basically 5V output and at least 5A output max), we had ourselves a very nice and reliable regulator config. 

Robots are noisy machines, both literally in an audible sense, and power wise. Last year we had major issues with premade modules for power regulations which just couldn't handle our Pi, so this year, we decided to go overkill on ensuring our robot has clean power everywhere at all times. Our decoupling circuit includes:
  -  One 220uF 10V 10TPE220ML Panasonic Polymer Tantalum capacitor for bulk power storage.
  -  Two 10uF 6.3V X6S CL10X106MQ8NNNC Ceramic MLCC Samsung capacitors for mid frequency decoupling
  -  Four 100nF 6.3V X7R CGA0402X7R104K6R3GT Ceramic MLCC HRE capacitors for high frequency decoupling which were placed right next to the GPIO power pins of the Pi 5

<p float="left">
  <img src="robot-photos/Pics/5v-reg-1.png" width="400"/>
  &nbsp;
  <img src="robot-photos/Pics/vreg.jpg" width="400"/>
</p>

For the other two power regulators, we decided to go with the following combination:
  - AP2112K-3.3TRG1 3.3V 1A LDO regulator for the Pico 2 and 3.3V sensors (ToF and IMU)
  - ME6118A50PG 5V 1A LDO regulator for our MG90S servo

<p float="left">
  <img src="robot-photos/Pics/5v-3v3-reg-1.png" width="400"/>
  &nbsp;
</p>

### The Brain
Our robot runs on a Raspberry Pi 5 (8GB). We chose this SBC because we had it available and because the only other option we had was a Pi 4 with 2GB of ram. The choice is obvious :D
The Pi 5 communicates with the Pi Pico 2 Zero through UART on pins GP14/15 (and pins GP0/1 on the Pico 2). 

Workloads are distributed between both the SBC and the MCU like this:
<p float="left">
  <img src="robo-photos/Pics/comms-topology.png" width="350"/>
  &nbsp;
</p>

Pi 5 Handles camera input and OpenCV image processing and then sends movement data to the Pi Pico 2 Zero. 

We use a simple UART protocol to communicate between the SBC and MCU:
- Every packet
  
  `[0xAA] [TYPE] [LEN] [PAYLOAD...] [CRC8]`
  - `0xAA` Start byte, always
  - `TYPE` What kind of message
  - `LEN` Payload length in bytes
  - `CRC8` Checksum of everything except start byte

- Pi 5 -> Pico 2 Zero (commands): `TYPE = 0x01`
  
  `[0xAA] [0x01] [04] [dir] [speed] [servo] [led_mode] [CRC8]`
  - `dir` 0 = stop, 1 = fwd, 2 = bwd
  - `speed` 0 - 255
  - `servo` 0 - 180
  - `led_mode` 0 = idle, 1 = driving, 2 = obstacle, 3 = low battery

- Pico 2 Zero -> Pi 5 (telemetry): `TYPE = 0x02`
  
  `[0xAA] [0x02] [14] [tof0_H] [tof0_L] [tof1_H] [tof1_L] [tof2_H] [tof2_L] [tof3_H] [tof3_L] [batt_H] [batt_L] [yaw_H] [yaw_L] [enc_H] [enc_L] [CRC8]`
  - ToF values in mm, split into HI/LO bytes
  - Battery in mV
  - Yaw in tenths of a degree (so 1234 = 123.4 degrees)
  - Encoder ticks as int16

The robot is controlled through the 2.4in TFT Touchscren LCD display. It also shows all of the telemetry data collected from the Pi Pico 2 Zero and allows for manual control when debugging. The competition-ready interface will just feature a simple option to select between running the obstacle or open challenge code and one start button. 


### Sensors
After running into a lot of trouble with ultrasonic distance sensors (hc-sr04) last year, we now use Time-of-Flight distance sensors. The most readily available sensor of this kind on the market is the VL53Lx series of sensors.

We use 3 ToF sensors on the robot with the option of adding a 4th sensor (rear side sensor) if needed. The data is polled in single shot mode on each sensor for higher speed and precision.

<p float="left">
  <img src="robot-photos/Pics/vl53l0x.jpg" width="350"/>
  &nbsp;
</p>

For getting the turn angle we use the MPU6050 IMU sensor. It's not the most accurate sensor and has a little bit of drift when idle but that's nothing resetting the sensor yaw angle before every turn cant fix. 

<p float="left">
  <img src="robot-photos/Pics/mpu6050.png" width="350"/>
  &nbsp;
</p>

## Bill Of Materials
- <a href="https://www.waveshare.com/rp2350-zero.htm">Raspberry Pi Pico 2 Zero (WaveShare)</a>
- <a href="https://sync.ba/product/raspberry-pi-5/">Raspberry Pi 5 8GB</a>
- <a href="https://share.temu.com/9bHhRICRygB">Time-of-Flight VL53L0x (Temu)</a>
- <a href="https://share.temu.com/IlQc3Dd6nUB">IMU6050 (Temu)</a>
- <a href="https://www.waveshare.com/mg90s-servo.htm">MG90S Metal 9g Servo (WaveShare)</a>
- <a href="https://www.aliexpress.com/item/1005009751662765.html">N30 1:150 w Hall sensor motor (AliExpress)</a>
- <a href="https://share.temu.com/7EXXP9jFbPB">5360105 3.7V 5000MAh Li-Po Battery (Temu)</a>
- <a href="https://share.temu.com/1M8ztQmiIdB">IP2326 2S Charger module (Temu)</a>
- <a href="https://github.com/team-elektron/WRO-FE-2026/blob/main/Schematics/BOM_WRO-FE-2026.csv">PCB Components BOM (LCSC)</a>



## The assembly process and future improvements

### PCB assembly

<p float="left">
  <img src="robot-photos/Pics/pcb1.jpg" width="330"/>
  &nbsp;
  <img src="robot-photos/Pics/pcb2.png" width="300"/>
  &nbsp;
</p>

<a href="https://cloud.sphereconsoles.com/index.php/s/2eqJmXQ9cgKXqKD">Video of the assembly process</a>

We have designed a 4 layer PCB with a thickness of 1.6mm. The bottom layer carries the major power rails to their components along with providing a good ground plane. Middle layers carry all of the I2C, UART and other signal traces, and the top layer also acts like a ground plane for the most part. It has major power traces too and some but very few signal traces. The two copper GND planes are connected between the top and bottom layer by many vias throughout the whole PCB.

It is fully assembled by hand with our custom BGA station. All components were placed by hand. We used a solder paste with the Sn64Pb37 leaded formulation because of it's low melting point (~183c) and because it is easier to work with than lead-free solder paste. The trickiest part was soldering in the main 5V power regulator because of its VQFN-14 package, once it flowed into place, we had to rework it with a soldering iron to get rid of the aditional paste that oozed out to the sides + a nudge test with added AMTECH 559-V2-TF flux to ensure all of the solder joints flowed correctly. The PCB worked on the first try. 


### 3d Print & Assembly
Most of the robot parts are plastic and 3D printed. We used an Ender 3 V3 KE to print all of our parts. All of the 3D designs are available in the 3D Designs folder.
The robot is held together by M3 Nylon screws and standoffs. We used these instead of brass screws/standoffs because of the weight advantage they provide. 

<p float="left">
  <img src="robot-photos/Pics/chassis-batt.png" width="400"/>
  &nbsp;
</p>

The main chassis was printed with Copymaster3D PETG filament like all other components with the most notable settings being the use of 3 wall loops for extra strength. This is also the heaviest 3D printed part of our robot and the most important one structurally. Previous printing tests with 2 wall loops have shown that the model is prone to flexing under load which is not ideal so after adding another "rail" in our design and printing it with 3 wall loops, 4 bottom shell layers, 4 top shell layers and 50% honeycomb infill, we got a very nice and solid robot chassis.

<p float="left">
  <img src="robot-photos/Pics/drive-mech.png" width="465"/>
  &nbsp;
  <img src="robot-photos/Pics/drive-mech-2.png" width="400"/>
</p>

The main driving body holds in the 2 N30 motors. It's made from 2 separate pieces which screw down together, this same driving base has the camera tower mount screwed down onto it. It's printed with the same filament as the chassis and 3 wall loops and 30% honeycomb infill.
Right next to the main driving case is the gear holder as can be seen in the photos above. It makes sure all gears remain in one spot and don't move anywhere.
We have decided to keep the main wheel mounts integrated into the chassis but after having to take apart the robot case to dissasemble the driving mechanism multiple times, we have decided that it's best to make these detachable next time. 

<p float="left">
  <img src="robot-photos/Pics/steer-mech.png" width="400"/>
  &nbsp;
</p>

The steering mechanism is built from 3 different pieces:
  1. Bottom (Base) + Servo holder
  2. Steering Axle
  3. Top Cover + ToF Sensor Holder

They are connected together with Nylon screws (bottom and top part, the steering axle is connected to the servo). All 3 parts are printed with PETG filament with 3 wall loops, 30% honeycomb infill.
This design has had many changes until we settled on the most reliable design although it's center is not very precise. That's another thing we will improve next time. For now, we are compensating for the losses via software.




## Code Explanation

### Raspberry Pi Pico 2 <-> Pi 5 Communications bridge
Our robot is split into 2 control units, the Pi Pico 2 which handles the sensors and motors and communicates with the Pi 5. The Pi 5 handles the code for the open and obstacle challenge.
These 2 components communicate via UART.

<p float="left">
  <img src="robot-photos/Pics/bridge_comms_flowchart.svg" width="500"/>
  &nbsp;
</p>

### Open Challenge
For the open challenge we use a simple script that lives on the Pi Pico 2 Zero.

<p float="left">
  <img src="robot-photos/Pics/open-topology.png" width="350"/>
  &nbsp;
</p>

The code works by initiating a FOR loop which runs for a preset amount of times (turns). 
Main logic works by utilizing the side sensors as a line follower, and using the front distance sensor as a wall detector. 

Basically, go forward until front distance drops below 100cm, then initiate a 90 degree turn in the direction that the robot figures out by looking at which of the 2 side sensors is out of range (the side thats out of range is the side with no wall, so turn towards that one). It repeats this whole process approximately 12 times. 4 turns per lap, 3 laps total.
