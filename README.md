# Cat-and-Mouse Robot Project

## Table of contents
1. [Introduction](#1-introduction)
2. [FOC omniwheel](#2-foc-omniwheel)
   1. [Hardware](#21-hardware)
      1. [Electronics](#211-electronics)
      2. [Mechanics](#212-mechanics)
   2. [Software (Overview)](#22-software-overview)
   3. [Bottleneck](#23-bottleneck)
3. [DC omniwheel](#3-dc-omniwheel)
   1. [New Motor control PCB](#31-new-motor-control-pcb)
   2. [Adapting the frame to allocate new DC motors, and more...](#32-adapting-the-frame-to-allocate-new-dc-motors-and-more)
   3. [Movement strategy](#33-movement-strategy)
      1. [Cat mode](#331-cat-mode)
      2. [Mouse mode](#332-mouse-mode)
   4. [Issues](#34-issues)
4. [Regular DC](#4-regular-dc)
   1. [Hardware](#41-hardware)
5. [Stratey Implementation](#5-strategy-implementation)
   1. [Overview of Cat/Mouse Mode](#51-overview-of-cat/mouse-mode)
   2. [LiDAR handling](#52-lidar-handling)
   3. [Motor control](#53-motor-control)
   4. [Additional PIMP mode](#54-additional-pimp-mode)

## 1. Introduction
This project aims to design and build a **mobile robotic system** capable of playing a "cat and mouse" pursuit game.  
It combines **real-time embedded control**, **sensor fusion**, and **mechanical design** to create an autonomous robot capable of chasing or evading another robot in a dynamic environment. 
In addition to that we built ESP32-S3 based audio system with the intent of **listening with a mic** and answering using **AI based model through wifi connection and online server.**

## 2. FOC omniwheel

We first wanted to make a three omniwheeled robot with FOC control on brushless motors.

### 2.1. Hardware

The hardware design is divided into two main parts:

#### 2.1.1. Electronics
The electronic subsystem provides sensing, control, and communication between the robot's components. 

##### Motherboard

###### V1
<img width="497" height="475" alt="MB_v1_front" src="https://github.com/user-attachments/assets/6e6fcf94-91df-4f0f-8936-7639372d2f61" />

<img width="505" height="468" alt="MB_v1_back" src="https://github.com/user-attachments/assets/3c87e954-cd57-4a87-8d75-f7a1b9186ea7" />

**Main features:**
- **Microcontroller:** 480Mhz STM32H743 in TFBGA100 package, 1MB RAM 2MB Flash 
- **Sensors:** ADXL343 accelerometer and 3x TOF sensor 
- **Actuators:** UART COM for each STM32 FOC controller
- **Power Management:** 2.5A 5V buck, separate 3.3V ldo between mcu and accelerometer.  
- **Communication:** SPI / UART / Wireless link via esp32-C6
- **Features:** Up to 1Gb nand flash of 512Mb Nor flash 

###### V2
<img width="413" height="439" alt="MB_v2_Front" src="https://github.com/user-attachments/assets/f4dd61f5-7272-493a-889a-9575a6816da9" />

<img width="407" height="431" alt="mb_v2_back" src="https://github.com/user-attachments/assets/1b39e27e-b79e-4786-a5c8-040d7c2a8e96" />

This version improved the audio part by adding a micro sd card wich replaced the previous NOR flash in order to get faster read and write speed, on top of that we moved all the **audio processing into a faster esp32-S3** wich communicates with the stm32 by a uart link, it allows us to get **real AI functionnality** by using wifi link and also communicate with the **STM32 that controls all of the Robot's movements**.  
Finally this board is designed as a two board stack with the FOC controller v2.

**Main features:**
- **Microcontroller:** 480Mhz STM32H743 in TFBGA100 package, 1MB RAM 2MB Flash 
- **Sensors:** ADXL343 accelerometer and 3x TOF sensor, ICS43434 I2S mems microphone 
- **Actuators:** UART COM for each STM32 FOC controller
- **Power Management:** 2.5A 5V buck, separate 3.3V ldo between mcu and accelerometer.  
- **Communication:** SPI / UART / Wireless link via esp32-s3
- **Features:** Sd card port and esp32-s3 to manage all the audio part with class d amplifier max99357 and microphone in order to get AI to answer via a wifi call to an api. 

##### FOC STM32 based Controller

###### V1

<img width="494" height="451" alt="Front_foc_v1" src="https://github.com/user-attachments/assets/6b5619b5-29c1-4f13-8be5-400feb9f476e" />
<img width="453" height="441" alt="Back_foc_v1" src="https://github.com/user-attachments/assets/4ee0a33e-1e12-4118-83e6-69ec01d4831d" />
<img width="462" height="462" alt="image" src="https://github.com/user-attachments/assets/712b4d2a-a287-44da-9899-7801305328f0" />


The FOC controller is build arround a Iflight GBM2804H-100T motor, that we will use in direct drive to drive our robot using a 3 Wheel Omniwheel base. 

**Main features:**
- **Microcontroller:**  STSPIN32G0A2 built in soc with integrated STM32G031 and gate driver and opamp
- **Sensors:** MA330 absolut spi angle sensor 1uS measure time and 3us latency 
- **Features:** Uart communication to mainboard.

#### 2.1.2. Mechanics
The mechanical subsystem handles motion, traction, and physical interaction. 

**Main features:**
- **frame:** Lightweight 3D-printed or CNC-machined frame  
- **Drive System:** Differential or mecanum wheels for agile motion  
- **Mounts:** Sensor and board supports designed for modular assembly  
- **Simulation / CAD:** Designed on Onshape

<img width="439" height="324" alt="Trimetric view of Robot CAD" src="https://github.com/user-attachments/assets/44d66799-54e0-4019-a6f7-66a9d0e04330" />
<img width="462" height="372" alt="Trimetric view of PCB mainboard and robot" src="https://github.com/user-attachments/assets/d41bf95f-e37f-490f-8739-cbfe4e594bb7" />
<img width="473" height="383" alt="Top view of PCB mainboard and robot" src="https://github.com/user-attachments/assets/64269de6-15ea-4ae8-a2f1-ae855b1580eb" />

### 2.2. Software (Overview)
While hardware is the main focus, the robot also runs control and perception algorithms:

- **Real-time tasks:** Sensor acquisition, motor control, communication  
- **Sensor fusion:** Madgwick filter for attitude estimation  
- **Behavior layer:** Cat-and-mouse logic (chase / evade modes)  
- **Debug tools:** UART logs, LED indicators, optional telemetry GUI  

### 2.3. Bottleneck

However the STM32G0 are too slow for proper sensored FOC control, even after switching to low level functions, so we had to abandon using brushless motors and switch to DC ones instead to ensure that we will be able to drive the robot. 

## 3. DC omniwheel

### 3.1. New Motor control PCB

We kept the V2 Mainboard but had to drop the idea of a V2 FOC motor control board and instead use a single STSPING4 STM32 to send the PWM to the DC motor drivers.

### 3.2. Adapting the frame to allocate new DC motors, and more... 

<img width="300" height="200" alt="image" src="https://github.com/user-attachments/assets/23340a68-e94b-4956-baff-3ad4bc7a9bbd" />
<img width="200" height="500" alt="image" src="https://github.com/user-attachments/assets/84ff8ed7-5d61-4b16-81f0-c62f2a844988" />

https://github.com/user-attachments/assets/3e3c2521-501a-48cf-ad38-90644dd84529


We changed the motor mounts to accomodate the motors under the PCB stack. We therefore had to abandon the hand holding the LiDAT aesthetic to compensate for the extra space occupied byt the PCB and Motor assembly.

Nevertheless, we managed to maintain a fun look by adding a mascott Labubu holder and led strips.

### 3.3. Movement strategy

#### 3.3.1. Cat mode

The robot would behave like a "heat seeking missile" meaning that it would target the ennemy robot, thanks to a lidar filtering and clusteriwation algorithm, and move towards it while adjusting it's orientation so that the ennemy robot stays in the directional axis of the movement bearing:  

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/56b9fda2-c68d-4893-a54e-501124364802" />

Of course the robot being three way symmetrical it would choose it's bearing axis according to the ennemy's potition so that the closest bearing gets chosen as the "front" to minimize turning time.

Table edge detection halts the robot's movement and waits for a new target (should not happen thanks to very selective filtering)

#### 3.3.2. Mouse mode

The robot would pivot in such a way that the ennemy robot is aligned with the 180 degree LiDAR bearing and maintain a distance of at least 50cm.  

Table edge is handled by "reflecting" the robot to the  largest mid-angle between the table's edge line and the approaching ennemy's relative position, much like the "bouncing DVD logo".

### 3.4. Issues

Everything was advancing relatively smoothly the PCBs were promptly assembled and tested. Despite our attention to detail we could not have anticipated the mechanical issues that would arise from the use of low temperature bismuth based soldering paste.
The solder points were not mechanically sound leading to smd connectors breaking off and even bridging in between the USART and the Power connection underneath the connector during reheating while adjusting the solder points.  This unfortunately led to the loss of several STM32H7 chips delaying our work while we searched for the issue cqused by the auxiliary PCB, whereas our assumptions were, in order of testing:
- STM32H7 BGA pads bridging
- Mainboard connector bridging
- Motor board connector bridging

Thus leading to the loss of several pricey components.

Furthermore after some time one of the encoders from the DC motors stopped working. We had to spend some precious time to try and locate the issue, resoldering components, cswapping connectors, swapping motors, debugging code. To this day the issue has not been identified with certitude, however we suspect a manufacturing issue that led to a delamination on the single, non redundant, via transmitting the encoder logic... another issue wich could not have been anticipated.

With the short time we had left we decided that assembling a new board was not ideal, especially with the fast approaching holidays and final exams, all the more if we factor in the potential breaking points of the assembly process (mechanically and thermally fragile connectors leading to the use of sub-optimal bismuth solderpaste, potential unforeseen PCB manufacturing issues). Thus, we opted for a complete mechanical redesign to avoid further time consuming and error inducing PCB assembly, moving from a three wheeled robot to a simple two wheeled robot.

## 4. Regular DC

It is with great regeret and disappointment that we opted for the failsafe route: basic geometry, relying on the leftover functionalities of our already assembled PCBs.

### 4.1. Hardware

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/adc10831-b124-4d93-b555-bde5916f606c" />

Here is the final aesthetic of the robot, granted, this version is much less elegant and complex, but that is it's vantage point, its simplicity has allowed us to obtain a functional result, after exploring many challenging and most educational approaches. 

## 4. Strqgtegy Implementqtion

### 5.1 Overview of Cat/Mouse Mode

In cat mode the robot acts like a heat seeking missile, meaning that when the Lidar detects an object the robot orients itself so that the object sits at a 180° bearing on the Lidar reading, meaning that the object is directly in front of the robot. When a 5° orientation precision is reached the robot moves forward and only stops to reorient itself if the orientation error becomes greater than 5°.

In mouse mode we adopt a pseudo-random flee method. The robot orients itself so that the cat sits behind it and moves forward when the difference in distance becomes lower than 50cm. If the side of a table is met the robot reorients itself to face the greatest half of the angle formed between the table's border and the cat's bearing position, thus always fleeing where there is more space.

### 5.2 LiDAR handling

The LiDAR is spun at roughly 8Hz guaranteeing a 1° accuracy and fast response. After parsing the RX data is stored in a 360 uiint32 long list, each index increment analog to a 1° increment, only the points ranging from 0 to 3m are stored. Then a clusterization algorithm detects and filters the objects of diameter greater than 3cm and smaller than 8cm, which could be the opposing robot's LiDAR. Then the detected clusters are classified according to their computed size in order to isolate the most likely candidate to be the ennemy's LiDAR.

### 5.3 Motor control

Thwo PCBs are mounted, the top one is equipped with the STM32H7 and ESP32S3 for great computing capabilities and handling Wifi and Buetooth connectivity as well as aesthetic equipments such as LEDs, Screens and Speakers. The STM32H7 communicates the movement commands to the STM32G4 mounted on the bottom PCB at a 200Hz frequency, and the bottom PCB handles the motor control PIDs and returns the speeds for the STM32H7 to compute the odometry.

### 5.4 Additional PIMP mode

In an attempt to spice things up we experimented with the cinematics of a two omniwheeled robot, more particularly its ability to drift by taking advantage of the lateral movement allowed by the rollers. Hence by advancing and turning simultaneously it is possible to crab walkj in a circle. Hence a third state that although not as elegant as a computed evasion from the cat could be used to run fast circles around the cat when in mouse mode.
