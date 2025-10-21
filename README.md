# Cat-and-Mouse Robot Project

## Introduction
This project aims to design and build a **mobile robotic system** capable of playing a “cat and mouse” pursuit game.  
It combines **real-time embedded control**, **sensor fusion**, and **mechanical design** to create an autonomous robot capable of chasing or evading another robot in a dynamic environment.
In addition tothat we built ESP32-S3 based audio system capable of **listening by a mic** and answering using **AI based model using wifi connection and online server.**

## Hardware :

The hardware design is divided into two main parts:

### 1.Electronics
The electronic subsystem provides sensing, control, and communication between the robot’s components.
#### Motherboard

##### V1


**Main features:**
- **Microcontroller:** STM32-based control board running FreeRTOS  
- **Sensors:** Accelerometers, gyroscope, magnetometer, distance sensors  
- **Actuators:** DC or BLDC motors with FOC drivers  
- **Power Management:** Battery regulation and protection circuits  
- **Communication:** SPI / UART / CAN / Wireless link  


### 2.Mechanics
The mechanical subsystem handles motion, traction, and physical interaction.

**Main features:**
- **Chassis:** Lightweight 3D-printed or CNC-machined frame  
- **Drive System:** Differential or mecanum wheels for agile motion  
- **Mounts:** Sensor and board supports designed for modular assembly  
- **Simulation / CAD:** Designed in Fusion 360 / SolidWorks  

<!-- Add a render or photo -->
![Mechanical Design](docs/images/mechanical_design.png)

---

## Software (Overview)
While hardware is the main focus, the robot also runs control and perception algorithms:

- **Real-time tasks:** Sensor acquisition, motor control, communication  
- **Sensor fusion:** Madgwick filter for attitude estimation  
- **Behavior layer:** Cat-and-mouse logic (chase / evade modes)  
- **Debug tools:** UART logs, LED indicators, optional telemetry GUI  

---

## 📁 Repository Structure

