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
<img width="497" height="475" alt="MB_v1_front" src="https://github.com/user-attachments/assets/6e6fcf94-91df-4f0f-8936-7639372d2f61" />

<img width="505" height="468" alt="MB_v1_back" src="https://github.com/user-attachments/assets/3c87e954-cd57-4a87-8d75-f7a1b9186ea7" />

**Main features:**
- **Microcontroller:** STM32-based control board running FreeRTOS  
- **Sensors:** Accelerometers, gyroscope, magnetometer, distance sensors  
- **Actuators:** DC or BLDC motors with FOC drivers  
- **Power Management:** Battery regulation and protection circuits  
- **Communication:** SPI / UART / CAN / Wireless link  
##### V2

**Main features:**
- **Microcontroller:** STM32-based control board running FreeRTOS  
- **Sensors:** Accelerometers, gyroscope, magnetometer, distance sensors  
- **Actuators:** DC or BLDC motors with FOC drivers  
- **Power Management:** Battery regulation and protection circuits  
- **Communication:** SPI / UART / CAN / Wireless link

#### FOC STM32 based Controller

##### V1

<img width="494" height="451" alt="Front_foc_v1" src="https://github.com/user-attachments/assets/6b5619b5-29c1-4f13-8be5-400feb9f476e" />
<img width="453" height="441" alt="Back_foc_v1" src="https://github.com/user-attachments/assets/4ee0a33e-1e12-4118-83e6-69ec01d4831d" />


##### V2

In developpment...


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

