# Irrigation Embedded Control System

## Description
This project is an embedded control system designed for managing irrigation effectively. Utilizing a Nucleo F401RE board, this system integrates various sensors and actuators through a custom-designed PCB, enabling automated water distribution based on real-time environmental data.

## System Overview
- **MCU Controller**: Nucleo F401RE Development Board
- **Sensors**: Interfaces for US100 water depth sensor and other environmental parameters
- **PMW Motor Control**: Includes L9110 motor speed control for pumps and MG90S servo motor control for valves
- **User Interface**: Input and indicator interfaces for real-time control and monitoring through UART protocol

## Hardware and Software Requirements
- **Nucleo F401RE Board**
- **Custom PCB**
- **Sensors and Actuators as specified in the design**
- **Recommended IDE**: STM32Cube 1.13.2

## Schematic
![image](https://github.com/b57zheng/Irrigation_Embedded_System/assets/98293562/27f361c9-77d5-42d0-baa7-69a5a71ab404)

## PCB Layout
![image](https://github.com/b57zheng/Irrigation_Embedded_System/assets/98293562/0d413e1d-5e44-4656-b292-ce61a25b20dd)
![image](https://github.com/b57zheng/Irrigation_Embedded_System/assets/98293562/c79fdecf-29c5-4000-bb45-4394bec07f31)
![image](https://github.com/b57zheng/Irrigation_Embedded_System/assets/98293562/f74db943-f429-45e2-9c53-7241a2b4181b)

## Installation
1. Assemble the hardware on the custom PCB following the schematic provided.
2. Flash the Nucleo board with the firmware included in the repository.
3. Connect all peripheral components as per the connection diagrams.

## Usage
### Setup Mode
- Reset the Nucleo board to enter the setup mode.
- Configure each zone and inlet by setting the PWM values through the terminal interface as detailed in the setup guide.

### Run Mode
- Start the system by setting the current wall clock time.
- The system operates automatically based on the preset schedules, adjusting the water distribution according to the sensor inputs.

## Contributing
Contributors are welcome to propose improvements to the system.

## Acknowledgments
- Thanks to the University of Waterloo for supporting this project.
