# Irrigation Embedded Control System

## Description
This project is an embedded control system designed for managing irrigation effectively. Utilizing a Nucleo F401RE board, this system integrates various sensors and actuators through a custom-designed PCB, enabling automated water distribution based on real-time environmental data.

## System Overview
- **MCU Controller**: Nucleo F401RE Development Board
- **Sensors**: Interfaces for US100 water depth sensor and other environmental parameters
- **PMW Motor Control**: Includes L9110 motor speed control for pumps and MG90S servo motor control for valves
- **Buffers**: Includes CD74HCT541 and CD74HC4050 for logic level translation
- **User Interface**: Input and indicator interfaces for real-time control and monitoring through UART protocol

## System Schematic
![image](https://github.com/user-attachments/assets/1b616fff-d22a-4726-9e12-a1deffbf70ab)

## PCB Layout
![image](https://github.com/user-attachments/assets/974e484e-5132-46c6-b568-1ab629f464a5)
![image](https://github.com/user-attachments/assets/70efa69a-b359-47cb-956d-dc25584625b4)
![image](https://github.com/user-attachments/assets/f84d32f4-228c-4d2c-948e-d154d83c699a)

## Hardware and Software Requirements
- **Nucleo F401RE Board**
- **Custom PCB**
- **Sensors and Actuators as specified in the design**
- **Recommended IDE**: STM32Cube 1.13.2

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
