# Irrigation Embedded Control System

## Description
This project is an advanced embedded control system designed for managing irrigation effectively. Utilizing a Nucleo F401RE board, this system integrates various sensors and actuators through a custom-designed PCB, enabling automated water distribution based on real-time environmental data.

## System Overview
- **MCU Controller**: Nucleo F401RE Development Board
- **Sensors**: Interfaces for US100 water depth sensor and other environmental parameters
- **Actuators**: Includes PWM L9110 motor speed control for pumps and servo motor for valves
- **User Interface**: Input and indicator interfaces for real-time control and monitoring

## Hardware and Software Requirements
- **Nucleo F401RE Board**
- **Custom PCB**
- **Sensors and Actuators as specified in the design**
- **Required Software**: ARM mbed, ST-Link drivers

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
Contributors are welcome to propose improvements to the system. Please fork the repository and submit a pull request with your proposed changes.

## License
This project is released under the MIT License.

## Acknowledgments
- Thanks to the University of Waterloo for supporting this project.
- Gratitude to all contributors and testers who have improved the system.

---

Please replace placeholders like `[Schematic Placeholder]` and `[PCB Layout Placeholder]` with the respective images or further details you want to include. Also, adjust any technical details I may have misinterpreted from your input. Let me know if there's anything else you'd like to add or modify!
