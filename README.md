# Eurobot 2025 Mission Firmware

This repository contains the firmware for the Eurobot 2025 mission control system.

## Prerequisites

- Ubuntu Operating System
- STM32CubeIDE (latest version recommended)
- Git
- STM32F446RET6 Board

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd Eurobot-2025-Mission-FW
```

### 2. Open Project in STM32CubeIDE

1. Open STM32CubeIDE
2. Navigate to `File -> New -> STM32 Project from an Existing STM32CubeMX Configuration File (.ioc)`
3. In the file selection dialog, browse to the cloned repository
4. Select the `Eurobot-2025-Mission-FW.ioc` file
5. Click "Finish" to import the project

### 3. Build the Project

1. Once the project is imported, right-click on the project in the Project Explorer
2. Select "Build Project" or press Ctrl+B
3. The compiled binary will be available in the `Debug` folder

## Project Structure

- `Core/`: Contains the main application code
  - `ControlLib/`: Control system libraries
  - `Inc/`: Header files
  - `Src/`: Source files
- `Drivers/`: STM32F4 HAL drivers
- `Middlewares/`: Third-party middleware (FreeRTOS)
- `micro_ros_stm32cubemx_utils/`: Micro-ROS utilities

## Additional Information

For more details about the project or if you encounter any issues, please refer to the project documentation or create an issue in the repository.
