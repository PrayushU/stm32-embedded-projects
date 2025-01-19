
# STM32 Embedded Projects

This repository contains a collection of embedded systems projects and experiments designed for STM32 microcontrollers. Each project demonstrates specific features or techniques in bare-metal programming, focusing on real-world applications of microcontroller capabilities such as timers, UART communication, and task scheduling.

## Features and Topics
### General Projects
- **00HelloWorld**: A basic "Hello World" program to get started with STM32 microcontroller programming.
- **STM32_PLL_SYSCLK** and related projects:
  - Configuring the PLL to use as the system clock (SYSCLK).
  - Examples with different configurations, such as using HSE as the clock source.

### Timer-Based Projects
- **Time_base_100ms**: A timer-based delay implementation generating 100ms time intervals.
- **Time_base_100ms_IT**: The same as above but implemented using timer interrupts for better efficiency.
- **timer_IC_1**: Timer input capture example for measuring external signal timing.
- **timer_OC_1**: Timer output compare example for event triggering.
- **timer_OC_PWM_1** and **timer_OC_PWM_1_**:
  - Generating PWM signals using timer output compare.

### UART Communication
- **UART_Example**: Example demonstrating UART configuration and communication, including sending and receiving data.

### Task Scheduling
- **task_scheduler**: Implementation of a basic task scheduler using timers and interrupts for time-sharing multiple tasks.

### Peripheral Drivers
- **stm32f4xx_drivers**: Peripheral drivers for GPIO, UART, and other basic functionality written from scratch to interact directly with STM32 registers.

## Tools and Requirements
- **Development Environment**: STM32CubeIDE or GCC-based toolchain (`arm-none-eabi-gcc`).
- **Hardware**: STM32 microcontroller development board (e.g., STM32F4 series).
- **Debugger**: Tools like ST-LINK or OpenOCD for programming and debugging.

## How to Use
1. Clone the repository:
   ```
   git clone https://github.com/yourusername/stm32-embedded-projects.git
   ```
2. Navigate to the directory of the desired project.
3. Open the project files in your preferred IDE.
4. Compile and flash the binary to your STM32 microcontroller.
5. Follow the instructions in the project's source files or inline comments.

## Highlights
- **Modular Structure**: Each project is self-contained for easy understanding and reuse.
- **Bare-Metal Programming**: No external libraries or frameworks, focusing on direct register manipulation.
- **Practical Examples**: Covers essential microcontroller features like timers, UART, and PLL configurations.

## Contribution
Contributions are welcome! If you want to add new projects, improve existing examples, or fix issues, feel free to fork the repository and submit a pull request.

## License
This repository is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
