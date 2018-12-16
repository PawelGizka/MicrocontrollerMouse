cmake_minimum_required(VERSION 3.12)
project(MicrocontrollerMouse C)

set(CMAKE_C_STANDARD 99)


set(CMAKE_C_FLAGS -DSTM32F411xE)

include_directories(/opt/arm/stm32/inc /opt/arm/stm32/CMSIS/Include /opt/arm/stm32/CMSIS/Device/ST/STM32F4xx/Include)


add_executable(usart2 main.c)