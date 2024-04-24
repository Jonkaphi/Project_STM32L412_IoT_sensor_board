# Project_STM32L412_IoT_sensor_board
https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board
## Summary
This repository is for the code used in the vehicle occupancy monitor PCB from the "Self Powered IoT sensor board" 3rd Year project. 

![Screenshot 2024-04-14 144821](https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board/assets/103381620/422ad319-b25d-4ee4-b1fa-c5ecad54b088=100x250)

## Code Bases Used
* [VL53L3CX Ultralow power (ULP) application programming interface (API)](https://www.st.com/en/embedded-software/stsw-img033.html)
* [RFM95W HAL based driver for STM32L0, STM32L4 and STM32F4](https://github.com/henriheimann/stm32-hal-rfm95)
* 
## Instalation
* On how to clone this repo follow this GitHub [guide](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository?tool=desktop).
* All code dependecies are in the CubeIDE project, simplest method for opening this is to use the [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html), here is also the [user manual](https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-guide-stmicroelectronics.pdf) for the software.

## Code explanation
Flowchart below shows a simplifed view of how the code works. The operational modes of the device are controller using SW1. In monoitoring mode by default the device will measure five times with intermitent sleep period of eual number util about 33 seconds elapse, and the radio sends a update message a predefined specifc frequency band with the messuremnet data from ToF sensor as well as the status of the board. In conifguration mode the on board destination and source address, as well as the updat rate can be modifed. If another LoRa enabled device 
![Picture 1](https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board/assets/103381620/528a8c67-f28a-4a88-9b1c-21e896c0a08e)

