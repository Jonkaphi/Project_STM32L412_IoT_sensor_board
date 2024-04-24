# Project_STM32L412_IoT_sensor_board
https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board
## Summary
This repository is for the code used for STM32L412CBT6 in the vehicle occupancy monitor PCB from the "Self Powered IoT sensor board" 3rd Year project. 

![Screenshot 2024-04-14 144821](https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board/assets/103381620/422ad319-b25d-4ee4-b1fa-c5ecad54b088=100x250)

## Code Bases Used
* [VL53L3CX Ultralow power (ULP) application programming interface (API)](https://www.st.com/en/embedded-software/stsw-img033.html)
* [RFM95W HAL based driver for STM32L0, STM32L4 and STM32F4](https://github.com/henriheimann/stm32-hal-rfm95)
  
## Instalation
* On how to clone this repo follow this GitHub [guide](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository?tool=desktop).
* All code dependecies are in the CubeIDE project, simplest method for opening this is to use the [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html), here is also the [user manual](https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-guide-stmicroelectronics.pdf) for the software.

## Code explanation
![Picture 1](https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board/assets/103381620/528a8c67-f28a-4a88-9b1c-21e896c0a08e)

Flowchart above shows a simplifed view of how the code works. The operational modes of the device are controller using SW1. In monoitoring mode by default the device will measure for 500 ms five times with 6 second sleep periods inbetween until about 33 seconds elapse, and the radio sends a update messag at a predefined specifc frequency band (863 MHz) with the messuremnet data from ToF sensor as well as the status of the board.The packet structure can be seen below.

![Screenshot 2024-04-18 150937](https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board/assets/103381620/eb052fbe-2339-4a00-a6e2-265bde09bd8d)

In conifguration mode the default destination and source address, as well as the updat rate can be modifed if another LoRa enabled device sends the config data packet to the specific monitoring board source address at the service frequency band (869 MHz). The expected packet strcuture can be seen below.  

![Screenshot 2024-04-18 150959](https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board/assets/103381620/558bcaeb-ff92-49e1-b572-622840e33077)

