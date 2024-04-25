# IoT Parking Bay Occupancy Detection Board

https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board

## Summary

This repository is for the code used for STM32L412CBT6 in the vehicle occupancy monitor board and its Altium PCB design files for a "Self Powered IoT sensor board" project.

  

![Screenshot 2024-04-24 141626](https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board/assets/103381620/9dab08f1-4c93-4676-ab38-4c0f5108d8d7)

  
  

## Code Bases Used

* [VL53L3CX Ultralow power (ULP) application programming interface (API)](https://www.st.com/en/embedded-software/stsw-img033.html)

* [RFM95W HAL based driver for STM32L0, STM32L4 and STM32F4](https://github.com/henriheimann/stm32-hal-rfm95)

## Installation

* On how to clone this repo follow this GitHub [guide](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository?tool=desktop).

* The simplest method is opening the main folder containing the .project file with the [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) it should automatically recognise it, here is also the [user manual](https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-guide-stmicroelectronics.pdf) for the software for any further guidance.

  

## Code explanation

![Picture 1](https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board/assets/103381620/528a8c67-f28a-4a88-9b1c-21e896c0a08e)

  

* Flowchart above shows a simplified view of how the code works. The operational modes of the device are controller using SW1. In monitoring mode by default the device will measure for 500 ms five times with 6 second sleep periods in between until about 33 seconds elapse, and the radio sends an update message at a predefined specific frequency band (863 MHz) with the measurement data from ToF sensor as well as the status of the board. The packet structure can be seen below.

  

![Screenshot 2024-04-18 150937](https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board/assets/103381620/eb052fbe-2339-4a00-a6e2-265bde09bd8d)

  

* In configuration mode the default destination and source address, as well as the update rate can be modified if another Lora enabled device sends the config data packet to the specific monitoring board source address at the service frequency band (869 MHz). The expected packet structure can be seen below.  

  

![Screenshot 2024-04-18 150959](https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board/assets/103381620/558bcaeb-ff92-49e1-b572-622840e33077)

  

The flowchart above explains the code seen in the main.c file, located in path “project_STM32L412_IoT_sensor_board/Code/Core/Src”.  This file can be considered as a usable demo for the specific PCB.

  

## Known issues

Non are known if there are please use the [issue tracker](https://github.com/Jonkaphi/Project_STM32L412_IoT_sensor_board/issues).
