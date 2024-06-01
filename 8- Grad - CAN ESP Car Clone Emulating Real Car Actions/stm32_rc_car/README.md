# RC Car Control

----
# Issues
## 1. UART ISR Not working
- ### Environment
  - FreeRTOS Kernel V10.0.1 
  - UART uses DMA
  - DMA Interrupt Priority: `0`
  - `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` = `5`
  - `configLIBRARY_LOWEST_INTERRUPT_PRIORITY` = `15`
  


- ISR fires but when calling any API `..FromISR()` the whole system halts
- When DMA set priority set: `6`
  - ISR doesn't fire at all, although it's in the range `5 - 15`, 
  - _with 5 being the highest priority_ 

   
- ### Solution:
- Set DMA ISR priority to exactly `5` so that the ISR will fire and you will be able to call `...FromISR()` APIs safely without getting stuck in the `port.c@677` 👇

  		configASSERT( ucCurrentPriority >= ucMaxSysCallPriority );
---
## 2. DC Motors doesn't run on PWM duty less than 70%
- At duty cycle less than 70% the average DC voltage is less than the required to kickstart and run the motors this is due to the non-linearity of the L298n motor driver chip, therefore you'll have to map all the PWM duties to more than or equal 70% for the motors to run.
----


## 3. Undefined behavior when motors are ran with speed over 80% or their direction is changed suddenly
- This was clearly a `hardware issue`, at first we thought there isn't enough amperage to support all the motors on full speed however after measuring the motors max current and using 2A DC adapter the issue still persisted.
- `Solution:` purchasing `100nF capacitors` and soldering one on each motor as a bypass capacitance dealt with the reverse-conducting current and overcame the issue with energy flowing back in the circuit due to the sudden reverse in polarity of the motors and supporting high current being drawn by them 
- [Read more here](https://www.monolithicpower.com/en/dc-motor-system-input-capacitor-recommendations-and-discharge-circuit-og-mpq6526-and-mpq6527-family)