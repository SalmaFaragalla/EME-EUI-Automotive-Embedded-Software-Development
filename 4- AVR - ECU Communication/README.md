# EUI_AVR_COMM
 | Name  | Code |
| ------------- | ------------- |
| Ahmed Hisham  | 1023  |
| Nada Maher  | 1139  |
| Norhan Mohamed  | 764  |
| Salma Mohamed | 199  |

video link is [here](https://youtu.be/wunOPRubQNo?si=L48U6-k5mUqUtd0M)

# Description
In this project, ECU 1 is responsible for controlling the LCD, potentiometer (for throttle
control), and keypad (for gear control and speed limit management), while ECU 2
manages the EEPROM (I2C) interface to store the speed limit. The 2 ECUs
communicate with each other using SPI.

# Requirements 
1. LCD
a. Main (Current Speed & Current Gear)
b. Speed Limit (ON / Off)
c. Set Speed Limit
2. Pot
a. Speed Control (0 - 250)
3. Keypad
a. Gear Control (P – R – N - D)
The speed changed only in D and R
When I turn on speed limit , ECU2 read the limit speed form EEPROM and send it to ECU1 using SPI
The speed limit only work in D , and the maximum speed in R is 30 KM/H
