# EUI_AVR_sunrise_alarm
 | Name  | Code |
| ------------- | ------------- |
| Ahmed Hisham  | 1023  |
| Nada Maher  | 1139  |
| Norhan Mohamed  | 764  |
| Salma Mohamed | 199  |

video link is [here](https://youtu.be/WwDwOOemtg8?si=VkJGsz_3BjQq5BVa)

## Project Description: Multiple Timer Wake-Up System with LED and Buzzer

The project aims to create a wake-up system that allows users to set multiple timers to wake them up at different times using an ATmega32 microcontroller. The system utilizes LEDs and a buzzer to gradually wake up the user in a gentle manner. Additionally, the system provides a status display on an LCD screen to show the active alarms.

### Key Components:
1. ATmega32 Microcontroller: The ATmega32 microcontroller serves as the central processing unit of the system. It controls the overall functionality, manages the timers, and interacts with the other components.

2. Timers: The system allows users to set multiple timers with different wake-up times. Each timer can be programmed independently, specifying the desired wake-up time.

3. LEDs: A set of LEDs is used to simulate a sunrise effect. As the wake-up time approaches, the LEDs gradually increase their brightness, mimicking the rising sun and providing a natural wake-up experience.

4. Buzzer: A buzzer or a speaker produces an audible alarm once the wake-up time is reached, waking up the user.

5. LCD Screen: An LCD screen is used to display the status of the alarms. It shows the active alarms, indicating the wake-up times that have been set.

### Functionality:
1. Timer Configuration: Users can set multiple timers through the system's interface, specifying the desired wake-up time for each timer.

2. LED Gradual Brightness: As the wake-up time approaches, the LEDs start to light up gradually, emulating the sunrise. The brightness level is predetermined and cannot be adjusted by the user.

3. Buzzer Alarm: When the wake-up time is reached, the buzzer starts producing an alarm sound to wake up the user.

4. Timer Management: Users have the ability to cancel or modify existing timers through the system's interface, providing flexibility in adjusting wake-up times based on changing schedules or preferences.

5. Light-Dependent Resistor (LDR) Interaction: The wake-up sequence can be temporarily halted by the user placing their hand on an LDR sensor. The LDR detects the presence of the user's hand, causing the wake-up sequence to pause. This feature allows the user to control the wake-up process and adapt it to their specific needs.

6. Alarm Status Display: The system utilizes an LCD screen to display the status of the alarms. The screen shows the active alarms, providing a visual representation of the wake-up times that have been set.

By utilizing an ATmega32 microcontroller and incorporating features such as timer management, LED-based sunrise simulation, a buzzer alarm, an LCD screen for alarm status display, and LDR interaction, the wake-up system provides a gentle wake-up experience for users. The system allows users to set multiple timers, manage wake-up times, and view the status of alarms through the LCD screen, while simulating a natural sunrise.
