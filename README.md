# Exercise 1 - Dimmer  
  
Implement a program for switching all three LEDs on/off and dimming them. The program should work as  
follows:  
• SW1, the middle button is the on/off button. When button is pressed the state of LEDs is toggled.  
Program must require the button to be released before the LEDs toggle again. Holding the button  
may not cause LEDs to toggle multiple times.  
• SW0 and SW2 are used to control dimming when LEDs are in ON state. SW0 increases brightness  
and SW2 decreases brightness. Holding a button makes the brightness to increase/decrease  
smoothly. If LEDs are in OFF state the buttons have no effect.  
• When LED state is toggled to ON the program must use same brightness of the LEDs they were at  
when they were switched off.  
• If LEDs are ON and dimmed to 0% then pressing SW1 will set 50% brightness immediately.  
• If LEDs are ON and dimmed to 0% then pressing SW0 will increase the brightness.  
• PWM frequency divider must be configured to output 1 MHz frequency and PWM frequency must  
be 1 kHz.  
