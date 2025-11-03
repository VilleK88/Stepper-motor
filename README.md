# Exercise 3 - Stepper motor  
  
In this exercise you need to calibrate the stepper motor position and count the number of steps per
revolution by using an optical sensor. The reducer gearing ratio is not exactly 1:64 so the number of steps
per revolution is not exactly 4096 steps with half-stepping. By calibrating motor position and the number of
steps per revolution we can get better accuracy.  

Connect the stepper motor driver board to the 6x1 pin connector on the development board so that motor
connector is facing up. Connect motor connector to driver board and the dispenser base grove cable to
ADC_1 connector.  

  • Opto fork – GP28 – Configure as an input with pull-up  
  • Stepper motor controller – GP2, GP3, GP6 and GP13  
    o All four pins are outputs  
    o Pins are connected to the stepper motor driver pins IN1 – IN4  
    
Implement a program that reads commands from standard input. The commands to implement are:  
  • status – prints the state of the system:  
    o Is it calibrated?  
    o Number of steps per revolution or “not available” if not calibrated  
  • calib – perform calibration  
  o Calibration should run the motor in one direction until a falling edge is seen in the opto fork  
    input and then count number of steps to the next falling edge. To get more accurate results  
    count the number of steps per revolution three times and take the average of the values.  
  • run N – N is an integer that may be omitted. Runs the motor N times 1/8th of a revolution. If N is  
    omitted run one full revolution. “Run 8” should also run one full revolution.
