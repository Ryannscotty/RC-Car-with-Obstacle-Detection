# RC-Car-with-Obstacle-Detection


Term Project Overview


This project is about the innovation of robotics and embedded systems that we find ourselves interacting with in our everyday lives. The project consists of an RC car that is about to detect obstacles from a certain specified distance and stop before any collision occurs to the vehicle. By the use of an bluetooth app via personal computer you will be able to command character directions to the RC car to enable the movement, This RC car is capable of having the functions Forward, Backward, Left, Right and Stop functions and the ultrasonic sensor will also control a automatic stop if obstacle is detected within the distance threshold. The aspirations and goals I have for this project is that as the world we know and live in become more immersed into integrating technology into our everyday lives I noticed autonomous and self - working cars are more and more becoming a necessity to make driving safer and easier, having obstacle avoidance on cars can help save lives.

![Screen Shot 2022-04-14 at 8 56 39 PM](https://user-images.githubusercontent.com/97707478/163516006-2b468fa2-d713-4aa1-83e4-c2ae44fb6f60.png)

 

![Screen Shot 2022-04-14 at 8 48 42 PM](https://user-images.githubusercontent.com/97707478/163515313-5457a4b7-4df6-4649-aa1d-56fc3c458936.png)


System Architecture

![Screen Shot 2022-04-14 at 8 53 18 PM](https://user-images.githubusercontent.com/97707478/163515620-b17ae7ee-6183-4ae9-8ef6-922372f6e092.png)

 Figure 1: RC car Block Diagram
 
 
PORT A
This port contains the 1st Motor Driver(LM298N) and the pins being used inside this port module are PA6, PA7, and PA2-PA5. PA6 is the connected to the ENA pin of the LM298N driver and this pin will send PWM duty cycle signal from the TM4C123GH6PM to the front-left wheel and the pins PA2 and PA3 are connect to the IN2 and IN3 pins of the LM298N driver which controls the direction polarity of the output
 motor. PA7 is connected to the ENB pin of the motor driver which sends the PWM duty cycle signal to the front-right wheel while the pins PA4 and PA5 control the polarity of the motor.
 
PORT D
This port contains the 2nd Motor Driver(LM298N) and the pins being used inside this port module are PD0, PD1, and PD2,PD3, PD6, and PD7. PD0 is the connected to the ENA pin of the LM298N driver and this pin will send PWM duty cycle signal from the TM4C123GH6PM to the front-left wheel and the pins PD2 and PD3 are connect to the IN2 and IN3 pins of the LM298N driver which controls the direction polarity of the output motor. PD1 is connected to the ENB pin of the motor driver which sends the PWM duty cycle signal to the front-right wheel while the pins PD6 and PD7 control the polarity of the motor.

PORT C
The Port C module contains the functions of the HC-06 bluetooth module and the pins being used for its implementation as well. This port will be using the UART serial communication terminal to transmit and receive character bit information. This bluetooth module consists of pin functions like VCC, GND, RX, TX and the RX pin is connected to PC5 of the TM4C123GH6PM microcontroller while the TX pin is connected to PC4 of the microcontroller.

 PORT B
The PORT B module interfaces with the HC-SR04 Ultrasonic sensor, and this sensor can send a trigger sound pulse duration at 10us seconds at 60ms a time and by doing this, an echo will be received and this duration can be used to measure distance. The sensor consists of an VCC, GND, Trigger, and Echo pins, the trigger pin is connected to the PB6 pin of the TM4C123GH6PM microcontroller and this pin is also configured with the TIMER module of the microcontroller as well. The Echo pin is connected to the PB2 pin of the microcontroller that captures the pulse duration of the trigger.

RC Car FlowChart
 
 ![Screen Shot 2022-04-14 at 8 54 57 PM](https://user-images.githubusercontent.com/97707478/163515767-5847d64f-2425-486e-a861-f3a8364b8089.png)

 
This flowchart contains the initialization of all the software modules involved with the project. We store these initialization functions inside the main function of the program which will permanently start up the sequence for running the specified modules of the project. The initialization functions start off with the PORT function start up for the ports described in the block diagram information above. After that we have the UART initialization function which runs the start up for the UART1 module at a 9600 Baud Rate for serial communication via bluetooth hardware module. The PWM( Pulse-Width-modulation) start-up initialization used PWM0 with GEN2 and GEN3 driving the PWM outputs. We also have the TIMER module function initialization that runs the start-up for the trigger, echo and sensor distance.

System Interrupts



We Have two software interrupts being implemented in this project, one is the UART interrupt and this interrupt is triggered when it receives a character from the bluetooth app via PC. Inside the UART interrupt handler function has the ability to clear the interrupt flag of the UART then check if there are characters available to be read on the UART1 data register and if there are characters to be read it will began to transmit these character to the serial terminal of the PC and trigger the direction movement of the RC car. Conditional statements are being used to implement direction control for the RC car, if the data that is being read equal “ F”, “ B”, “L”, “R” or “S” the PWM functions “FORWARD”, “BACKWARD”, “LEFT”, “RIGHT” and “STOP” will be implemented accordingly.

We have the 2nd interrupt being implemented inside this project which is the Trigger interrupt that contains the 10us pulse duration trigger from the ultrasonic sensor, this interrupt is configured with the TIMER3A module of the TM4C123GH6PM microcontroller, the module is in periodic count-down mode which is set at a period of 60ms. This means the interrupt will be triggered every 60ms and the handler function will set the trigger pin of the sensor HIGH for a duration of 10us then turn LOW.

We have another two important functions that are being involved with the RC car project, and these functions play a crucial role in getting the sensors to work properly to detect obstacles as efficiently as possible. The first function that we will discuss will be the Sensor_Distance function, which consists of reading the echo pin of the ultrasonic sensor. The ultrasonic sensors echo pin needs to receive a pulse duration of 8 burst before the device will turn HIGH and with that rising edge event we can capture the beginning of the pulse duration and with a LOW signal from the echo pin we can capture a falling edge event, by taking the difference of these two edge captured events we can measure the distance of an obstacle relative to us.
The other important software function of this system is the Microsencod_delay function that consists of the implementation of a delay of 10us for the use of the trigger function of the ultrasonic sensor. This function is configured with the use of the TIMER2A module which is set in the periodic count-down mode with a period of 1us second. We can implement a 10us delay by declaring an integer variable named “time” inside the
Microsecond_delay caller. Also a For-loop is used to clear the periodic flag register for the amount of time declared with use of the “ time” variable. The loop will detect the falling edge of the TIMER, then clear to flag for as long as needed.

![Screen Shot 2022-04-14 at 8 58 14 PM](https://user-images.githubusercontent.com/97707478/163515987-c960ee5c-b060-408d-90c5-1963b9397163.png)

