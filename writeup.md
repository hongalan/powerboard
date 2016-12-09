# Powerboard, An Intelligent Powered Longboard
## *Alan Hong, Northwestern University MSR Final Project 2016*

[ ![Image of driveable_board](https://www.dropbox.com/s/x7l0krllq7ypzek/07_driveable.jpg?raw=1)](
https://youtu.be/V36Dq53fRM4 "video demo")

~~~
Table of Contents
1. Goal and Motivation
2. Project Overview
3. Design Walkthrough
   a. Mechanical
   b. Electrical
   c. Software
4. Results and Discussion

~~~


### **1. Goal and Motivation:**  

In brief, the goal of this project is to create a self-navigating longboard with shared autonomy. The vision is that the user may dictate a desired end location in a pre-generated map of the surroundings containing a route to this destination. The longboard then localizes itself on the map and provides feedback through a steering mechanism, guiding the user to this set destination while detecting and avoiding obstacles such as objects and people. 

The user and the longboard have shared control of speed and direction: 
*The user sets desired speed by means of a hand-held joystick, but the longboard can slow or stop itself if it detects an obstacle.
*The user can lean to determine direction of turns, but the longboard's steering mechanism provides resistance to bias the user away from walls and towards the direction of turns as necessary.

This project was designed to expose myself to the challenges presented in high-voltage electronics, sensor integration, component sizing, and navigation principles such as Bayesian filters and planning algorithms. It also introduced questions of human-robot interaction and how a robot must behave to safely interact with its user.


### **2. Project Overview:**  

The longboard is propelled by a pair of high-torque, 3-phase, brushless DC motors, each of which receives modulated power from an ESC ("electronic speed control"). The ESC provides power according to a PPM signal provided by a microcontroller; this signal is determined by a PD control loop accepting feedback from RPM sensors attached to both motors. Finally, a custom-made PCB was made to safely interface the low and high voltage components.

Navigation and obstacle-avoidance is possible using sensors and a PC running navigation tools through ROS. These sensors consist of a Hokuyo scanning laser range finder, an IMU, and the aforementioned RPM sensors.

The steering mechanism is driven by a geared brushed DC motor which actuates the trucks of the longboard by means of a capstan drive and bowden cable.

Currently, the sensors and actuators have all been integrated such that the PC running ROS receives sufficient data to perform sensor fusion, localize the robot, and map the observed environment. Work now is focused on incorporating this information with ROS navigation tools that run Bayesian filters and path-planning algorithms to navigate the longboard robot and provide the user with feedback.


### **3. Design Walkthrough:**

[A detailed list of compenents and vendors can be found here.](https://github.com/hongalan/powerboard/blob/master/components_list.txt)

[My Arduino code and referenced libraries can be found here.](https://github.com/hongalan/powerboard-arduino)

#### **A. Mechanical**

The motors propelling the longboard are two 170kV brushless motors, chosen for their compact size and high torque constant. An ESC typically use the back-EMF produced by the motor to determine the appropriate modulation of power, but in addition, each motor also contains Hall sensors to indicate to the ESC what modulation is optimal.
The two motors then drive the rear wheels of the longboard using a timing belt at a 13:36 gear ratio to further improve the torque applied.

![Image of motor](https://www.dropbox.com/s/p9zvrqz3vefb8cx/04_brushless_motor.jpg?raw=1)

The steering mechanism makes use of a Molon geared motor controlled by an H bridge. A timing belt between pulleys providing a 30:18 gear ratio reduces the torque applied to the longboard trucks, but improves the backdriveability of the motor as the user leans and provides torque in the opposite direction. This decision prioritized user safety, as the user's desired direction can at any time outweigh the feedback provided by the steering mechanism, preventing the longboard from forcing the user off a cliff or into an undetected obstacle.

![Image of steering_mechanism](https://www.dropbox.com/s/73w7ywyhl7rp5vl/04_steering_mechanism.jpg?raw=1)

###### _Lessons_
Originally, a pair of 300kv brushless motors were used to drive the motor. At that point however, the ESC's were damaged. This was presumably caused by an overcurrent issue, though this has not been verified. The motors had a relatively low torque constant for this application, making it more prone to stalling and drawing an unexpected amount current for an extended period of time. Thereafter, motors with a higher torque constant were chosen, and a current sensor was installed to ensure operating current remained within acceptable ampherage levels.


#### **B. Electrical**

The ESC's are supplied power by a 6-cell, 22.2V, LiPo battery, and are given a PPM signal by an Arduino Mega. To protect the low-voltage components, they are isolated from the high-voltage components by means of a custom-made PCB designed in KiCad, featuring a set of optocouplers, current sensors, a 5V regulator, and safety shutoff mechanisms including an emergency stop and dead man's switch.

![Image of pcb](https://www.dropbox.com/s/o434b1s8a9orjfm/02_pcb_v2_populated.jpg?raw=1)

The Arduino is responsible for reading the position of the joystick potentiometer and providing a ramp acceleration to a corresponding speed. It also collects readings from the current sensors to ensure that the operating current remains within an acceptable level; if the current becomes too high, it pulls the motor speed down until current is again below this level. Finally, the Arduino takes readings from the IMU and RPM sensors and sends this data to the PC.
If you are interested in wiring components to the Arduino as I had done, my pin mapping are specified in [my main Arduino script.](https://github.com/hongalan/powerboard-arduino/blob/master/src/pcb_main.cpp)

Once the wiring is completed, the components are placed underneath the longboard in an enclosure for protection.

![Image of enclosure](https://www.dropbox.com/s/vn57v63w1gm03l3/06_longboard_enclosure.jpg?raw=1)


###### _Lessons_
Though the Arduino is less powerful than a PIC microcontroller, the Arduino was chosen primarily for the the broad set of libraries available for interfacing with sensors and transmitting ROS messages. The Arduino's performance was seen to be perfectly acceptable, and so, though initial development was done on the PIC, the Arduino was chosen purely for expediency.


#### **C. Software**

The [Arduino script](https://github.com/hongalan/powerboard-arduino/blob/master/src/pcb_main.cpp) should be compiled with the [provided libraries](https://github.com/hongalan/powerboard-arduino/tree/master/lib) and then uploaded to an Arduino Mega contains several parameters available for reconfiguration.

    __MPS_REF_INCREMENT //Determines maximum acceleration
    __MPS_MAX   //Maximum speed in meters per second
    __CURRENT_LIM //Specifies maximum permissible operating current in amps
    __ROS_FLAG  ////Set true if Arduino is to be connected to PC and publish ROS messages; if PC is not connected, set false

The data sent from the Arduino to the PC is then accessed in [ROS](http://wiki.ros.org) to perform localization, mapping, and navigation. My `powerboard_pcl` package [can be found here](https://github.com/hongalan/powerboard). For details on its installation and usage, refer to the [README file](https://github.com/hongalan/powerboard/blob/master/README.md) .

![Image of hokuyo](https://www.dropbox.com/s/zb80ywd3jlqficv/08_hokuyo.jpg?raw=1)

###### _Lessons_
Initially, navigation was to be performed using a point cloud representation of the environment. However, it was seen that a 2D laser scan of the environment was sufficient to localize the system. The point cloud may be used in the future to recognize more complex landmarks and to classify obstacles.


### **4. Results and Discussion:**  

Speed control has been reliably implemented and incorporates ramping behavior to prevent the user from experiencing sudden acceleration.

ROS receives laser scan data at ~18hz and receives speed and orientation measurements at ~20hz. Through sensor fusion, the system successfully localizes itself in the generated map. Effort is now focused on combining this with ROS navigation tools and using the steering mechanism to interact with the user through shared autonomy.
    
![Image of gmapping](https://www.dropbox.com/s/i5ho9jo01eoibly/09_hallway_map_d110.jpg?raw=1)
