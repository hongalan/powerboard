# Powerboard - An Intelligent Powered Longboard
## *Alan Hong, Northwestern University MSR Final Project 2016*

![Image of driveable_board](https://octodex.github.com/images/yaktocat.png)

~~~
Table of Contents
1. Goal and Motivation
2. Project Overview
3. Design Walkthrough
   a. Mechanical
   b. Mechatronic
   c. Software
4. Results and Discussion

~~~


### **1. Goal and Motivation:**  

In a sentence, the goal of this project is to create a self-navigating longboard with shared autonomy. The vision is that the user may dictate a desired end location in a pre-generated map of the surroundings containing a route to this destination. The longboard then localizes itself on the map and provides feedback through a steering mechanism, guiding the user to this set destination while detecting and avoiding obstacles such as objects and people. 

The user and the longboard have shared control of speed and direction: 
*The user sets desired speed by means of a hand-held joystick, but the longboard can slow or stop itself if it detects an obstacle.
*The user can lean to determine direction of turns, but the longboard's steering mechanism provides resistance to bias the user away from walls and towards the direction of turns as necessary.

This project was designed to expose myself to the challenges presented in high-voltage electronics, sensor integration, component sizing, and navigation principles such as Bayesian filters and planning algorithms. It also introduced questions of human-robot interaction and how a robot must behave to safely interact with its user.


### **2. Project Overview:**  

The longboard is propelled by a pair of high-torque, 3-phase, brushless DC motors, each of which receives modulated power from an ESC ("electronic speed control"). The ESC provides power according to a PPM signal provided by a microcontroller; this signal is determined by a PD control loop accepting feedback from RPM sensors attached to both motors. Finally, a custom-made PCB was made to safely interface the low and high voltage components.

Navigation and obstacle-avoidance is possible using sensors and a PC running navigation tools through ROS. These sensors consist of a Hokuyo scanning laser range finder, an IMU, and the aforementioned RPM sensors.

The steering mechanism is driven by a geared brushed DC motor which actuates the trucks of the longboard by means of a capstan drive and throttle cable.

Currently, the sensors and actuators have all been integrated such that the PC running ROS receives sufficient data to perform sensor fusion, localize the robot, and map the observed environment. Work now is focused on incorporating this information with ROS navigation tools that run Bayesian filters and path-planning algorithms to navigate the longboard robot and provide the user with feedback.


### **3. Design Walkthrough:**

[A detailed component list and vendors can be found here.](http://google.com)
[My Arduino code and referenced libraries can be found here.](https://github.com/hongalan/powerboard-arduino)
#### **A. Mechanical**

The motors propelling the longboard are two 170kV brushless motors, chosen for their compact size and high torque constant. An ESC typically use the back-EMF produced by the motor to determine the appropriate modulation of power, but in addition, each motor also contains Hall sensors to indicate to the ESC what modulation is optimal.
The two motors then drive the rear wheels of the longboard using a timing belt at a 13:36 gear ratio to further improve the torque applied.

![Image of motor](https://octodex.github.com/images/yaktocat.png)

The steering mechanism makes use of a Molon geared motor controlled by an H bridge. A timing belt between pulleys provide a 30:18 gear ratio reduces the torque applied to the longboard trucks, but improves the backdriveability of the motor as the user leans and provides torque in the opposite direction. This decision prioritized user safety, as the user's desired direction can at any time outweigh the feedback provided by the steering mechanism, preventing the longboard from forcing the user off a cliff or into an undetected obstacle.

![Image of steering_mechanism](https://octodex.github.com/images/yaktocat.png)

###### **- Lessons**
Originally, a pair of 300kv brushless motors were used to drive the motor. At that point however, the ESC's were damaged. This was presumably caused by an overcurrent issue, though this has not been verified. The motors had a relatively low torque constant for this application, making it more prone to stalling and drawing an unexpected amount current for an extended period of time. Thereafter, motors with a higher torque constant were chosen, and a current sensor was installed to ensure operating current remained within acceptable ampherage levels.


#### **B. Electrical**

The ESC's are supplied power by a 6-cell, 22.2V, LiPo battery, and are given a PPM signal by an Arduino Mega. To protect the low-voltage components, they are isolated from the high-voltage components by means of a custom-made PCB designed in KiCad, featuring a set of optocouplers, current sensors, a 5V regulator, and safety shutoff mechanisms including an emergency stop and dead man's switch.

![Image of pcb](https://octodex.github.com/images/yaktocat.png)

The Arduino is responsible for reading the position of the joystick potentiometer and providing a ramp acceleration to a corresponding speed. It also collects readings from the current sensors to ensure that the operating current remains within an acceptable level; if the current becomes too high, it pulls the motor speed down until current is again below this level. Finally, the Arduino takes readings from the IMU and RPM sensors and sends this data to the PC.
If you are interested in wiring components to the Arduino as I had done, my pin mapping are specified in [my main Arduino script.](https://github.com/hongalan/powerboard-arduino/blob/master/src/pcb_main.cpp)

###### **- Lessons**
Though the Arduino is less powerful than a PIC microcontroller, the Arduino was chosen primarily for the the broad set of libraries available for interfacing with sensors and transmitting ROS messages. The Arduino's performance was seen to be perfectly acceptable, and so, though initial development was done on the PIC, the Arduino was chosen purely for expediency.



#### **C. Software**

The [Arduino script](https://github.com/hongalan/powerboard-arduino/blob/master/src/pcb_main.cpp) should be compiled with the [provided libraries](https://github.com/hongalan/powerboard-arduino/tree/master/lib) and then uploaded to an Arduino Mega contains several parameters available for reconfiguration.

    __MPS_REF_INCREMENT //Determines maximum acceleration
    __MPS_MAX   //Maximum speed in meters per second
    __CURRENT_LIM //Specifies maximum permissible operating current in amps
    __ROS_FLAG  ////Set true if Arduino is to be connected to PC and publish ROS messages; if PC is not connected, set false


The data sent from the Arduino to the PC is then accessed in [ROS](http://wiki.ros.org) to perform localization, mapping, and navigation. My `<powerboard>` package [can be found here](https://github.com/hongalan/powerboard).


![Image of hokuyo](https://octodex.github.com/images/yaktocat.png)


![Image of gmapping](https://octodex.github.com/images/yaktocat.png)


### **4. Lessons and Challenges:**  

* It was found that, while the classifier can be made more accurate with a large training set, this slows the classfier at runtime. This may be the case because a larger training set provides more criteria that must be checked when detecting an object. A balance was reached that returned reasonably accurate results.

* The robustness of the classifier was seen to be dependent on the diversity of images chosen for training the classifer. For instance, the camera resolution, object proximity, lighting, and orientation could be varied. However, varying these qualities too much was seen to reduce accuracy. For instance, the classifier trained on images of the object from multiple perspectives resulted in lower accuracy than that which was trained on as many images from a single perspective. I suspect similar accuracy may be accomplished with a sufficiently large dataset, but this slows the performance.

* It was seen that using a single method of classification is not as reliable as combining several methods. For instance, the Haar classifier can be very accurate in a given setting provided the parameters are tuned accordingly. However, the same parameters would fail to find the object in a different setting. To achieve robust performance across settings, the parameters were chosen to allow for a wider margin of error, producing more false positives. These false positives were addressed by comparing these results to results from the past and by comparing the color composition to a reference image.

* The primary limiting factor of the object location process was seen to be the classifier. In order to reduce the number of times that it must run, one can reduce the number of false positives early on and obviate the need to run classifier. One can also pass only a portion of the image to the classifier; this can be done anticipating the location of the object from its past locations and estimated trajectory and assigning a corresponding ROI.

* The speed of Baxter's performance is seen to suffer for relying on a path-planner to calculate the trajectory of the end-effector. It is concluded that the path-planner is not needed and can be replaced by working with the robot's Jacobian in joint-velocity space.

