# WRO
this is the github post for team ...
WRO
Meet Our Team: UK Finals Participants

We are excited to introduce our team members participating in the UK finals: Parth Srivastav, Nitya Srivastav, and Jai Kumar Kairamakonda. Parth & Jai are Year 9 students, aged 14 where Nitya is year 6 student, aged 11 respectively.

About Us Parth Srivastav Parth had prior experience with WRO. He has participated in the World Robotics Olympiad (WRO) thrice before, in the Robo mission junior and elementary categories. Parth is a Year 9 student at Maiden Erlegh School. He enjoys playing piano and learning robotics including raspberrypi, and its related sensors like LiDAR, MPU6050 Gyro and Raspberrypi camera.

Jai Kumar Kairamakonda Jai is a talented Python coder with a keen interest in learning and self-improvement, both mentally and physically. He loves reading books and coding in Python during his free time.

Nitya had prior experience of Robo mission elementary and is now learning raspberrypi and sensors used on autonmous vehicle.

Our Project How It Works Our project involves:
- electrical wiring
- coding
- circuit architecture

Circuit Architecture Our WRO robot uses the Raspberry Pi 4B as the motherboard, offering the same capabilities as some laptops but in a much smaller form factor. The Raspberry Pi ensures a snappy response time. We are using Raspberry pi 4B with 8 GB RAM, which provides sufficient capabilty of performing complex tasks including supporting computer vision, in parallel to supporting LiDAR, Gyro and controlling the robots movement. We have integrated the raspberry pi 4b with a 5,000mah power bank, to ensure uninterrupted power supply for up to 8 hrs of continuous operation.  

Sensors:
Our robot is equipped with 3 sensors to provide a comprehensive view of its surroundings. For avoiding frontal collisions, we use a TF Luna a LIDAR sensor, which react extremely quickly to any object in front. These sensors can detect objects up to 8 meters ahead in brightly lit areas and 2.5 meters in darker areas. With our TF luna sending signal hundreds of times a second frontal crashing is near impossible.

Camera:
We chose 2 cameras for this build the raspberry pi camera module3 wide and the DFrobot huskylens camera. We started with huskylens, to exploit its onboard image processing capabilities. However we had challenges in integrating Huskylens with raspberrypi, which prompted us to move to raspberry pi camera. The raspberry pi camera follows a different architecture and instead of onboard image / object detection capabilities, we now rely on using Open CV libraries to process the raw video coming from the raspberry pi camera, analyse and take decisions. In terms of connectivity it connects the ribbon cable, instead of the GPIO pins. We chose the wide lens instead of the normal camera, to take a wider video snapshot and provide better situational awareness - essential for this competition. When came to running the camera, we choice ultralytics , to analysis the images, and OpenCV ,to edit the images. We had issues with downloading ultralytics, so we tried to downgrade to see if we could get passed the"externally managed environment error" but it did not help as there was limited support. So we upgraded back and tried to use an environment which worked and allowed us to download ultralytics.

LiDAR:
We are using a unidirectional TF luna LiDAR. TFluna works on time of flight principle and calculates the distance from the obstacle based on the time difference between sending an receiving a response. In our configuration, we are sending 50 pulses a second for optimum situational awareness. The LiDAR is placed at the bottom of the drivers cabin to detect obstacles. 

Robot body:
For Robot body, we started with Lego Spike Prime, building on our earlier experience but struggled with producing Ackermann steering. The connectors used to make steering wheels was fragile and broke down often. In the interest of time, we replaced it with a Remote controlled truck and connected its motor with raspberry pi via a motor controller. The RC truck we chose offered a storage place in the back to store the raspberrypi, battery, motor controller. Our choice of RC truck was also influenced by our desire for better aesthetics, and resemblance with real life vehicles than a bespoke robot with wires protruding from many directions. Each unit - raspberrypi, power bank, motor controller were seperated by lego blocks to ensure they remain in their place and do not impact other units.

Motor controller:
We use the Hailege motor controller L298N to communicate with our motors rapidly, minimizing input delay and allowing quick reactions to sensor data.
Raspberrypi camera is placed on the top of the drivers cabin to provide a view of the map 

The MPU6050 Gyro is placed just behind the camera, but in the centre of the robot to calculate veicles position and trajectory. Care was taken in ensuring the cables are not visible and the vehicle looks like a conventional vehicle on the road. 

Our Journey:
We started of with two SPIKE hubs , as that was what we had left and had expierence in, we shortly came to realise it is very hard to make 2 spike hubs communicate - after leaving 2 hubs we realised that we could flash both hubs to robot inventor allowing us to make the 2 comminicate. However, we choice to not do that as there was a delay (which was obvious) but it was too long - nearly a second. That could be very dangerous for our robot later on. So we moved onto only 1 hub as shown in mark 3 and 5 we tried to make a nice aesthetic - but came to realise that the robot body was too heavy.Or in mark 3 is case it was not able to fit all the motors while looking aesthetically pleasing. So we had to remove all the parts that were not required. Which lead, to an uglier robot that worked (mark 4 or mark 6). Then we came to the issue that the lego parts were not strong enough to have accurate turns - this was also partially due to the fact that the gyro sensor was not very accurate. So we moved on to a pre-built vehicle we brought many different types , such as a Land Rover, Mercedes and Ford RC vehicles which were all too small. We then came to the RC truck , found in Mark 9 , which came with a load bed which allowed quick access to the raspberry pi and other essential equipment. With 2 fast motors and a working Ackermann system this truck was a perfect base for us to use. After sorting the base, we had to make the raspberry pi communicate with the body. So we removed soldering connecting the remote control and instead soldered wires which connected to the raspberry pi - via a motor controller. Now that we had our body communicating to our robot, we had to now put it all together to complete the 3 laps required. So, to put our sensors in we made holes in the cabin and truck load to allow the gyro (placed on top of the cabin) and the Lidar(placed underneath the grill) cables flow through making a cleaner design and less strain on the wire. Finally, we had to place the camera which we decided to put in the windshield but came to realise that it is not too optimal for our robot. So we put it on the roof giving the camera extra range giving better results.
