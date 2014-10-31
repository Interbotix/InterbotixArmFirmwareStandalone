```
  }--\     InterbotiX     /--{
      |    Arm Fimrware  |
   __/                    \__
  |__|                    |__|
```


   This firmware allows you to control the InterbotiX Robot Arms in a standalone fashion - no PC or seperate micro controller device is required.
   
   
   Arm Link will send serial packets to the ArbotiX Robocontroller that
   specify coordinates for that arm to move to. TheArbotiX robocontroller
   will then do the Inverse Kinematic calculations and send commands to the
   DYNAMIXEL servos to move in such a way that the end effector ends up at
   the specified coordinate.
 
   Robot Arm Compatibilty: 
	   PhantomX Pincher Robot Arm
		 http://learn.trossenrobotics.com/interbotix/robot-arms/pincher-arm
	   PhantomX Reactor Robot Arm
		 http://learn.trossenrobotics.com/interbotix/robot-arms/reactor-arm
	   WidowX Robot Arm
		 http://learn.trossenrobotics.com/interbotix/robot-arms/widowx-arm
 

InterbotiXArmAnalog
	Allows you to control the arm via analog inputs plugged directly into the Robot Arm. You will need 3 joysticks, 1 Rotational Knob, and 2 pushbuttons.

InterbotiXArmPlayback
	Designed for use in conjunction with the Arm Link Software or firmware, allows you to play back pre-set poses.




-----------------------------------------------
See these links for information on controlling the arm via another device or PC see these links
http://learn.trossenrobotics.com/36-demo-code/137-interbotix-arm-link-software.html
http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html