
I wrote the forwards and inverse kinematics myself to learn more about linear algebra however I the way i did the inverse kinematics I dont' think is the 'proper' way to do it. 

realsense install:
https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

also need eigen and pigpio

Modules:
`kinematics`:
	does forwards and backwards kinematics
	takes: reletive goal pose
	returns: joint angles
	
`vision`:
	keep track of world position
	returns: world position

`arm`:
	takes joint angles and sends them to the arm, maybe do some smoothing/pathing
	takes: joint angles
	does: sends angles to arm with gpio

`main`:
	get pose from vision
	work out reletive pose of target (maybe do in vision)
	move target to be valid, collision free
	gets joint angles from inverse kinematics
	?plans path
	send path to arm
