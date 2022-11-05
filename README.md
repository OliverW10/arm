# Arm

a small robot arm.

### v2
Rewritten in c++ using a intel realsense T265 camera connected to a raspberry pi to get high rate pose estimates and uses apriltags to set the goal position reletive to that pose. Controls the arm directly from a raspberry pi's gpio. Ideally can be entirely self contained with a raspberry pi, realsense camera, arm and battery all in a handheld box.


### v1
Initial prototype written in python, had apriltag markers around the base of the arm and on a target cube which were seen by a usb webcam connected to a computer which would then compute the joint positions and send it to an arduino uno over serial which would actuate the arm. The biggest issue was that relying on the camera seeing both the arm apriltag and the target apriltag in the same frame, the arm would often block view of the tags around its base when it was moving around its base.
