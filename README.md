# ardrone_essentials
contains several packages for running the ardrone parrot 2

##Author
Patrick Hamod

##Introduction
This package contains cam_test which is the only package I have created seperate-
ly.The other packages have modified to make use of this new package. The new
package ensures that each call to the rosservice to change the camera of the
drone is processed with minimal problems from the delay of the rosservice call.

all packages included use the ros framework and more information can be found 
[here][ros]

##Dependencies
This package was tested in ubuntu 14.04 and ros indigo in order to install the
packages.

next insert the packages into your working ros src directory then use

```
rosdep install ardrone_autonomy
rosdep install ardrone_tutorials
rosdep install tum_ardrone
rosdep install cam_test
```

now all dependencies should be downloaded. Use catkin_make to build the packages

##Use
All packages can be run using the following code:
```
roslaunch cam_test ardrone.launch
```
if you want to run individual packages refer to the packackage below

###ardrone_autonomy
---
This package is required to run the ardrone parrot 2 in ros. to run package aft-
er install, use one of the launch files included in ardrone_autonomy:

```
roslaunch ardrone_autonomy ardrone.launch
```
or 
```
roslaunch ardrone_autonomy ardrone_aggressive.launch
```

###ardrone_tutorials
---
This package needs ardrone_autonomy to run so any launch files in this package
also creates a node for ardrone_autonomy for joypad control use:
```
roslaunch ardrone_tutorials ardrone.launch
```
####joypad control
This set up assumes the use of a logitech f310 joypad. included in this github
there is a list of set ups and button layouts. The current layout uses the foll-
owing:

using x-input mode light off

a=takeoff
x=land
lb=emergency

left stick y-axis = drone roll
left stick x-axis = drone pitch
control pad x-axis=drone yaw
control pad y-axis=elevation

####keyboard control
to use keyboard control run the following:
```
roslaunch ardrone_tutorials keyboard_controller.launch
```
or
```
roslaunch ardrone_tutorials keyboard_controller_outdoor.launch
```
for controls go to [robohub][Robohub]

###tum_ardrone
This package uses PTAMs throught the ardrone and allows for localization and 
mapping this package has been modified to only use the top camera so switching
cameras does not mess up the state estimation. for more information look [here]
[tum_ardrone]

This package requires ardrone_autonomy to run for this package to do anything
useful. run this package with:
```
roslaunch tum_ardrone tum_ardrone.launch
```

###cam_test
This code runs and every 8 frames the command to change cameras is sent. This 
code like the rest requires ardrone_autonomy to run. This code only works on 
ardrone parrot 2

In order to run this code use:
```
rosrun cam_test cam_test
```
once the code is running publish to the /ardrone/cameraswap topic start switch-
ing camera:
```
rostopic pub /ardrone/cameraswap std_msgs/String "start"
```

topics published include:
ardrone/front_cam
ardrone_bottom_cam
---
these are modified publishers of the ardrone_autonomy topics that publish with
corrections to the large amounts of service alls to change cameras

ardrone/cameraInUse
---
this publishes a boolean value true if bottom camera and false if front camera

topics subscribed to:
ardrone/cameraswap
---
this was meant for if more commands are needed but for my purposes a start suff-
iced

ardrone/image_raw
---
to check if you are recieving images from the drone use:
```
rosrun image_view image_view image:=/ardrone/image_raw
```

## last remarks
If you configure the drone to connect to a network the drone.run script can be
executed for quicker start up and remembering the command to connect the drone
to the network

info for connecting the drone to a network can be found [here][multidrone]

If you have questions not covered in the readme my email is pfhamod@gmail.com


## License
---
Released under the MIT Liscense 




[Robohub]: http://robohub.org/up-and-flying-with-the-ar-drone-and-ros-getting-started/
[multidrone]:https://github.com/AutonomyLab/ardrone_autonomy/wiki/Multiple-AR-Drones 
[afrl]: http://afrl.cse.sc.edu/afrl/home/
[tum_ardrone]: http://wiki.ros.org/tum_ardrone
[ros]: http://wiki.ros.org/
