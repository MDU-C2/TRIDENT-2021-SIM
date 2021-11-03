# TRIDENT-2021-SIM

This is the simulator used in the Trident AUV/USV project. The simulator used is [Stonefish](https://github.com/patrykcieslak/stonefish) which is ROS interfaced by the ROS package [Stonefish\_ros](https://github.com/smarc-project/stonefish_ros), originally developed by [Patryk Cieślak](https://github.com/patrykcieslak). Also, the package [cola2\_msgs](https://bitbucket.org/iquarobotics/cola2_msgs/src/master/) is used for convenient ROS messages. 

## Requirements

The simulation is CPU heavy and requires a recent GPU. The minimum requirement is the support for OpenGL 4.3. The software is developed and tested on Linux Ubuntu. It should work on any Unix based platform.

* Any recent ROS1 distro should work (neotic recommended).
* OpenGL Mathematics library (libglm-dev, version >= 0.9.9.0)
* SDL2 library (libsdl2-dev, may need the fix described [here](https://github.com/patrykcieslak/stonefish#installation))
* Freetype library (libfreetype6-dev)

## Installation

* Clone this repo into your ros workspace src directory:
```
cd ~/catkin_ws/src && git clone https://github.com/ProjectMDH/TRIDENT-2021-SIM
```

* Update the git submodules:
```
cd ~/catkin_ws/src/TRIDENT-2021-SIM && git submodule update --init --recursive
```

* Build the workspace:
```
cd ~/catkin_ws && catkin_make
```

## Running

Make sure to source the workspace:
```
source ~/catkin_ws/devel/setup.bash
```

* Athena:
```
roslaunch trident_sim athena_simulation.launch
```

* Naiad:
```
roslaunch trident_sim naiad_simulation.launch
```

* Trident:
```
roslaunch trident_sim trident_simulation.launch
```

**NOTE**: if you dont want to spawn the camera visualization node append `camera_show:=false` at the end of the command.

## Thrusters

The thrusters are controlled through the thruster\_setpoints topic for each vehicle separately.
The message type is cola2\_msgs/Setpoints which consists of:
```
Header header
float64[] setpoints
```
where each setpoint is a value between -1 and 1 and represents the amount of thrust to send to the respective thruster.
The order of the setpoints is visualized below:

Athena:

```
[Right] [Left]
```

Naiad:

```
[Right] [Left] [Front Right] [Front Left] [Back Right] [Back Left]
```

## Sensors

The sensors are accessed through their respective topics. The available sensors are listed below:

Athena:

* `/athena/gps`
* `/athena/imu`
* `/ahtena/odometry` (actual position)

Naiad:

* `/naiad/gps`
* `/naiad/imu`
* `/naiad/pressure`
* `/naiad/usbl`
* `/naiad/camera/image\_color`
* `/naiad/odometry` (actual position)

**NOTE**: The gps does not work under water. The usbl gives the position of Athena relative to itself.
