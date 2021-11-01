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
roslaunch cola2_stonefish athena_simulation.launch
```

* Naiad:
```
roslaunch cola2_stonefish naiad_simulation.launch
```

* Trident:
```
roslaunch cola2_stonefish trident_simulation.launch
```

