# Universal Robots Trajectory controller
# NOTE SOMETHING ABOUT STARTING POSITIONS...
Package using the Universal Robots ROS2 driver to control a UR16e.

### Capabilities:
- Record waypoints by manually moving the robot using freedrive, and clicking 'j' or 'l' on your keyboard for move J or move L.
- Generates .yaml trajectory files.
- For each move a flag can be set to turn a laser scanner on or off.
- The trajectories can then be played back. 
- Currently uses the duration of moves, not velocity or acceleration. 

## Installation and dependencies
The main dependency for this package is the UR ROS2 driver. Additionally, there is a ROS2 keyboard driver, which is used when recording trajectories. 
- [Universal Robots ROS2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
- [keyboard & keyboard_msgs](https://github.com/Luka140/ros2-keyboard)
```
sudo apt-get install ros-humble-ur
git clone git@github.com:Luka140/ros2-keyboard.git
```

To make full use of the package, it is used alongside the following two packages to enable creating 3D scans. Nodes from these packages are launched from some of the launch files.
- [scancontrol](https://github.com/Luka140/scancontrol/tree/ros2-devel)
- [lls_processing](https://github.com/Luka140/lls_processing)

```
git clone git@github.com:Luka140/scancontrol.git
git clone git@github.com:Luka140/lls_processing.git
```



## Launch


## Nodes 
The current node architecture does not make much sense, to be refactored. 

### `ur_controller`

### `surface_scan_coordinator`

### `ur_trajectory_recorder`

### `ur_trigger_tester`
