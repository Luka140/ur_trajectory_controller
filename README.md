# Universal Robots Trajectory controller
Package using the Universal Robots ROS2 driver to control a UR16e.

### Capabilities:
- Record waypoints by manually moving the robot using freedrive, and clicking 'j' or 'l' on your keyboard for move J or move L. Note that there are some traces of code relating to different moves, like move p, but these were not implemented fully.
- Generates .yaml trajectory files.
- For each move, a flag can be set to turn a laser scanner on or off.
- The trajectories can then be played back. 
- Currently uses the duration of moves, not velocity or acceleration. 

## Installation and dependencies
The main dependency for this package is the UR ROS2 driver. Additionally, there is a ROS2 keyboard driver, which is used when recording trajectories. 
- [Universal Robots ROS2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
- [keyboard & keyboard_msgs](https://github.com/Luka140/ros2-keyboard)
```
git clone git@github.com:Luka140/ur_trajectory_controller.git
sudo apt-get install ros-humble-ur
git clone git@github.com:Luka140/ros2-keyboard.git
```

To make full use of the package, it is used alongside the following two packages to enable creating 3D scans. Nodes from these packages are launched from some of the launch files.
- [scancontrol](https://github.com/Luka140/scancontrol/tree/ros2-devel)
- [lls_processing](https://github.com/Luka140/lls_processing)
- Open3D

```bash 
git clone git@github.com:Luka140/scancontrol.git
git clone git@github.com:Luka140/lls_processing.git

pip install open3d==0.18.0 numpy==1.24.0
```



## Launch 
To create a 3D scan, launch the following:
```bash
ros2 launch ur_trajectory_controller ur_surface_measurement.launch.py
```

To record a trajectory, launch the following:
```bash 
ros2 launch ur_trajectory_controller ur_record_trajectory.launch.py
```
It can be useful to launch the laser scanner node during recording, to see where the scanner is pointing. For this, use:
```bash
ros2 launch ur_trajectory_controller ur_launch_scanner.launch.py
```


## Nodes 
The current node architecture works but is not the most logical but it came as a result of changing goals. The `ur_controller` takes a config .yaml file, which specifies the trajectory, processes it and stores it in a list. If `autonomous_execution` is set to true, it will compile a UR script for this trajectory and then send it to the `ur_script_interface` in the UR driver. Otherwise, it will send the moves one by one whenever prompted by the `~/trigger_move` topic. The `measurement_coordinator` (`surface_scan_coordinator.py`) node is an extension of the previous node. It also takes in the config .yaml. It loops through the moves and triggers them whenever the duration of the previous one has passed. Additionally, it checks the `laser_on` entry in the config for each move, and turns the laser scanner on or off based on this. At the end of the trajectory, it requests `/pcl_constructor/combine_pointclouds` to create a reconstruction of the full point cloud. It may make more sense to make the `ur_controller` receive messages with a move or set of moves, from the `measurement_coordinator`, rather than providing both of them with the full config file. 

In addition to these two nodes, there is the `ur_trajectory_recorder` node. This listens to key presses and will store the current joint state as a `movej` or `movel` if the "J" or "L" key is pressed. Then when the "Enter" key is pressed, a .yaml file will be created with this stored trajectory, which can be read by `measurement_coordinator` and `ur_controller`. Note that after recording, the durations of the moves should still be set, along with the laser state. In addition to this, there is a list of safe starting joint positions at the bottom. The joint states should be in this range of values (in radians) to be able to start the trajectory. 

The `ur_traj_trigger_tester` file contains a node that publishes some messages on key presses. It was used for debugging at some point but is largely important. [TO BE REMOVED]

### `ur_controller`
Note that there are some interfaces left in this node to use the scaled joint trajectory controller. This is not used, in favour of the ur script interface. The ur script interface was used to guarantee linear motions. 

Published topics:
- `/urscript_interface/script_command` (std_msgs/String): Sends UR script in string format to be executed by the UR16e.

Subscribed topics:
- `joint_states` (sensor_msgs/JointState); Subscribes to joint states to validate starting positions if the check_starting_point parameter is enabled.
- `~/trigger_move` (std_msgs/Empty): Triggers execution of the next move in the goals list.

Parameters:
- `autonomous_execution` (bool, default: False): Determines whether all moves should be executed autonomously or one at a time.
- The trajectory config file. This includes a collection of additional parameters which are automatically set. See [this example](https://github.com/Luka140/ur_trajectory_controller/blob/master/config/trajectories/trajectory_dual_robot_setup.yaml) of a config file. 
 
### `surface_scan_coordinator`
Published topics:
-`/ur_controller/trigger_move` (std_msgs/Empty): Triggers the next move in the robot's trajectory.
- `/laseron` (std_msgs/Empty): Turns the laser on during surface measurements.
- `/laseroff` (std_msgs/Empty): Turns the laser off during surface measurements.

Services:
- `/pcl_constructor/combine_pointclouds` (data_gathering_msgs/srv/RequestPCL): (CLIENT) Requests the combination and storage of all point clouds gathered during the trajectory execution.
- `/execute_loop` (data_gathering_msgs/srv/RequestPCL): (SERVER) Starts a loop of trajectory execution when the `loop_on_service parameter` is enabled. Returns the final combined point cloud after the loop.

Parameters:
- `auto_loop` (bool, default: False): If True, the trajectory loop executes automatically and infinitely.
- `loop_on_service` (bool, default: False): If True, allows external triggering of trajectory loops via the /execute_loop service.
- The trajectory config file. This includes a collection of additional parameters which are automatically set. See [this example](https://github.com/Luka140/ur_trajectory_controller/blob/master/config/trajectories/trajectory_dual_robot_setup.yaml) of a config file. 

### `ur_trajectory_recorder`
The `path_recorder` node allows the recording of robot joint states and the generation of trajectories that can be executed later. It listens for joint states and specific keyboard inputs to build and save trajectories.

Subscribed topics:
- `/joint_states` (sensor_msgs/JointState): Provides joint positions of the robot, which are used to calculate averaged positions.
- `/keydown` (keyboard_msgs/Key): Detects key presses to trigger recording, resetting, or saving of trajectory waypoints.

Parameters:
- `record_urscript` (bool, default: `True`):
    - If True, trajectories are formatted for ur_script.
    - If False, trajectories are formatted for the scaled_joint_trajectory_controller. (May not work as intended)
- `filepath` (str, default: `src/ur_trajectory_controller/config/trajectories`): The path where the trajectory is stored.
- `default_velocity` (dict, default = `{"movej": 0.0, "movel": 0.0, "movep": 0.0}`): The default velocity for each move type.
- `default_acceleration` (dict, default = `{"movej": 0.0, "movel": 0.0, "movep": 0.0}`): The default velocity for each move type.
- `default_movement_duration`  (float, default = `10.` seconds): The default duration of each move. With this set, the velocity and acceleration are not used.
- `starting_limits` (dict, default = `{
            'shoulder_pan_joint': [-0.2, 0.2],
            'shoulder_lift_joint': [-1.7, -1.4],
            'elbow_joint': [-1.7, -1.4],
            'wrist_1_joint': [-1.7, -1.4],
            'wrist_2_joint': [1.4, 1.7],
            'wrist_3_joint': [-0.2, 0.2]}`): The range of allowed joint positions to start the trajectory.

The following are the keybindings:
|       Key      |              Action             |
|:--------------:|:-------------------------------:|
| Key.KEY_P      | Record a movep waypoint.        |
| Key.KEY_L      | Record a movel waypoint.        |
| Key.KEY_J      | Record a movej waypoint.        |
| Key.KEY_S      | Record a scaled trajectory.     |
| Key.KEY_DELETE | Reset the trajectory.           |
| Key.KEY_RETURN | Save trajectory to a YAML file. |

### `ur_trigger_tester` [TO BE REMOVED]
