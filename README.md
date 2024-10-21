# Universal Robots Trajectory controller
Package using the Universal Robots ROS2 driver to control a UR16.

The current node architecture does not make much sense, to be refactored. 

### Capabilities:
- Record waypoints by manually moving the robot using freedrive, and clicking 'j' or 'l' for move J or move L.
- Generates .yaml trajectory files.
- For each move a flag can be set to turn a laserscanner on or off.
- The trajectories can then be played back. 
- Currently uses the duration of moves, not velocity or acceleration. 
