# UR10 Robot Controller in ROS

### An adaptive controller for simultaneous trajectory following, gaze control, and collision avoidance with moving obstacles.

The task of the robot has three components:
1. move to a starting position in joint space
2. follow a circular trajectory with the end-effector
3. always keep the end-effector oriented towards the currently active visual target on the floor
4. avoid collisions with the moving objects

To achive this behaviuor, three control laws are applied:
* a joint space controller for the initial motion from the singular position into the start position
* a Cartesian space controller to follow the circular trajectory, to control the orientation towards the targets and to move along trajectories back to the circular path after being diverted due to nearby obstacles
* an impedance controller is used to avoid collisions with obstacles.

For a more detailed theoretical introduction and documentation of the system, see the report.

#### Trajectory following and gaze control
https://user-images.githubusercontent.com/11507299/120662500-7fe1cc00-c489-11eb-9984-186d0c105838.mp4

#### Simoultaneous collision avoidance with up to three mobing obstacles
Uploading collision_avoidance.mp4…
