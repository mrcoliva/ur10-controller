# UR10 Robot Controller in ROS

### An adaptive controller for simultaneous trajectory following, gaze control, and collision avoidance with moving obstacles.

The task of the robot has three components:
1. move to a starting position in joint space
2. follow a circular trajectory with the end-effector
3. always keep the end-effector oriented towards the currently active visual target on the floor
4. avoid collisions with the moving objects

To achive this behaviour, three control laws are applied:
* a joint space controller for the initial motion from the singular position into the start position
* a Cartesian space controller to follow the circular trajectory, to control the orientation towards the targets and to move along trajectories back to the circular path after being diverted due to nearby obstacles
* an impedance controller is used to avoid collisions with obstacles.

For a more detailed theoretical introduction and documentation of the system, see the report.

### How to run the controller
To run the controller, perform the following steps:
1. build the workspace: `catkin_make`
2. start the simulator: `roslaunch tum_ics_ur10_bringup bringUR10-simulator.launch`
3. in a new terminal, start the controller: `roslaunch tum_ics_ur10_controller_tutorial testSimpleEffortCtrl.launch`

This starts the controller and it begins by executing a joint space motion to move from the initial singular position into a start position for the task. 
Now, the robot begins to interpolate onto and then move along a circular path and it controls its orientation to look towards the current target. On the circular trajectory,
the current target position is visualized as a pose that moves along the path.

To test the behavior with moving targets, in a new terminal run `rosservice call /add_moving_targets`.
To test the obstacle avoidance, add the obstacles via `rosservice call /add_obstacles {1/2/3}`.

Many behaviors of the controller can be customized by configuring the parameters in `tum_ics_ur10_controller_tutorial/launch/configs/simpleEffortCtrl.yaml`.
The effect of the parameters is documented in place.

#### Trajectory following and gaze control
https://user-images.githubusercontent.com/11507299/120662500-7fe1cc00-c489-11eb-9984-186d0c105838.mp4

#### Simoultaneous collision avoidance with up to three mobing obstacles
https://user-images.githubusercontent.com/11507299/120663765-9c323880-c48a-11eb-81b2-aeb3bb9d16c8.mp4
