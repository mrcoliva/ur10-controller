# UR10 Robot Controller in ROS

##### An adaptive controller for simoultaneous trajectory following, gaze control, and collision avoidance with moving obstacles.

Three control laws are applied: a joint space controller for the initial motion from the singular position into the start position, a Cartesian space controller to follow the circular trajectory, to control the orientation towards the targets and to move along trajectories back to the circular path after being diverted due to nearby obstacles. An impedance controller is used to avoid collisions with obstacles.

For a theoretical introduction and documentation of the system, see the report.

#### Trajectory following and gaze control
https://user-images.githubusercontent.com/11507299/120660458-a4d53f80-c487-11eb-970d-4877951838d9.mp4

#### Simoultaneous collision avoidance with up to three mobing obstacles
https://user-images.githubusercontent.com/11507299/120660688-de0daf80-c487-11eb-9f90-0c6a716abd1f.mp4

