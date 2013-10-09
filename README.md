REEM-C Gazebo simulation preview

Requisites:
- Gazebo standalone 1.8.
- A complete ROS Fuerte desktop installation. Later ROS distributions are not supported by this preview.
- An installation of the osrf-common package.

Build the source:
- From the current directory, source the setup file:

  source reemc_setup.sh

- Build all packages. If some requisites fail, please install the missing dependencies. The rosdep command can
  help here.

  rosmake -a

Launch Gazebo:

- Open three terminals in the current folder, and source the setup file as described above.

- Terminal 1: Should load REEM-C standing in an empy world.

  roslaunch reemc_gazebo reemc_empty_world.launch

- Terminal 2: Loads joint trajectory controllers for all joints.

  roslaunch reemc_controller_configuration joint_trajectory_controllers.launch

- Terminal 3: Move to a few stored poses.

  rosparam load `rospack find reemc_tutorials`/config/reemc_poses.yaml
  rosrun reemc_tutorials reach_pose small_squat
  rosrun reemc_tutorials reach_pose squat
  rosrun reemc_tutorials reach_pose home