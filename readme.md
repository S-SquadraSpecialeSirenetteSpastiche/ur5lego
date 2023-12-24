# UR5 Lego Project

This project aims to control a UR5 robotic arm to move megablocks from one position to another. The robot can be simulated or real.
This project requires `locosim` to initialize the simulation and the real robot, so make sure you have it installed and
that it is set-up correctly, it is suggested to have it in the same workspace as this project, both under src/, however this is not required.


## Starting the project

First, start the simulation by calling the script ur5_generic.py:
`python locosim/robot_control/base_controllers/ur5_generic.py`
Then you can start the robot controller with
`roslaunch ur5lego ur5lego.launch`
NOTE: ur5lego.launch will be the final launch file, for now start inverse_kinematics.launch, as it is the only launchfile available


## Moving the robot

You can easily move the robot in the cartesian space by using the move_wrapper.bash script:
`./scripts/move_wrapper.bash <x> <y> <z> <roll> <pitch> <yaw> <time>`
where the arguments are the desired position and orientation of the end effector in the cartesian space and the time to move in seconds.
If you want to control it from code you wrote yourself, you can call the `move_server` action server with the same arguments as above.
This will come with less overhead and the possibility to wait until the robot has finished moving.


## Spawning blocks

To spawn blocks in the simulation, you can use the spawn_block.bash script:
`./scripts/spawn_block.bash block_name:=block1 block_type:=X1-Y1-Z2`
The name of the block can be any string as long as it is not the name of another object in the simulation, the block types are listed under blocks_description/

## Grasping

Grasping in the simulation is functional but may have some bugs due to limitations in Gazebo. To grasp a block, line up the gripper with the block and set the angles of the gripper to approximately -0.08, you can do so by calling the script:
`./scripts/move_wrapper.bash -0.08`
If you want to control it from code you wrote yourself you can call the `gripper_server` action server with the same arguments as above.
NOTE: the gripper_server is not yet implemented, however you can send a float over `/gripper_joint_position`


## Real robot

To use the real robot, you need to change two flags: one is the `REAL_ROBOT` flag to `true` in the `movement/position_publisher.cpp` file, and the other one is the `real_robot` flag under `locosim/robot_control/base_controllers/params.py` to `True`.
Then, follow the startup procedure for the real robot, which is described in the `locosim` readme.
Once the robot has moved to the homing position, you can control it the same way as the simulation
