# TODO: stop advertising /ur5/z_base_camera/camera/set_parameters and /ur5/ee_camera/camera/set_parameters

rosservice call /gazebo/delete_model ur5
rosnode kill /ur5/controller_spawner
roslaunch ur5lego spawn_robot.launch &