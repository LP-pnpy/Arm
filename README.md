###
rm -rf install/my_robot_description && colcon build --packages-select my_robot_description

###
colcon build --packages-select my_robot_description my_robot_moveit_config
