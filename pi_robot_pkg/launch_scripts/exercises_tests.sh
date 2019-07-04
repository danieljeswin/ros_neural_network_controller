# Spawn the piRobot
roslaunch pi_robot_pkg spawn_robot.launch

# Launch the controllers and robot state publisher
roslaunch pi_robot_pkg pi_robot_control.launch



# 1) Launch without any joint controler, to see that only fixed links are publushed in tf
# 2) launch with some controled joints, and see it only calculates those tf that link that
# 3) launch one that controles everything


# Commands to move Robot
rostopic pub /pi_robot/head_tilt_joint_position_controller/command std_msgs/Float64 "data: 0.0"                
rostopic pub /pi_robot/head_pan_joint_position_controller/command std_msgs/Float64 "data: 0.7"
