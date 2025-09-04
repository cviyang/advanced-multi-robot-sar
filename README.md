# Single

roslaunch multi_bot_gazebo single_robot_gazebo.launch 

conda activate mypytorch && roslaunch yolov8_ros yolo_v8_single.launch

roslaunch mbot_navigation sim_bringup_slam.launch

roslaunch semantic_slam start.launch

# Multi

roslaunch multi_bot_gazebo multi_robot_gazebo.launch 

roslaunch multi_bot_navigation multi_slam.launch

roslaunch multi_bot_navigation map_merge.launch

conda activate mypytorch && roslaunch yolov8_ros yolo_v8.launch

conda activate mypytorch && roslaunch yolov8_ros yolo_v8_2.launch

conda activate mypytorch && roslaunch yolov8_ros yolo_v8_3.launch

killall gzserver
