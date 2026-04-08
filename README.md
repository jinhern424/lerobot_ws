## Command to run
This will allow us to control the SO101_arm through Rviz2 with MoveIt 2

Terminal 1
```bash
ros2 launch lerobot_description so101_gazebo.launch.py use_sim:=false
```
Terminal 2
```bash
ros2 launch lerobot_controller so101_controller.launch.py is_sim:=false
```
Terminal 3
```bash
ros2 launch lerobot_moveit so101_moveit.launch.py is_sim:=false
```
Launch the Intel RealSense
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true
```
Aruco Marker Calibration
```bash
ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/camera/color/image_raw \
    -r camera_info:=/camera/color/camera_info \
    -p family:=36h11 \
    -p size:=0.15 \
    -p approx_sync:=true
```
