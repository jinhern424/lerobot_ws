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
