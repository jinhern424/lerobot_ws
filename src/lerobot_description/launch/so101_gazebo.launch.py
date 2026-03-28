import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    lerobot_description = get_package_share_directory("lerobot_description")

    # 1. Declare Launch Arguments
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(lerobot_description, "urdf", "so101.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    use_sim_arg = DeclareLaunchArgument(
        name="use_sim",
        default_value="true",
        description="Launch simulation (true) or physical hardware (false)"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(lerobot_description).parent.resolve())]
    )
    
    # 2. Process URDF with the 'use_sim' flag passed into Xacro
    robot_description = ParameterValue(
        Command([
            "xacro ", LaunchConfiguration("model"),
            " use_sim:=", LaunchConfiguration("use_sim") # Passes the flag to URDF!
        ]),
        value_type=str
    )

    # 3. Core Nodes (Always run)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": LaunchConfiguration("use_sim") # True for Gazebo, False for Real World
        }]
    )

    # ==========================================
    # 4. SIMULATION SPECIFIC NODES
    # (Only run if use_sim is true)
    # ==========================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments=[("gz_args", [" -v 4 -r empty.sdf "])],
        condition=IfCondition(LaunchConfiguration("use_sim"))
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "so101"],
        condition=IfCondition(LaunchConfiguration("use_sim"))
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        condition=IfCondition(LaunchConfiguration("use_sim"))
    )

    # ==========================================
    # 5. PHYSICAL HARDWARE SPECIFIC NODES
    # (Only run if use_sim is false)
    # ==========================================
    # Note: You will need a ros2_controllers.yaml file configured for your arm
    controllers_file = os.path.join(lerobot_description, "config", "ros2_controllers.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_file],
        output="both",
        condition=UnlessCondition(LaunchConfiguration("use_sim"))
    )

    return LaunchDescription([
        model_arg,
        use_sim_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        #ros2_control_node
    ])