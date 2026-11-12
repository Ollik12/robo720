import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Set model directory explicitly for Gazebo to find the models
    models_dir = PathJoinSubstitution([
        FindPackageShare("ex5"),
        "model"
    ])
    set_model_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[models_dir, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")]
    )

    # RViz config file with tf tree and robot model ready on startup
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ex5"), "config", "default_config_ex5.rviz"]
    )

    # Initialize controller arguments
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ex5"), "config", "franka_controllers.yaml"]
    )

    # Initial joint configuration
    initial_positions = PathJoinSubstitution(
        [FindPackageShare("ex5"), "config", "initial_positions.yaml"]
    )

    # World file
    world_file = os.path.join(
        get_package_share_directory("ex5"), "world", "cam_world.sdf"
    )

    # Find the robot description file
    robot_description_xacro = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("franka_description"),
                 "urdf",
                 "effort_fr3_arm.urdf.xacro"]
            ),
            " ",
            "end_effector:=gripper", # End effector set to be forcetorque sensor
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
            " ",
            "effort_commands:=true",
            " ",
            "camera:=true",
            " ",
            "camera_image_width:=1280",
            " ",
            "camera_image_height:=960",
            " ",
            "initial_positions_file:=",
            initial_positions,
        ]
    )

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True,
                    "robot_description": ParameterValue(robot_description_xacro, value_type=str)
        }],
        remappings=[("robot_description", "robot_description")]
    )

    gripper_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_position_controller",
            "--controller-manager", "/controller_manager",
            "--switch-timeout", "300.0",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--switch-timeout", "300.0",
        ],
    )

    visual_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "visual_controller",
            "--controller-manager", "/controller_manager",
            "--switch-timeout", "300.0",
        ],
    )

    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "fr3_arm",
            "-allow_renaming",
            "true",
        ],
    )

    # launch arguments for Gazebo {args, world (- v 1 = log level)}
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": f" -r -v 1 {world_file}"}.items(),
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config]
    )

    # Bridge Gazebo topics to ROS
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
        ],
    )

    obstacle_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="obstacle_pose_bridge",
        arguments=[
            "/world/cam_scene/pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
        ],
        output="screen",
    )

    scene_bridge = Node(
        package="ros_ign_scene_bridge",
        executable="scene_bridge",
        parameters=[{"world_name": "cam_scene"}],
        output="screen",
    )

     # Aruco marker pose estimation node
    pose_estimate = Node(
        package='ex5',
        executable='pose_estimate.py',  
        output='screen',
        arguments=[
            '--ros-args',
            '-p', 'image_topic:=/camera/image',
            '-p', 'camera_info_topic:=/camera/camera_info',
            '-p', 'dictionary:=DICT_4X4_50',
            '-p', 'marker_id:=0',
            '-p', 'marker_length_m:=0.04',
        ],
    )

    nodes_to_start = [
        set_model_path,
        robot_state_publisher_node,
        gz_launch_description,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[
                    joint_state_broadcaster_spawner,
                    gripper_position_controller_spawner,
                    visual_controller_spawner,
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node],
            )
        ),
        gz_spawn_entity,
        ros_gz_bridge,
        obstacle_pose_bridge,
        scene_bridge,
        pose_estimate,
        SetEnvironmentVariable('IGN_GAZEBO_VERBOSE', '1'),
    ]

    return LaunchDescription(nodes_to_start)