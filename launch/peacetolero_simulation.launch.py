import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare 


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="peacetolero",  # Edit for other robots
        description="Name of the robot",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("peacetolero_stonefish"), "config", "view.rviz"]
    )

    # Diccionario con el URDF expandido
    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("peacetolero_description"), "urdf", "peacetolero_alpha.config.xacro"]
                ),
                " ",
                "prefix:=", "",
                " ",
                "namespace:=", "",
            ]
        )
    }

    include_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("peacetolero_stonefish"),
                "launch",
                "description_sim.launch.py"
            ])
        ),
        launch_arguments={
            "namespace": "peacetolero"
        }.items()
    )

    odom_tf_node = Node(
        package="peacetolero_stonefish",
        executable="odom_to_tf.py",
        name="odom_tf",
        remappings=[
            ("/odom_topic", "/peacetolero/dynamics/odometry_truth"),
        ],
        parameters=[{"fixed_frame": "world_ned", "base_link": "peacetolero/base_link"}],
        output="screen",
    )

    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world2ned",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "3.1415",
            "--frame-id",
            "world",
            "--child-frame-id",
            "world_ned",
        ],
        output="screen",
    )

    ros2_stonefish_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("stonefish_ros2"),
                        "launch",
                        "stonefish_simulator.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "simulation_data": PathJoinSubstitution(
                [FindPackageShare("peacetolero_stonefish"), "resources"]
            ),
            "scenario_desc": PathJoinSubstitution(
                [FindPackageShare("peacetolero_stonefish"), "scenarios", "cirtesu_tank.scn"]
            ),  # Edit for other robots
            "simulation_rate": "300.0",
            "window_res_x": "1440",
            "window_res_y": "900",
            "rendering_quality": "low",
        }.items(),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        namespace="peacetolero",
        parameters=[
           robot_description,
            PathJoinSubstitution(
                [
                    FindPackageShare("peacetolero_description"),
                    "config",
                    "peacetolero_alpha_controllers.yaml",
                ]
            ),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ["peacetolero", "/controller_manager"],
        ],
    )

    feedback_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "feedback_joint_position_trajectory_controller",
            "--controller-manager",
            ["peacetolero", "/controller_manager"],
        ],
    )

    cmdvel_to_joints_node = Node(
        package="peacetolero_stonefish",     
        executable="cmd_vel_map.py",  
        name="cmdvel_to_jointstate",
        output="screen",
    )

    joint_state_filter_node = Node(
        package="peacetolero_stonefish",     
        executable="joint_states_filter.py", 
        name="joint_states_filter",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            robot_name_arg,
            odom_tf_node,
            static_transform_publisher_node,
            ros2_stonefish_node,
            cmdvel_to_joints_node,
            joint_state_filter_node,
            include_description,
            control_node,
            joint_state_broadcaster_spawner,
            feedback_joint_trajectory_controller_spawner,
            rviz_node, 
        ]
    )
