import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="peacetolero",  # Edit for other robots
        description="Name of the robot",
    )

    # robot_description_arg = DeclareLaunchArgument(
    #     "robot_description",
    #     default_value=[LaunchConfiguration("robot_name"), "_description"],
    #     description="Description of the robot",
    # )

    # robot_xacro_arg = DeclareLaunchArgument(
    #     "robot_xacro",
    #     default_value=PathJoinSubstitution(
    #         [
    #             FindPackageShare(LaunchConfiguration("robot_description")),
    #             "urdf",
    #             "payload.urdf.xacro",
    #         ]
    #     ),
    #     description="Path to the robot Xacro file",
    # )

    # robot_description = Command(
    #     [
    #         "xacro ",
    #         " ",
    #         LaunchConfiguration("robot_xacro"),
    #         " robot_namespace:=",
    #         LaunchConfiguration("robot_name"),
    #     ]
    # )

    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="screen",
    #     parameters=[{"robot_description": robot_description}],
    #     namespace=LaunchConfiguration("robot_name"),
    # )

    # odom_tf_node = Node(
    #     package="peacetolero_stonefish",
    #     executable="odom_to_tf.py",
    #     name="odom_tf",
    #     remappings=[
    #         ("/odom_topic", "/peacetolero/dynamics/odometry_truth"),
    #     ],
    #     parameters=[{"fixed_frame": "world_ned", "base_link": "peacetolero/base_link"}],
    #     output="screen",
    # )

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
                [FindPackageShare("peacetolero_stonefish"), "scenarios", "sea.scn"]
            ),  # Edit for other robots
            "simulation_rate": "300.0",
            "window_res_x": "1440",
            "window_res_y": "900",
            "rendering_quality": "low",
        }.items(),
    )

    return LaunchDescription(
        [
            robot_name_arg,
            # robot_description_arg,
            # robot_xacro_arg,
            # robot_state_publisher,
            static_transform_publisher_node,
            ros2_stonefish_node,
        ]
    )
