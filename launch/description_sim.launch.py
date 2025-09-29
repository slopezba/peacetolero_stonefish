import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    use_tank = LaunchConfiguration('use_tank')
    # Definir las descripciones de los robots
    description_file_cirtesu = os.path.join(
        get_package_share_directory('bluerov2_cirtesu_core'), 
        'urdf', 
        'cirtesu', 
        'cirtesu.urdf.xacro'
    )
    robot_description_cirtesu = os.popen(f'xacro {description_file_cirtesu}').read().strip()
    
    description_file_bluerov2 = os.path.join(
        get_package_share_directory('peacetolero_description'), 
        'urdf', 
        'peacetolero_alpha.config.xacro'
    )
    robot_description_peacetolero = os.popen(f'xacro {description_file_bluerov2}').read().strip()

    # Nodo robot_state_publisher para el bluerov
    robot_state_publisher_node_peacetolero = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        remappings=[('/robot_description', '/peacetolero/robot_description'),
                    ('/joint_states', '/peacetolero/joint_states'),],
        parameters=[
            {'robot_description': robot_description_peacetolero}
        ]
    )

    # Nodo robot_state_publisher para el tanque
    robot_state_publisher_node_cirtesu = Node(
        condition=IfCondition(use_tank),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher_cirtesu',
        remappings=[('/robot_description', '/cirtesu/robot_description')],
        parameters=[
            {'robot_description': robot_description_cirtesu}
        ]
    )
    
    cirtesu_static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "3.1416",
                "--yaw", "0",
                "--frame-id", "world_ned",
                "--child-frame-id", "cirtesu_tank"
            ]
        )

    return LaunchDescription([
        # Declarar el argumento 'use_depth' con valor por defecto 'true'
        DeclareLaunchArgument(
            'use_tank',
            default_value='true',
            description='Flag to indicate whether to publish the cirtesu tank or not'
        ),
        robot_state_publisher_node_peacetolero,
        robot_state_publisher_node_cirtesu,
        cirtesu_static_tf,
        
    ])

