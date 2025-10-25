import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    pkg_path= get_package_share_directory('arm_control')

    urdf_file= os.path.join(pkg_path, 'urdf', 'Arm_Urdf.urdf')

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'arm.rviz')


    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]

        ),

        Node(
            package='arm_control',
            executable='joint_teleop',
            name='joint_teleop',
            output='screen',
            prefix='xterm -e'
        ),
        

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )


    ])