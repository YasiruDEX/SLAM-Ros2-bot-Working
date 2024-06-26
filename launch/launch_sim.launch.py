import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction



def generate_launch_description():

    package_name='SLAM_ros2_bot'

    delayed_controller_manager_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_joint_1_position_controller"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_joint_2_position_controller"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_joint_1_position_controller"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_joint_2_position_controller"],
            ),
        ],
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
                
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','my_parameters.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    
    depthimage_to_laserscan = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[('depth', '/kinect_depth/depth/image_raw'),
                        ('depth_camera_info', '/kinect_depth/camera_info')],
            parameters=[{'output_frame': 'laser_frame'}]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','my_parameters.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'SLAM_ros2_bot'],
                        output='screen')
    
    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_cont"],
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_broad"],
    # )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        joystick,
        twist_mux,
        spawn_entity,
        depthimage_to_laserscan,
        # diff_drive_spawner,
        # joint_broad_spawner,
        delayed_controller_manager_spawner
    ])
