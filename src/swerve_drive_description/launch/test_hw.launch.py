from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from os.path import join

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # position_x = LaunchConfiguration("position_x")
    # position_y = LaunchConfiguration("position_y")
    # orientation_yaw = LaunchConfiguration("orientation_yaw")

    swerve_drive_base_dir = get_package_share_directory('swerve_drive_description')
    swerve_drive_base_launch_file = os.path.join(swerve_drive_base_dir, 'launch', 'swerve_control.launch.py')

    swerve_drive_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(swerve_drive_base_launch_file),
        # launch_arguments={'arg_name': 'arg_value'}.items()  # Optional arguments to pass to the included launch file
    )


    def robot_state_publisher(context):
        performed_description_format = LaunchConfiguration('description_format').perform(context)
        # Get URDF or SDF via xacro
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('swerve_drive_description'),
                    performed_description_format,
                    f'swerve_drive.xacro.{performed_description_format}'
                ]),
            ]
        )
        robot_description = {'robot_description': robot_description_content}

        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command( \
                    ['xacro ', join(swerve_drive_base_dir, 'urdf/swerve_drive.xacro')])}]
        )
        return [node_robot_state_publisher]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('swerve_drive_description'),
            'config',
            'swerve_controller.yaml',
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            "-topic", "/robot_description",
            "-name", "swerve_drive",
            "-allow_renaming", "true",
            # "-z", "0.28",
            # "-x", position_x,
            # "-y", position_y,
            # "-Y", orientation_yaw
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_drive_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/world/default/model/the_bot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ],
        output='screen'
    )

    ld = LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),
        bridge,
        gz_spawn_entity,
        swerve_drive_base_launch,
        
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
