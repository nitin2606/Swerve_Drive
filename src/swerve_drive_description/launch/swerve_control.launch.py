import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory("swerve_drive_description"))
    xacro_file = os.path.join(pkg_path, "urdf", "base.xacro")
    controllers_file = os.path.join(pkg_path, "config", "swerve_controller.yaml")

    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    source_code_path = os.path.abspath(
        os.path.join(pkg_path, "../../../../src/swerve_drive_description")
    )
    urdf_save_path = os.path.join(source_code_path, "swerve.urdf")
    with open(urdf_save_path, "w") as f:
        f.write(robot_description_xml)

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("swerve_drive_description"), "urdf", "base.xacro"]
            ),
            " ",
        ]
    )

    params = {"robot_description": robot_description_xml, "use_sim_time": False}
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("swerve_drive_description"), "config", "robot.rviz"]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("swerve_drive_description"),
            "config",
            "swerve_controller.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        # arguments=["--ros-args", "--log-level", "debug"],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/swerve_drive_controller/cmd_vel", "/cmd_vel"),
            ("/swerve_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/swerve_drive_controller/odom", "/odom"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # prefix=["gdbserver localhost:3000"],
        arguments=["swerve_drive_controller", "--controller-manager", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    delay_control_node_after_robot_state_pub = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_pub_node,
            on_exit=[control_node],
        )
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        robot_state_pub_node,
        delay_control_node_after_robot_state_pub,
        control_node,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
