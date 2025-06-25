import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = os.path.join(get_package_share_directory("swerve_drive_description"))
    xacro_file = os.path.join(pkg_path, "urdf", "swerve_drive.xacro")
    robot_description_config = xacro.process_file(xacro_file)

    robot_description_xml = robot_description_config.toxml()
    source_code_path = os.path.abspath(
        os.path.join(pkg_path, "../../../../src/swerve_drive_description")
    )
    urdf_save_path = os.path.join(source_code_path, "swerve.urdf")
    with open(urdf_save_path, "w") as f:
        f.write(robot_description_xml)

    # Create a robot_state_publisher node
    params = {"robot_description": robot_description_xml, "use_sim_time": use_sim_time}
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_xml, "use_sim_time": use_sim_time},
            controllers_file,
        ],
        output="screen",
    )

    # Create the publisher gui
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    # Start Rviz2 with basic view
    rviz2_config_path = os.path.join(
        get_package_share_directory("swerve_drive_description"), "config/view.rviz"
    )
    run_rviz2 = ExecuteProcess(cmd=["rviz2", "-d", rviz2_config_path], output="screen")

    # Launch!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use sim time if true"
            ),
            node_robot_state_publisher,
            joint_state_publisher_gui,
            run_rviz2,
        ]
    )
