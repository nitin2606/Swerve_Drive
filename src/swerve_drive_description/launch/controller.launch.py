from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory("swerve_drive_description"))
    xacro_file = os.path.join(pkg_path, "urdf", "swerve_drive.xacro")
    controllers_file = os.path.join(pkg_path, "config", "swerve_controller.yaml")

    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    source_code_path = os.path.abspath(
        os.path.join(pkg_path, "../../../../src/swerve_drive_description")
    )
    urdf_save_path = os.path.join(source_code_path, "swerve.urdf")
    with open(urdf_save_path, "w") as f:
        f.write(robot_description_xml)

    params = {"robot_description": robot_description_xml, "use_sim_time": False}

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                # arguments=["swerve_drive_controller"],
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                # arguments=["diff_drive_controller"],
                # output="screen"
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description_xml, "use_sim_time": False},
                    controllers_file,
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "swerve_drive_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
            ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=joint_state_broadcaster_spawner,
            #         on_exit=[swerve_drive_controller_spawner],
            #     )
            # )
        ]
    )
