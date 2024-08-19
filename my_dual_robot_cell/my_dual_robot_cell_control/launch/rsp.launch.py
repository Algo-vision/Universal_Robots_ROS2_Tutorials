from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot2_ur_type = LaunchConfiguration("robot2_ur_type")
    robot1_ur_type = LaunchConfiguration("robot1_ur_type")

    robot2_robot_ip = LaunchConfiguration("robot2_robot_ip")
    robot1_robot_ip = LaunchConfiguration("robot1_robot_ip")

    robot2_use_mock_hardware = LaunchConfiguration("robot2_use_mock_hardware")
    robot2_mock_sensor_commands = LaunchConfiguration("robot2_mock_sensor_commands")
    robot1_use_mock_hardware = LaunchConfiguration("robot1_use_mock_hardware")
    robot1_mock_sensor_commands = LaunchConfiguration("robot1_mock_sensor_commands")

    headless_mode = LaunchConfiguration("headless_mode")

    robot2_kinematics_parameters_file = LaunchConfiguration(
        "robot2_kinematics_parameters_file"
    )
    robot1_kinematics_parameters_file = LaunchConfiguration(
        "robot1_kinematics_parameters_file"
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "urdf",
                    "my_dual_robot_cell_controlled.urdf.xacro",
                ]
            ),
            " ",
            "robot2_robot_ip:=",
            robot2_robot_ip,
            " ",
            "robot1_robot_ip:=",
            robot1_robot_ip,
            " ",
            "robot2_ur_type:=",
            robot2_ur_type,
            " ",
            "robot1_ur_type:=",
            robot1_ur_type,
            " ",
            "robot2_use_mock_hardware:=",
            robot2_use_mock_hardware,
            " ",
            "robot1_use_mock_hardware:=",
            robot1_use_mock_hardware,
            " ",
            "robot2_kinematics_parameters_file:=",
            robot2_kinematics_parameters_file,
            " ",
            "robot1_kinematics_parameters_file:=",
            robot1_kinematics_parameters_file,
            " ",
            "robot2_mock_sensor_commands:=",
            robot2_mock_sensor_commands,
            " ",
            "robot1_mock_sensor_commands:=",
            robot1_mock_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot2_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot2_robot_ip",
            default_value="192.168.56.102",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_robot_ip",
            default_value="192.168.56.101",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot2_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "robot2_calibration.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "robot1_calibration.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot2_use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot2_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
            
        ]
    )
