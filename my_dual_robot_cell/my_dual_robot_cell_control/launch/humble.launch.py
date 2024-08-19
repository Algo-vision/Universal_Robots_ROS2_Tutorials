from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import launch_ros.descriptions

import yaml
import os
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None
def launch_setup(context,*args,**kwargs):
    robot2_ur_type = LaunchConfiguration("robot2_ur_type")
    robot1_ur_type = LaunchConfiguration("robot1_ur_type")

    robot2_robot_ip = LaunchConfiguration("robot2_robot_ip")
    robot1_robot_ip = LaunchConfiguration("robot1_robot_ip")

    robot2_use_mock_hardware = LaunchConfiguration("robot2_use_mock_hardware")
    robot2_mock_sensor_commands = LaunchConfiguration("robot2_mock_sensor_commands")
    robot1_use_mock_hardware = LaunchConfiguration("robot1_use_mock_hardware")
    robot1_mock_sensor_commands = LaunchConfiguration("robot1_mock_sensor_commands")
     # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    description_launchfile = LaunchConfiguration("description_launchfile")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    headless_mode = LaunchConfiguration("headless_mode")

    # Robot specific arguments
    robot2_use_mock_hardware = LaunchConfiguration("robot2_use_mock_hardware")
    robot2_mock_sensor_commands = LaunchConfiguration("robot2_mock_sensor_commands")
    robot2_initial_joint_controller = LaunchConfiguration("robot2_initial_joint_controller")
    robot2_activate_joint_controller = LaunchConfiguration("robot2_activate_joint_controller")
    robot2_launch_dashboard_client = LaunchConfiguration("robot2_launch_dashboard_client")

    robot1_use_mock_hardware = LaunchConfiguration("robot1_use_mock_hardware")
    robot1_mock_sensor_commands = LaunchConfiguration("robot1_mock_sensor_commands")
    robot1_initial_joint_controller = LaunchConfiguration("robot1_initial_joint_controller")
    robot1_activate_joint_controller = LaunchConfiguration("robot1_activate_joint_controller")
    robot1_launch_dashboard_client = LaunchConfiguration("robot1_launch_dashboard_client")

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


    # Single controller manager comprising of controllers for both arms
    control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file, allow_substs=True),
        ],
        output="screen",
    )


    robot2_dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(robot2_launch_dashboard_client) and UnlessCondition(robot2_use_mock_hardware),
        executable="dashboard_client",
        name="dashboard_client",
        namespace="robot2",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot2_robot_ip}],
    )

    robot1_dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(robot1_launch_dashboard_client) and UnlessCondition(robot1_use_mock_hardware),
        executable="dashboard_client",
        name="dashboard_client",
        namespace="robot1",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot1_robot_ip}],
    )
    robot2_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        namespace="robot2",
        parameters=[{"robot_ip": robot2_robot_ip}],
        output="screen",
    )

    robot1_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        namespace="robot1",
        parameters=[{"robot_ip": robot1_robot_ip}],
        output="screen",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )
    controllers_active = [
        "robot2_joint_state_broadcaster",
        "robot1_joint_state_broadcaster",
        # "robot2_io_and_status_controller",
        # "robot1_io_and_status_controller",
        # "robot2_speed_scaling_state_broadcaster",
        # "robot1_speed_scaling_state_broadcaster",
        # "robot2_force_torque_sensor_broadcaster",
        # "robot1_force_torque_sensor_broadcaster",
    ]
    controllers_inactive = [
        "robot2_forward_position_controller",
        "robot1_forward_position_controller",
    ]
    controller_spawners = [controller_spawner(controllers_active)] + [
        controller_spawner(controllers_inactive, active=False)
    ]
    # There may be other controllers of the joints, but this is the initially-started one
    robot2_initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot2_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
        condition=IfCondition(robot2_activate_joint_controller),
    )
    robot1_initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot1_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
        condition=IfCondition(robot1_activate_joint_controller),
    )
    robot2_initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot2_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--inactive",
        ],
        condition=UnlessCondition(robot2_activate_joint_controller),
    )
    robot1_initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot1_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--inactive",
        ],
        condition=UnlessCondition(robot1_activate_joint_controller),
    )
    nodes_to_start = [
        robot_state_publisher_node,
        control_node,
        robot2_dashboard_client_node,
        robot1_dashboard_client_node,
        robot2_urscript_interface,
        robot1_urscript_interface,
        rviz_node,
        robot2_initial_joint_controller_spawner_stopped,
        robot1_initial_joint_controller_spawner_stopped,
        robot2_initial_joint_controller_spawner_started,
        robot1_initial_joint_controller_spawner_started,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
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
            default_value="ur16e",
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
            default_value="ur16e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot2_robot_ip",
            default_value="192.168.1.103",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_robot_ip",
            default_value="192.168.1.102",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot2_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    LaunchConfiguration("robot2_ur_type"),
                    "default_kinematics.yaml",
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
                    FindPackageShare("ur_description"),
                    "config",
                    LaunchConfiguration("robot1_ur_type"),
                    "default_kinematics.yaml",
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
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "combined_controllers.yaml",
                ]
            ),
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_launchfile",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "launch",
                    "rsp.launch.py",
                ]
            ),
            description="Launchfile (absolute path) providing the description. "
            "The launchfile has to start a robot_state_publisher node that "
            "publishes the description topic.",
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
            description="Enable headless mode for robot control for both arms.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot2_initial_joint_controller",
            default_value="robot2_scaled_joint_trajectory_controller",
            description="Initially loaded robot controller for the robot2 robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_initial_joint_controller",
            default_value="robot1_scaled_joint_trajectory_controller",
            description="Initially loaded robot controller for the robot1 robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot2_activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller for the robot2 robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller for the robot1 robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("my_dual_robot_cell_description"), "rviz", "urdf.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot2_launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="update_rate_config_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("my_dual_robot_cell_control"),
                        "config",
                    ]
                ),
                "/",
                "update_rate.yaml",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="my_dual_robot_cell_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
