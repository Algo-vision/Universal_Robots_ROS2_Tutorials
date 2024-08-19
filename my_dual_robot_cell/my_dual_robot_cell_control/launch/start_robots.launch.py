from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def launch_setup():
    # Initialize Arguments
    robot2_ur_type = LaunchConfiguration("robot2_ur_type")
    robot1_ur_type = LaunchConfiguration("robot1_ur_type")

    robot2_robot_ip = LaunchConfiguration("robot2_robot_ip")
    robot1_robot_ip = LaunchConfiguration("robot1_robot_ip")

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

    # Single controller manager comprising of controllers for both arms
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file, allow_substs=True),
            # We use the tf_prefix as substitution in there, so that's why we keep it as an
            # argument for this launchfile
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
        condition=UnlessCondition(robot2_use_mock_hardware),
    )

    robot1_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        namespace="robot1",
        parameters=[{"robot_ip": robot1_robot_ip}],
        output="screen",
        condition=UnlessCondition(robot1_use_mock_hardware),
    )

    robot2_controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        namespace="robot2",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(robot2_use_mock_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": robot2_activate_joint_controller},
            {
                "consistent_controllers": [
                    "robot2_io_and_status_controller",
                    "robot2_force_torque_sensor_broadcaster",
                    "robot2_joint_state_broadcaster",
                    "robot2_speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    robot1_controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        namespace="robot1",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(robot1_use_mock_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": robot1_activate_joint_controller},
            {
                "consistent_controllers": [
                    "robot1_io_and_status_controller",
                    "robot1_force_torque_sensor_broadcaster",
                    "robot1_joint_state_broadcaster",
                    "robot1_speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Spawn controllers
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
        "robot2_io_and_status_controller",
        "robot1_io_and_status_controller",
        "robot2_speed_scaling_state_broadcaster",
        "robot1_speed_scaling_state_broadcaster",
        "robot2_force_torque_sensor_broadcaster",
        "robot1_force_torque_sensor_broadcaster",
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

    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            "robot2_robot_ip": robot2_robot_ip,
            "robot1_robot_ip": robot1_robot_ip,
            "robot2_ur_type": robot2_ur_type,
            "robot1_ur_type": robot1_ur_type,
        }.items(),
    )

    nodes_to_start = [
        control_node,
        robot2_dashboard_client_node,
        robot1_dashboard_client_node,
        robot2_controller_stopper_node,
        robot1_controller_stopper_node,
        robot2_urscript_interface,
        robot1_urscript_interface,
        rsp,
        rviz_node,
        robot2_initial_joint_controller_spawner_stopped,
        robot1_initial_joint_controller_spawner_stopped,
        robot2_initial_joint_controller_spawner_started,
        robot1_initial_joint_controller_spawner_started,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
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
    return LaunchDescription(declared_arguments + launch_setup())
