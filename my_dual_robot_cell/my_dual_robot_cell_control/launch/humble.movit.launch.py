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
    alice_ur_type = LaunchConfiguration("alice_ur_type")
    bob_ur_type = LaunchConfiguration("bob_ur_type")

    alice_robot_ip = LaunchConfiguration("alice_robot_ip")
    bob_robot_ip = LaunchConfiguration("bob_robot_ip")

    alice_use_mock_hardware = LaunchConfiguration("alice_use_mock_hardware")
    alice_mock_sensor_commands = LaunchConfiguration("alice_mock_sensor_commands")
    bob_use_mock_hardware = LaunchConfiguration("bob_use_mock_hardware")
    bob_mock_sensor_commands = LaunchConfiguration("bob_mock_sensor_commands")

    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    headless_mode = LaunchConfiguration("headless_mode")

    # Robot specific arguments
    alice_use_mock_hardware = LaunchConfiguration("alice_use_mock_hardware")
    alice_mock_sensor_commands = LaunchConfiguration("alice_mock_sensor_commands")

    bob_use_mock_hardware = LaunchConfiguration("bob_use_mock_hardware")
    bob_mock_sensor_commands = LaunchConfiguration("bob_mock_sensor_commands")


    headless_mode = LaunchConfiguration("headless_mode")

    alice_kinematics_parameters_file = LaunchConfiguration(
        "alice_kinematics_parameters_file"
    )
    bob_kinematics_parameters_file = LaunchConfiguration(
        "bob_kinematics_parameters_file"
    )
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

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
            "alice_robot_ip:=",
            alice_robot_ip,
            " ",
            "bob_robot_ip:=",
            bob_robot_ip,
            " ",
            "alice_ur_type:=",
            alice_ur_type,
            " ",
            "bob_ur_type:=",
            bob_ur_type,
            " ",
            "alice_use_mock_hardware:=",
            alice_use_mock_hardware,
            " ",
            "bob_use_mock_hardware:=",
            bob_use_mock_hardware,
            " ",
            "alice_kinematics_parameters_file:=",
            alice_kinematics_parameters_file,
            " ",
            "bob_kinematics_parameters_file:=",
            bob_kinematics_parameters_file,
            " ",
            "alice_mock_sensor_commands:=",
            alice_mock_sensor_commands,
            " ",
            "bob_mock_sensor_commands:=",
            bob_mock_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
        ]
    )
    robot_description = {"robot_description": robot_description_content}



    # MoveIt Configuration
    moveit_config_package = LaunchConfiguration("moveit_config_package")

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", 'my_dual_robot_cell.srdf']
            ),
            
        ]
    )
    robot_description_semantic = {"robot_description_semantic": launch_ros.descriptions.ParameterValue(robot_description_semantic_content,value_type=str)}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            moveit_config_package.perform(context),
            os.path.join("config", 'joint_limits.yaml'),
        )
    }
    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(moveit_config_package.perform(context), "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml(moveit_config_package.perform(context), "config/moveit_controllers.yaml") 
    print(moveit_config_package.perform(context))
    print(controllers_yaml)
    controllers_yaml["alice_scaled_joint_trajectory_controller"]["default"] = True
    controllers_yaml["bob_scaled_joint_trajectory_controller"]["default"] = True
    # the scaled_joint_trajectory_controller does not work on fake hardware

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
            warehouse_ros_config,
        ],
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "moveit.rviz"]
    )
    # Servo node for realtime control
    alice_servo_yaml = load_yaml(moveit_config_package.perform(context), "config/alice_ur_servo.yaml")
    alice_servo_params = {"moveit_servo": alice_servo_yaml}
    alice_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            alice_servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )
     # Servo node for realtime control
    bob_servo_yaml = load_yaml(moveit_config_package.perform(context), "config/bob_ur_servo.yaml")
    bob_servo_params = {"moveit_servo": bob_servo_yaml}
    bob_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            bob_servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )


    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
        ],
    )


    nodes_to_start = [
        rviz_node,
        move_group_node,
        alice_servo_node,
        bob_servo_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_ur_type",
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
            default_value="ur3",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_ur_type",
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
            default_value="ur3",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_robot_ip",
            default_value="192.168.56.102",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_robot_ip",
            default_value="192.168.56.101",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "alice_calibration.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "bob_calibration.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_mock_sensor_commands",
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
            "alice_use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_mock_sensor_commands",
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
            "alice_initial_joint_controller",
            default_value="alice_scaled_joint_trajectory_controller",
            description="Initially loaded robot controller for the alice robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_initial_joint_controller",
            default_value="bob_scaled_joint_trajectory_controller",
            description="Initially loaded robot controller for the bob robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller for the alice robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller for the bob robot arm.",
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
            "alice_launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_launch_dashboard_client",
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
