import os

import yaml

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders
    pkg_path = FindPackageShare(package="parc_robot_bringup").find("parc_robot_bringup")
    pkg_description = FindPackageShare(package="parc_robot_description").find(
        "parc_robot_description"
    )
    pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find(
        "ros_gz_sim"
    )

    bridge_params = os.path.join(pkg_path, "config/gz_bridge.yaml")
    robot_controllers = os.path.join(pkg_path, "config/controllers.yaml")
    rviz_config_file = os.path.join(pkg_path, "rviz/task1.rviz")
    goal_location_sdf = os.path.join(pkg_path, "models/goal_location/model.sdf")
    world_filename = "mini.sdf"
    # world_filename = "gen3.sdf"
    # world_filename = "gen2.sdf"
    # world_filename = "generated.sdf"
    # world_filename = "empty.world"
    # world_filename = "parc_task1.world"
    world_path = os.path.join(pkg_path, "worlds", world_filename)
    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_path, "models")
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    world = LaunchConfiguration("world")
    route = LaunchConfiguration("route")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value=world_path,
        description="Full path to the world model to load",
    )

    declare_ros2_control_cmd = DeclareLaunchArgument(
        name="use_ros2_control",
        default_value="False",
        description="Use ros2_control if true",
    )

    declare_route_cmd = DeclareLaunchArgument(
        name="route",
        default_value="route1",
        description="Route for robot navigation",
        choices=["route1", "route2", "route3"],
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, "launch", "robot_state_publisher_launch.py")]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Launch Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Function to spawn entities in Gazebo with pose dependent on route chosen
    def spawn_gazebo_entities(context):

        nonlocal route, goal_location_sdf

        # List of actions to be added to the launch description later
        actions = []

        # Set path to route parameter yaml file
        params_file = os.path.join(
            pkg_path,
            "config/",
            "task1_" + context.launch_configurations["route"] + "_params.yaml",
        )

        # Open route specific yaml file
        if os.path.exists(params_file):
            with open(params_file, "r") as f:
                params = yaml.safe_load(f)

                spawn_x_val = str(params["/**"]["ros__parameters"]["x"])
                spawn_y_val = str(params["/**"]["ros__parameters"]["y"])
                spawn_z_val = str(params["/**"]["ros__parameters"]["z"])
                spawn_yaw_val = str(params["/**"]["ros__parameters"]["yaw"])
                goal_x_val = str(params["/**"]["ros__parameters"]["goal_x"])
                goal_y_val = str(params["/**"]["ros__parameters"]["goal_y"])
                goal_z_val = str(params["/**"]["ros__parameters"]["goal_z"])

                route_params_file = LaunchConfiguration("route_params_file")

                # Declare route params file launch argument
                actions.append(
                    DeclareLaunchArgument(
                        name="route_params_file",
                        default_value=params_file,
                    )
                )

                # Load route parameters file
                actions.append(
                    Node(
                        package="parc_robot_bringup",
                        executable="load_task1_params.py",
                        parameters=[route_params_file],
                    )
                )

                # Spawn PARC robot in Gazebo
                actions.append(
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        output="screen",
                        arguments=[
                            "-topic",
                            "robot_description",
                            "-name",
                            "parc_robot",
                            "-x",
                            spawn_x_val,
                            "-y",
                            spawn_y_val,
                            "-z",
                            spawn_z_val,
                            "-Y",
                            spawn_yaw_val,
                        ],
                    )
                )

                # Spawn goal location
                actions.append(
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        output="screen",
                        arguments=[
                            "-file",
                            goal_location_sdf,
                            "-name",
                            "goal_location",
                            "-x",
                            goal_x_val,
                            "-y",
                            goal_y_val,
                            "-z",
                            goal_z_val,
                        ],
                    )
                )

        return actions

    # Start Gazebo ROS bridge
    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    # Start Gazebo ROS Left Image bridge
    start_gazebo_ros_left_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/left_camera/image_raw"],
        output="screen",
    )

    # Start Gazebo ROS Left Image bridge
    start_gazebo_ros_right_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/right_camera/image_raw"],
        output="screen",
    )
    
    # Start Gazebo ROS ZED2 Center Image bridge
    start_gazebo_ros_zed2_center_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/zed2_camera_center/image_raw"],
        output="screen",
    )
     
     # Start Gazebo ROS ZED2 Left Raw Image bridge
    start_gazebo_ros_zed2_left_raw_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/zed2_left_raw/image_raw"],
        output="screen",
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # Spawn robot_base_controller
    start_robot_base_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robot_base_controller'],
        output='screen'
    )

    # Delayed start_robot_base_controller_cmd action
    start_delayed_robot_base_controller_cmd = TimerAction(
        period=19.0, actions=[start_robot_base_controller_cmd]
    )

    # Spawn joint_state_broadcaser
    start_joint_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_broadcaster'],
        output='screen'
    )

    # Delayed joint_broadcaster_cmd action
    start_delayed_joint_broadcaster_cmd = TimerAction(
        period=19.0, actions=[start_joint_broadcaster_cmd]
    )

    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=start_joint_broadcaster_cmd,
    #         on_exit=[start_rviz_cmd],
    #     )
    # )

    # start_delayed_joint_broadcaster_cmd = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=start_robot_state_publisher_cmd,
    #         on_exit=[start_joint_broadcaster_cmd],
    #     )
    # )

    # start_delayed_robot_base_controller_cmd = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=start_joint_broadcaster_cmd,
    #         on_exit=[start_robot_base_controller_cmd],
    #     )
    # )

    # COMMENT OUT LATER
    # Start teleop node
    start_teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_path, "launch", "teleop_launch.py")]
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_ros2_control_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_route_cmd)
    ld.add_action(set_env_vars_resources)

    # Add any actions
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_teleop_cmd)
    ld.add_action(OpaqueFunction(function=spawn_gazebo_entities))
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_left_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_right_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_zed2_center_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_zed2_left_raw_image_bridge_cmd)
    # ld.add_action(start_robot_base_controller_cmd)
    # ld.add_action(start_joint_broadcaster_cmd)

    # ld.add_action(start_delayed_robot_base_controller_cmd)
    # ld.add_action(start_delayed_joint_broadcaster_cmd)

    # ld.add_action(delay_rviz_after_joint_state_broadcaster_spawner)

    return ld
