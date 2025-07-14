import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString

from launch import LaunchDescription
from launch.actions import OpaqueFunction  # NEW: for deferred execution
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression,
    TextSubstitution,
)


def generate_launch_description():
    # ---------------------------------------------------------------------
    # Package names & default paths
    # ---------------------------------------------------------------------
    bringup_pkg = "arcs_cohort_bringup"
    rviz_pkg = "arcs_cohort_rviz"
    gazebo_sim_pkg = "arcs_cohort_gazebo_sim"
    sensor_preproc_pkg = "arcs_cohort_sensor_preprocessor"
    nav_pkg = "arcs_cohort_navigation"

    default_world_path = os.path.join(
        get_package_share_directory(gazebo_sim_pkg),
        "worlds",
        "test_obstacles_world_1.world",
    )
    default_model_file = os.path.join(
        get_package_share_directory(gazebo_sim_pkg), "description", "robot.gazebo.xacro"
    )
    default_rviz_config_file = os.path.join(
        get_package_share_directory(rviz_pkg), "rviz", "cohort_default.rviz"
    )
    default_rover_config_file = os.path.join(
        get_package_share_directory(bringup_pkg), "config", "two_rovers.yaml"
    )
    default_sensor_preprocessor_config_file = os.path.join(
        get_package_share_directory(sensor_preproc_pkg),
        "config",
        "sensor_preprocessor.yaml",
    )
    default_ros2_control_params_file = os.path.join(
        get_package_share_directory(gazebo_sim_pkg),
        "config",
        "gazebo_ros2_control_params.yaml",
    )
    default_scan_topic = "scan/merged/scan"
    default_pointcloud_topic = "camera/points/filtered/base"
    default_local_costmap_plugins = TextSubstitution(
        text='["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]'
    )
    default_global_costmap_plugins = TextSubstitution(
        text='["static_layer", "obstacle_layer", "stvl_layer", "inflation_layer"]'
    )
    default_ekf_params = os.path.join(
        get_package_share_directory(nav_pkg), "config", "ekf_params.yaml"
    )
    default_slam_params = os.path.join(
        get_package_share_directory(nav_pkg), "config", "slam_params.yaml"
    )
    default_nav2_params = os.path.join(
        get_package_share_directory(nav_pkg), "config", "nav2_mppi_stamped_params.yaml"
    )
    default_log_level = "INFO"

    # ---------------------------------------------------------------------
    # Declare launch arguments
    # ---------------------------------------------------------------------
    declare_args = [
        DeclareLaunchArgument("world", default_value=default_world_path),
        DeclareLaunchArgument("model_file", default_value=default_model_file),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config_file),
        DeclareLaunchArgument("rover_config", default_value=default_rover_config_file),
        DeclareLaunchArgument(
            "sensor_preprocessor_config",
            default_value=default_sensor_preprocessor_config_file,
        ),
        DeclareLaunchArgument(
            "camera_resolution",
            default_value="VGA",
            description=(
                'ZED resolution: "HD2K", "HD1080", "HD720", or "VGA" (default).'
            ),
        ),
        DeclareLaunchArgument("lidar_update_rate", default_value="10"),
        DeclareLaunchArgument(
            "ros2_control_params", default_value=default_ros2_control_params_file
        ),
        DeclareLaunchArgument("scan_topic", default_value=default_scan_topic),
        DeclareLaunchArgument(
            "pointcloud_topic", default_value=default_pointcloud_topic
        ),
        DeclareLaunchArgument(
            "local_costmap_plugins", default_value=default_local_costmap_plugins
        ),
        DeclareLaunchArgument(
            "global_costmap_plugins", default_value=default_global_costmap_plugins
        ),
        DeclareLaunchArgument("ekf_params", default_value=default_ekf_params),
        DeclareLaunchArgument("slam_params", default_value=default_slam_params),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        DeclareLaunchArgument("log_level", default_value=default_log_level),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_clock_bridge", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_lidar", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gazebo_sim", default_value="true"),
        DeclareLaunchArgument("use_sensor_preprocessor", default_value="true"),
        DeclareLaunchArgument("use_ros2_control", default_value="true"),
        DeclareLaunchArgument("use_navigation", default_value="true"),
        DeclareLaunchArgument("use_ekf", default_value="true"),
        DeclareLaunchArgument("use_slam", default_value="true"),
        DeclareLaunchArgument("use_nav2", default_value="true"),
        DeclareLaunchArgument("use_joystick", default_value="false"),
        DeclareLaunchArgument("use_keyboard", default_value="false"),
    ]

    # ---------------------------------------------------------------------
    # Launch-configs (Substitutions used later)
    # ---------------------------------------------------------------------
    world = LaunchConfiguration("world")
    model_file = LaunchConfiguration("model_file")
    camera_resolution = LaunchConfiguration("camera_resolution")
    rviz_config = LaunchConfiguration("rviz_config")
    rover_config = LaunchConfiguration("rover_config")
    sensor_preproc_cfg = LaunchConfiguration("sensor_preprocessor_config")
    lidar_update_rate = LaunchConfiguration("lidar_update_rate")
    ros2_control_params = LaunchConfiguration("ros2_control_params")
    scan_topic = LaunchConfiguration("scan_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    local_costmap_plugs = LaunchConfiguration("local_costmap_plugins")
    global_costmap_plugs = LaunchConfiguration("global_costmap_plugins")
    ekf_params = LaunchConfiguration("ekf_params")
    slam_params = LaunchConfiguration("slam_params")
    nav2_params = LaunchConfiguration("nav2_params")
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_clock_bridge = LaunchConfiguration("use_clock_bridge")
    use_camera = LaunchConfiguration("use_camera")
    use_lidar = LaunchConfiguration("use_lidar")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo_sim = LaunchConfiguration("use_gazebo_sim")
    use_sensor_preproc = LaunchConfiguration("use_sensor_preprocessor")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_navigation = LaunchConfiguration("use_navigation")
    use_ekf = LaunchConfiguration("use_ekf")
    use_slam = LaunchConfiguration("use_slam")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_joystick = LaunchConfiguration("use_joystick")
    use_keyboard = LaunchConfiguration("use_keyboard")

    # ---------------------------------------------------------------------
    # Gazebo (top-level, independent of rover loop)
    # ---------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    start_clock_bridge = Node(
        condition=IfCondition(use_clock_bridge),
        name="clock_bridge",
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        output="screen",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # ---------------------------------------------------------------------
    # Deferred setup: everything that needs the rover YAML
    # ---------------------------------------------------------------------
    def launch_setup(context, *args, **kwargs):
        actions = []

        # Resolve the rover-config path *after* substitutions are available
        rover_cfg_path = rover_config.perform(context)
        with open(rover_cfg_path, "r") as f:
            rovers_yaml = yaml.safe_load(f)

        for rover in rovers_yaml["rovers"]:
            ns = rover["name"]
            prefix = ""
            x, y, z = rover["x"], rover["y"], rover["z"]

            push_namespace = PushRosNamespace(namespace=ns)

            prefix_ = PythonExpression(["'", prefix, "_' if '", prefix, "' else ''"])
            ns_ = PythonExpression(["'", ns, "/' if '", ns, "' else ''"])
            _ns_ = PythonExpression(["'/", ns, "/' if '", ns, "' else ''"])

            substituted_ros2_ctrl = ReplaceString(
                source_file=ros2_control_params,
                replacements={
                    "<NAMESPACE>": ns,
                    "<NAMESPACE_>": ns_,
                    "<_NAMESPACE_>": _ns_,
                    "<PREFIX>": prefix,
                    "<PREFIX_>": prefix_,
                },
            )

            robot_description = Command(
                [
                    "xacro ",
                    model_file,
                    " namespace:=",
                    ns,
                    " prefix:=",
                    prefix,
                    " camera_resolution:=",
                    camera_resolution,
                    " lidar_update_rate:=",
                    lidar_update_rate,
                    " ros2_control_params:=",
                    substituted_ros2_ctrl,
                    " use_joystick:=",
                    use_joystick,
                    " use_keyboard:=",
                    use_keyboard,
                    " use_camera:=",
                    use_camera,
                    " use_lidar:=",
                    use_lidar,
                    " use_ros2_control:=",
                    use_ros2_control,
                ]
            )

            # Robot state publisher
            rsp_node = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
            )

            # Use gazebo_sim.launch.py from arcs_cohort_gazebo_sim package to
            # bring up Gazebo-related nodes, etc. for rover, but without
            # bringing up an instance of Gazebo itself.
            gazebo_sim_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory(gazebo_sim_pkg),
                            "launch",
                            "gazebo_sim.launch.py",
                        )
                    ]
                ),
                condition=IfCondition(use_gazebo_sim),
                launch_arguments={
                    "namespace": ns,
                    "prefix": prefix,
                    "world": world,
                    "model_file": model_file,
                    "camera_resolution": camera_resolution,
                    "lidar_update_rate": lidar_update_rate,
                    "ros2_control_params": ros2_control_params,
                    "log_level": log_level,
                    "use_sim_time": use_sim_time,
                    "use_clock_bridge": "false",
                    "use_gazebo": "false",
                    "use_spawner": "false",
                    "use_camera": use_camera,
                    "use_lidar": use_lidar,
                    "use_rsp": "false",
                    "use_jsp": "false",
                    "use_jsp_gui": "false",
                    "use_ros2_control": use_ros2_control,
                    "use_joystick": use_joystick,
                    "use_keyboard": use_keyboard,
                }.items(),
            )

            # Spawn rover instance into Gazebo sim
            spawn_entity_node = Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    ns,
                    "-x",
                    str(x),
                    "-y",
                    str(y),
                    "-z",
                    str(z),
                ],
                output="screen",
            )

            # Launch rover-specific RViz instance
            rviz_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory(rviz_pkg),
                            "launch",
                            "rviz.launch.py",
                        )
                    ]
                ),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    "namespace": ns,
                    "prefix": prefix,
                    "rviz_config": rviz_config,
                    "use_sim_time": use_sim_time,
                    "log_level": log_level,
                }.items(),
            )

            # Launch rover-specific sensor preprocessor
            sensor_preproc_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory(sensor_preproc_pkg),
                            "launch",
                            "sensor_preprocessor_bringup.launch.py",
                        )
                    ]
                ),
                condition=IfCondition(use_sensor_preproc),
                launch_arguments={
                    "namespace": ns,
                    "prefix": prefix,
                    "sensor_preprocessor_config": sensor_preproc_cfg,
                    "use_sim_time": use_sim_time,
                    "log_level": log_level,
                }.items(),
            )

            # Launch rover-specific navigation
            navigation_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory(nav_pkg),
                            "launch",
                            "navigation_bringup.launch.py",
                        )
                    ]
                ),
                condition=IfCondition(use_navigation),
                launch_arguments={
                    "namespace": ns,
                    "prefix": prefix,
                    "scan_topic": scan_topic,
                    "pointcloud_topic": pointcloud_topic,
                    "local_costmap_plugins": local_costmap_plugs,
                    "global_costmap_plugins": global_costmap_plugs,
                    "ekf_params": ekf_params,
                    "slam_params": slam_params,
                    "nav2_params": nav2_params,
                    "use_sim_time": use_sim_time,
                    "use_ekf": use_ekf,
                    "use_slam": use_slam,
                    "use_nav2": use_nav2,
                    "log_level": log_level,
                }.items(),
            )

            # Group rover-specific nodes under its namespace
            actions.append(
                GroupAction(
                    [
                        push_namespace,
                        rsp_node,
                        spawn_entity_node,
                    ]
                )
            )
            # Rover-specific launchers already have namespace as param
            actions.extend(
                [
                    gazebo_sim_launch,
                    rviz_launch,
                    sensor_preproc_launch,
                    navigation_launch,
                ]
            )

        return actions

    # ---------------------------------------------------------------------
    # Assemble top-level launch description
    # ---------------------------------------------------------------------
    ld = []
    ld.extend(declare_args)
    ld.extend([gazebo_launch, start_clock_bridge])
    ld.append(OpaqueFunction(function=launch_setup))  # NEW: deferred rover loop

    return LaunchDescription(ld)
