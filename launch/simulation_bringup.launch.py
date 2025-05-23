import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Package and file paths
    rviz_pkg = "arcs_cohort_rviz"
    gazebo_sim_pkg = "arcs_cohort_gazebo_sim"
    sensor_preproc_pkg = "arcs_cohort_sensor_preprocessor"
    nav_pkg = "arcs_cohort_navigation"

    # Defaults
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
    default_sensor_preprocessor_config_file = os.path.join(
        get_package_share_directory(sensor_preproc_pkg),
        "config",
        "sensor_preprocessor.yaml",
    )
    default_ros2_control_params_file = os.path.join(
        get_package_share_directory(gazebo_sim_pkg), "config", "gazebo_ros2_control_params.yaml"
    )
    default_scan_topic = "scan/merged/scan"
    default_pointcloud_topic = "camera/points/filtered/base"
    default_odom_topic = "odometry/filtered"
    default_local_costmap_plugins = TextSubstitution(
        text='["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]'
    ),
    default_global_costmap_plugins = TextSubstitution(
        text='["static_layer", "obstacle_layer", "stvl_layer", "inflation_layer"]'
    ),
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

    # Declare launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace under which to bring up nodes, topics, etc.",
    )
    declare_prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description=(
            "A prefix for the names of joints, links, etc. in the robot model). "
            "E.g. 'base_link' will become 'cohort1_base_link' if prefix "
            "is set to 'cohort1'."
        ),
    )
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="Path to the world file to load",
    )
    declare_model_file_arg = DeclareLaunchArgument(
        "model_file",
        default_value=default_model_file,
        description="Path to the robot model file",
    )
    declare_camera_resolution_arg = DeclareLaunchArgument(
        "camera_resolution",
        default_value="VGA",
        description=(
            "Resolution profile of the simulated Stereolabs Zed camera."
            'Options: "HD2K" (2208x1242), "HD1080" (1920x1080), '
            '"HD720" (1280x720) or "VGA" (672x376).'
        ),
    )
    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config_file,
        description="Path to RViz configuration file",
    )
    declare_lidar_update_rate_arg = DeclareLaunchArgument(
        "lidar_update_rate",
        default_value="10",
        description="Set the update rate of the LiDAR sensor.",
    )
    declare_sensor_preprocessor_config_arg = DeclareLaunchArgument(
        "sensor_preprocessor_config",
        default_value=default_sensor_preprocessor_config_file,
        description="Path to sensor preprocessor configuration file",
    )
    declare_ros2_control_params_arg = DeclareLaunchArgument(
        "ros2_control_params",
        default_value=default_ros2_control_params_file,
        description="Path to the params file for ros2_control.",
    )
    declare_scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value=default_scan_topic,
        description="Laser scan topic to be used by navigation.",
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value=default_pointcloud_topic,
        description="Point cloud topic to be used by navigation.",
    )
    declare_odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value=default_odom_topic,
        description="Odometry topic to be used by navigation.",
    )
    declare_local_costmap_plugins_arg = DeclareLaunchArgument(
        "local_costmap_plugins",
        default_value=default_local_costmap_plugins,
        description="YAML-style list of plugins to use in the local costmap."
    )
    declare_global_costmap_plugins_arg = DeclareLaunchArgument(
        "global_costmap_plugins",
        default_value=default_global_costmap_plugins,
        description="YAML-style list of plugins to use in the global costmap."
    )
    declare_ekf_params_arg = DeclareLaunchArgument(
        "ekf_params",
        default_value=default_ekf_params,
        description="Path to the params file to load for the robot_localization package EKF node.",
    )
    declare_slam_params_arg = DeclareLaunchArgument(
        "slam_params",
        default_value=default_slam_params,
        description="Path to the params file to load for the slam_toolbox package SLAM node.",
    )
    declare_nav2_params_arg = DeclareLaunchArgument(
        "nav2_params",
        default_value=default_nav2_params,
        description="Path to the params file to load for the nav2_bringup package Nav2 bringup launcher.",
    )
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=default_log_level,
        description="Set the log level for nodes.",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    declare_use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="false",
        description="If true, include the lidar in the robot description.",
    )
    declare_use_rsp_arg = DeclareLaunchArgument(
        "use_rsp", default_value="true", description="Launch robot_state_publisher."
    )
    declare_use_jsp_arg = DeclareLaunchArgument(
        "use_jsp", default_value="false", description="Launch joint_state_publisher."
    )
    declare_use_jsp_gui_arg = DeclareLaunchArgument(
        "use_jsp_gui",
        default_value="false",
        description="Launch joint_state_publisher_gui",
    )
    declare_use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz"
    )
    declare_use_gazebo_sim_arg = DeclareLaunchArgument(
        "use_gazebo_sim", default_value="true", description="Launch Gazebo"
    )
    declare_use_sensor_preprocessor_arg = DeclareLaunchArgument(
        "use_sensor_preprocessor",
        default_value="true",
        description="Launch the sensor preprocessor pipeline.",
    )
    declare_use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="true",
        description="Use ROS2 Control for the robot.",
    )
    declare_use_navigation_arg = DeclareLaunchArgument(
        "use_navigation",
        default_value="true",
        description="Launch the navigation stack.",
    )
    declare_use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="true",
        description="Launch robot_localization package EKF node.",
    )
    declare_use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true",
        description="Launch slam_toolbox package SLAM node.",
    )
    declare_use_nav2_arg = DeclareLaunchArgument(
        "use_nav2",
        default_value="true",
        description="Launch nav2_bringup package Nav2 bringup launcher.",
    )
    declare_use_joystick_arg = DeclareLaunchArgument(
        "use_joystick",
        default_value="false",
        description="Launch robot teleop with joystick.",
    )
    declare_use_keyboard_arg = DeclareLaunchArgument(
        "use_keyboard",
        default_value="false",
        description="Launch robot teleop with keyboard.",
    )

    # Launch configurations
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    world = LaunchConfiguration("world")
    model_file = LaunchConfiguration("model_file")
    camera_resolution = LaunchConfiguration("camera_resolution")
    rviz_config = LaunchConfiguration("rviz_config")
    sensor_preprocessor_config = LaunchConfiguration("sensor_preprocessor_config")
    lidar_update_rate = LaunchConfiguration("lidar_update_rate")
    ros2_control_params = LaunchConfiguration("ros2_control_params")
    scan_topic = LaunchConfiguration("scan_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    local_costmap_plugins = LaunchConfiguration("local_costmap_plugins")
    global_costmap_plugins = LaunchConfiguration("global_costmap_plugins")
    ekf_params = LaunchConfiguration("ekf_params")
    slam_params = LaunchConfiguration("slam_params")
    nav2_params = LaunchConfiguration("nav2_params")
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_lidar = LaunchConfiguration("use_lidar")
    use_rsp = LaunchConfiguration("use_rsp")
    use_jsp = LaunchConfiguration("use_jsp")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo_sim = LaunchConfiguration("use_gazebo_sim")
    use_sensor_preprocessor = LaunchConfiguration("use_sensor_preprocessor")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_navigation = LaunchConfiguration("use_navigation")
    use_ekf = LaunchConfiguration("use_ekf")
    use_slam = LaunchConfiguration("use_slam")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_joystick = LaunchConfiguration("use_joystick")
    use_keyboard = LaunchConfiguration("use_keyboard")

    # Log info
    log_info = LogInfo(msg=['Simulation bringup launching with namespace: ', namespace, ', prefix: ', prefix])

    # Use PushRosNamespace to apply the namespace to all nodes below
    push_namespace = PushRosNamespace(namespace=namespace)

    # Build the prefix with underscore.
    # This expression will evaluate to, for example, "cohort1_" if
    # the prefix is "cohort1", or to an empty string if prefix is empty.
    prefix_ = PythonExpression(
        ["'", prefix, "_' if '", prefix, "' else ''"]
    )

    # Build the namespace with slash
    # This expression will evaluate to, for example, "cohort1/" if
    # the namespace is "cohort1", or to an empty string if namespace is empty.
    namespace_ = PythonExpression(
        ["'", namespace, "/' if '", namespace, "' else ''"]
    )

    # Build the namespace with leading and trailing slashes.
    # This expression will evaluate to, for example, "/cohort1/" if
    # the namespace is "cohort1", or to an empty string if namespace is empty.
    _namespace_ = PythonExpression(
        ["'/", namespace, "/' if '", namespace, "' else ''"]
    )

    # Perform substitutions of <NAMESPACE> and <PREFIX> in EKF params file
    substituted_ros2_control_params = ReplaceString(
        source_file=ros2_control_params,
        replacements={
            '<NAMESPACE>': namespace,
            '<NAMESPACE_>': namespace_,
            '<_NAMESPACE_>': _namespace_,
            '<PREFIX>': prefix,
            '<PREFIX_>': prefix_,
        }
    )

    # Robot description from Xacro, including the conditional robot name prefix.
    robot_description = Command(
        [
            "xacro ",
            model_file,
            " namespace:=",
            namespace,
            " prefix:=",
            prefix,
            " camera_resolution:=",
            camera_resolution,
            " lidar_update_rate:=",
            lidar_update_rate,
            " ros2_control_params:=",
            substituted_ros2_control_params,
            " use_joystick:=",
            use_joystick,
            " use_keyboard:=",
            use_keyboard,
            " use_lidar:=",
            use_lidar,
            " use_ros2_control:=",
            use_ros2_control,
        ]
    )

    # Robot State Publisher node
    rsp_node = Node(
            condition=IfCondition(use_rsp),
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": use_sim_time}
            ],
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
    )

    # Joint State Publisher node
    jsp_node = Node(
            condition=IfCondition(
                PythonExpression(
                    ["'", use_jsp, "' == 'true' and '", use_jsp_gui, "' != 'true'"]
                )
            ),
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
    )

    # Joint State Publisher GUI node
    jsp_gui_node = Node(
            condition=IfCondition(use_jsp_gui),
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
    )

    # Include rviz.launch.py
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
            "namespace": namespace,
            "prefix": prefix,
            "rviz_config": rviz_config,
            "log_level": log_level,
        }.items(),
    )

    # Include gazebo_sim.launch.py
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
            "namespace": namespace,
            "prefix": prefix,
            "world": world,
            "model_file": model_file,
            "camera_resolution": camera_resolution,
            "lidar_update_rate": lidar_update_rate,
            "ros2_control_params": ros2_control_params,
            "log_level": log_level,
            "use_sim_time": use_sim_time,
            "use_lidar": use_lidar,
            "use_rsp": "false",  # Disable RSP in gazebo_sim
            "use_jsp": "false",  # Disable JSP in gazebo_sim
            "use_jsp_gui": "false",  # Disable JSP GUI in gazebo_sim
            "use_ros2_control": use_ros2_control,
            "use_joystick": use_joystick,
            "use_keyboard": use_keyboard,
            "use_navigation": use_navigation,
        }.items(),
    )

    # Include sensor_preprocessor_bringup.launch.py
    sensor_preprocessor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(sensor_preproc_pkg),
                    "launch",
                    "sensor_preprocessor_bringup.launch.py",
                )
            ]
        ),
        condition=IfCondition(use_sensor_preprocessor),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
            "sensor_preprocessor_config": sensor_preprocessor_config,
            "log_level": log_level,
        }.items(),
    )

    # Include navigation_bringup.launch.py
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
                "namespace": namespace,
                "prefix": prefix,
                "scan_topic": scan_topic,
                "pointcloud_topic": pointcloud_topic,
                "odom_topic": odom_topic,
                "local_costmap_plugins": local_costmap_plugins,
                "global_costmap_plugins": global_costmap_plugins,
                "ekf_params": ekf_params,
                "slam_params": slam_params,
                "nav2_params": nav2_params,
                "use_sim_time": use_sim_time,
                "use_ekf": use_ekf,
                "use_slam": use_slam,
                "use_nav2": use_nav2,
                "log_level": log_level,
            }.items()
        )

    return LaunchDescription(
        [
            # Declare launch arguments
            declare_namespace_arg,
            declare_prefix_arg,
            declare_world_arg,
            declare_model_file_arg,
            declare_camera_resolution_arg,
            declare_rviz_config_arg,
            declare_lidar_update_rate_arg,
            declare_sensor_preprocessor_config_arg,
            declare_ros2_control_params_arg,
            declare_scan_topic_arg,
            declare_pointcloud_topic_arg,
            declare_odom_topic_arg,
            declare_local_costmap_plugins_arg,
            declare_global_costmap_plugins_arg,
            declare_ekf_params_arg,
            declare_slam_params_arg,
            declare_nav2_params_arg,
            declare_log_level_arg,
            declare_use_sim_time_arg,
            declare_use_lidar_arg,
            declare_use_rsp_arg,
            declare_use_jsp_arg,
            declare_use_jsp_gui_arg,
            declare_use_rviz_arg,
            declare_use_gazebo_sim_arg,
            declare_use_sensor_preprocessor_arg,
            declare_use_ros2_control_arg,
            declare_use_navigation_arg,
            declare_use_ekf_arg,
            declare_use_slam_arg,
            declare_use_nav2_arg,
            declare_use_joystick_arg,
            declare_use_keyboard_arg,
            # Log
            log_info,
            # Nodes
            GroupAction([
                push_namespace,
                rsp_node,
                jsp_node,
                jsp_gui_node,
            ]),
            # Launchers
            rviz_launch,
            gazebo_sim_launch,
            sensor_preprocessor_launch,
            navigation_launch,
        ]
    )
