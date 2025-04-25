import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package and file paths
    pkg_bringup = "arcs_cohort_bringup"
    pkg_gazebo_sim = "arcs_cohort_gazebo_sim"
    pkg_description = "arcs_cohort_description"
    pkg_sensor_preproc = "arcs_cohort_sensor_preprocessor"
    pkg_nav = "arcs_cohort_navigation"

    # Defaults
    default_world_path = os.path.join(
        get_package_share_directory(pkg_gazebo_sim),
        "worlds",
        "test_obstacles_world_1.world",
    )
    default_model_path = "description/robot.urdf.xacro"
    default_rviz_config_template_file = os.path.join(
        get_package_share_directory(pkg_description),
        "rviz_config",
        "robot_model.rviz.template",
    )
    default_rviz_config_file = os.path.join(
        get_package_share_directory(pkg_description), "rviz_config", "robot_model.rviz"
    )
    default_ekf_params = os.path.join(
        get_package_share_directory(pkg_nav), "config", "ekf_params.yaml"
    )
    default_slam_params = os.path.join(
        get_package_share_directory(pkg_nav), "config", "slam_params.yaml"
    )
    default_nav2_params = os.path.join(
        get_package_share_directory(pkg_nav), "config", "nav2_params.yaml"
    )
    default_sensor_preprocessor_config_file = os.path.join(
        get_package_share_directory(pkg_sensor_preproc),
        "config",
        "sensor_preprocessor.yaml",
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
    declare_model_package_arg = DeclareLaunchArgument(
        "model_package",
        default_value=pkg_description,
        description="Package containing the robot model",
    )
    declare_model_file_arg = DeclareLaunchArgument(
        "model_file",
        default_value=default_model_path,
        description="Relative path to the robot model file",
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
    declare_rviz_config_template_arg = DeclareLaunchArgument(
        "rviz_config_template",
        default_value=default_rviz_config_template_file,
        description="Path to the RViz config template file.",
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
    declare_use_rviz_config_template_arg = DeclareLaunchArgument(
        "use_rviz_config_template",
        default_value="true",
        description="If true, generate the RViz config from the specified RViz config template.",
    )
    declare_use_sensor_preprocessor_arg = DeclareLaunchArgument(
        "use_sensor_preprocessor",
        default_value="true",
        description="If true, launch the sensor preprocessor.",
    )
    declare_use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="true",
        description="Use ROS2 Control for the robot.",
    )
    declare_use_navigation_arg = DeclareLaunchArgument(
        "use_navigation",
        default_value="true",
        description="Bring up navigation stack.",
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
    world = LaunchConfiguration("world")
    model_package = LaunchConfiguration("model_package")
    model_file = LaunchConfiguration("model_file")
    prefix = LaunchConfiguration("prefix")
    camera_resolution = LaunchConfiguration("camera_resolution")
    namespace = LaunchConfiguration("namespace")
    rviz_config_template = LaunchConfiguration("rviz_config_template")
    rviz_config = LaunchConfiguration("rviz_config")
    sensor_preprocessor_config = LaunchConfiguration("sensor_preprocessor_config")
    lidar_update_rate = LaunchConfiguration("lidar_update_rate")
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
    use_rviz_config_template = LaunchConfiguration("use_rviz_config_template")
    use_sensor_preprocessor = LaunchConfiguration("use_sensor_preprocessor")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_navigation = LaunchConfiguration("use_navigation")
    use_ekf = LaunchConfiguration("use_ekf")
    use_slam = LaunchConfiguration("use_slam")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_joystick = LaunchConfiguration("use_joystick")
    use_keyboard = LaunchConfiguration("use_keyboard")

    # Robot description from Xacro, including the conditional robot name prefix.
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution([FindPackageShare(model_package), model_file]),
            " namespace:=",
            namespace,
            " prefix:=",
            prefix,
            " camera_resolution:=",
            camera_resolution,
            " use_lidar:=",
            use_lidar,
            " lidar_update_rate:=",
            lidar_update_rate,
            " use_ros2_control:=",
            use_ros2_control,
        ]
    )

    # Use PushRosNamespace to apply the namespace to all nodes below
    push_namespace = PushRosNamespace(namespace=namespace)

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

    # Build the prefix with underscore.
    # This expression will evaluate to, for example, "cohort_" if
    # the prefix is "cohort", or to an empty string if prefix is empty.
    prefix_ = PythonExpression(
        ["'", prefix, "_' if '", prefix, "' else ''"]
    )

    # Generate RViz config from template.
    # The robot prefix will be substituted into the RViz config template in
    # place of the ARCS_COHORT_PREFIX variable and the namespace will be
    # substituted in place of ARCS_COHORT_NAMESPACE.
    #
    # NOTE: We should probably change this approach later.  It's a neat trick,
    # but might not be manageable/scaleable.  Using fixed RViz configurations
    # for different robot/world scenarios is probably a more robust approach.
    # This type of dynamic RViz config generation could still be useful in the
    # early stages of project development to test namespacing, prefixing, etc.
    #
    namespace_env_var = PythonExpression(
        ["'/", namespace, "' if '", namespace, "' else ''"]
    )
    rviz_config_generator = ExecuteProcess(
        condition=IfCondition(use_rviz_config_template),
        cmd=[
            [
                "ARCS_COHORT_PREFIX='",
                prefix_,
                "' ",
                "ARCS_COHORT_NAMESPACE='",
                namespace_env_var,
                "' ",
                "envsubst < ",
                rviz_config_template,
                " > ",
                rviz_config,
            ]
        ],
        shell=True,
        output="screen",
    )

    # RViz node
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", log_level],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose'),
        ],
    )

    # Include gazebo_sim.launch.py
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(pkg_gazebo_sim),
                    "launch",
                    "gazebo_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "namespace": "",
            "prefix": prefix,
            "world": world,
            "use_sim_time": use_sim_time,
            "model_package": model_package,
            "model_file": model_file,
            "use_rsp": "false",  # Disable RSP in gazebo_sim
            "use_jsp": "false",  # Disable JSP in gazebo_sim
            "use_jsp_gui": "false",  # Disable JSP GUI in gazebo_sim
            "use_joystick": use_joystick,
            "use_keyboard": use_keyboard,
            "use_navigation": use_navigation,
            "log_level": log_level,
        }.items(),
    )

    # Include sensor_preprocessor_bringup.launch.py
    sensor_preprocessor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(pkg_sensor_preproc),
                    "launch",
                    "sensor_preprocessor_bringup.launch.py",
                )
            ]
        ),
        condition=IfCondition(use_sensor_preprocessor),
        launch_arguments={
            "namespace": "",
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
                    get_package_share_directory(pkg_nav),
                    "launch",
                    "navigation_bringup.launch.py",
                )
            ]
        ),
        condition=IfCondition(use_navigation),
        launch_arguments={
            "namespace": "",
            "prefix": prefix,
            "use_sim_time": use_sim_time,
            "ekf_params": ekf_params,
            "slam_params": slam_params,
            "nav2_params": nav2_params,
            "use_ekf": use_ekf,
            "use_slam": use_slam,
            "use_nav2": use_nav2,
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            declare_namespace_arg,
            declare_prefix_arg,
            declare_world_arg,
            declare_model_package_arg,
            declare_model_file_arg,
            declare_camera_resolution_arg,
            declare_rviz_config_template_arg,
            declare_rviz_config_arg,
            declare_lidar_update_rate_arg,
            declare_sensor_preprocessor_config_arg,
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
            declare_use_rviz_config_template_arg,
            declare_use_sensor_preprocessor_arg,
            declare_use_ros2_control_arg,
            declare_use_navigation_arg,
            declare_use_ekf_arg,
            declare_use_slam_arg,
            declare_use_nav2_arg,
            declare_use_joystick_arg,
            declare_use_keyboard_arg,
            # Namespace
            push_namespace,
            # Nodes
            rsp_node,
            jsp_node,
            jsp_gui_node,
            rviz_config_generator,
            rviz_node,
            # Launchers
            sensor_preprocessor_launch,
            gazebo_sim_launch,
            navigation_launch,
        ]
    )
