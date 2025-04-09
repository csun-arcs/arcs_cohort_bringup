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
    pkg_nav = "arcs_cohort_navigation"

    # Paths to default files
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
        get_package_share_directory(pkg_nav),
        'config',
        'ekf_params.yaml'
    )
    default_slam_params = os.path.join(
        get_package_share_directory(pkg_nav),
        'config',
        'slam_params.yaml'
    )
    default_nav2_params = os.path.join(
        get_package_share_directory(pkg_nav),
        'config',
        'nav2_params.yaml'
    )

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="Path to the world file to load",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    declare_model_package_cmd = DeclareLaunchArgument(
        "model_package",
        default_value=pkg_description,
        description="Package containing the robot model",
    )
    declare_model_file_cmd = DeclareLaunchArgument(
        "model_file",
        default_value=default_model_path,
        description="Relative path to the robot model file",
    )
    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description=(
            "Name of the robot (specifying this will add the "
            "robot name prefix to joints, links, etc. in the robot model)."
        ),
    )
    declare_camera_resolution_cmd = DeclareLaunchArgument(
        "camera_resolution",
        default_value="VGA",
        description=(
            "Resolution profile of the simulated Stereolabs Zed camera."
            'Options: "HD2K" (2208x1242), "HD1080" (1920x1080), '
            '"HD720" (1280x720) or "VGA" (672x376).'
        ),
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace under which to bring up nodes, topics, etc.",
    )
    declare_use_rsp_cmd = DeclareLaunchArgument(
        "use_rsp", default_value="true", description="Launch robot_state_publisher"
    )
    declare_use_jsp_cmd = DeclareLaunchArgument(
        "use_jsp", default_value="false", description="Launch joint_state_publisher"
    )
    declare_use_jsp_gui_cmd = DeclareLaunchArgument(
        "use_jsp_gui",
        default_value="false",
        description="Launch joint_state_publisher_gui",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz"
    )
    declare_use_rviz_config_template_cmd = DeclareLaunchArgument(
        "use_rviz_config_template",
        default_value="true",
        description="If true, generate the RViz config from the specified RViz config template.",
    )
    declare_rviz_config_template_cmd = DeclareLaunchArgument(
        "rviz_config_template",
        default_value=default_rviz_config_template_file,
        description="Path to the RViz config template file",
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config_file,
        description="Path to RViz configuration file",
    )
    declare_use_lidar_cmd = DeclareLaunchArgument(
        "use_lidar",
        default_value="false",
        description="If true, include the lidar in the robot description",
    )
    declare_lidar_update_rate_cmd = DeclareLaunchArgument(
        "lidar_update_rate",
        default_value="10",
        description="Set the update rate of the LiDAR sensor.",
    )
    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="true",
        description="Use ROS2 Control for the robot",
    )
    declare_use_navigation_cmd = DeclareLaunchArgument(
        "use_navigation",
        default_value="false",
        description="Bring up navigation stack.",
    )
    declare_ekf_params_cmd = DeclareLaunchArgument(
        "ekf_params",
        default_value=default_ekf_params,
        description="Path to the params file to load for the robot_localization package EKF node",
    )
    declare_slam_params_cmd = DeclareLaunchArgument(
        "slam_params",
        default_value=default_slam_params,
        description="Path to the params file to load for the slam_toolbox package SLAM node",
    )
    declare_nav2_params_cmd = DeclareLaunchArgument(
        "nav2_params",
        default_value=default_nav2_params,
        description="Path to the params file to load for the nav2_bringup package Nav2 bringup launcher",
    )
    declare_use_ekf_cmd = DeclareLaunchArgument(
        "use_ekf", default_value="true", description="Launch robot_localization package EKF node"
    )
    declare_use_slam_cmd = DeclareLaunchArgument(
        "use_slam", default_value="true", description="Launch slam_toolbox package SLAM node"
    )
    declare_use_nav2_cmd = DeclareLaunchArgument(
        "use_nav2", default_value="true", description="Launch nav2_bringup package Nav2 bringup launcher"
    )

    # Launch configurations
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    model_package = LaunchConfiguration("model_package")
    model_file = LaunchConfiguration("model_file")
    robot_name = LaunchConfiguration("robot_name")
    camera_resolution = LaunchConfiguration("camera_resolution")
    namespace = LaunchConfiguration("namespace")
    use_rsp = LaunchConfiguration("use_rsp")
    use_jsp = LaunchConfiguration("use_jsp")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rviz_config_template = LaunchConfiguration("use_rviz_config_template")
    rviz_config_template = LaunchConfiguration("rviz_config_template")
    rviz_config = LaunchConfiguration("rviz_config")
    use_lidar = LaunchConfiguration("use_lidar")
    lidar_update_rate = LaunchConfiguration("lidar_update_rate")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_navigation = LaunchConfiguration("use_navigation")
    ekf_params = LaunchConfiguration("ekf_params")
    slam_params = LaunchConfiguration("slam_params")
    nav2_params = LaunchConfiguration("nav2_params")
    use_ekf = LaunchConfiguration("use_ekf")
    use_slam = LaunchConfiguration("use_slam")
    use_nav2 = LaunchConfiguration("use_nav2")

    # Compute the robot prefix only if a robot name is provided
    # This expression will evaluate to, for example, "cohort_" if
    # robot_name is "cohort", or to an empty string if robot_name is empty.
    robot_prefix = PythonExpression(
        ["'", robot_name, "_' if '", robot_name, "' else ''"]
    )
    # Compute the prefix argument only if a robot_name/robot_prefix is provided.
    # This expression will evaluate to, for example, "prefix:=cohort_" if
    # robot_prefix is "cohort_", or to an empty string if robot_prefix is empty.
    robot_prefix_arg = PythonExpression(
        ["('prefix:=' + '", robot_prefix, "') if '", robot_prefix, "' else ''"]
    )

    # Robot description from Xacro, including the conditional robot name prefix.
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution([FindPackageShare(model_package), model_file]),
            " ",
            robot_prefix_arg,
            " ",
            "camera_resolution:=",
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
    push_namespace = PushRosNamespace(namespace)

    # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(use_rsp),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
        output="screen",
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
    )

    # Joint State Publisher GUI node
    jsp_gui_node = Node(
        condition=IfCondition(use_jsp_gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
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
                robot_prefix,
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
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Include gazebo_sim.launch.py
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("arcs_cohort_gazebo_sim"),
                    "launch",
                    "gazebo_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "model_package": model_package,
            "model_file": model_file,
            "use_rsp": "false",  # Disable RSP in gazebo_sim
            "use_jsp": "false",  # Disable JSP in gazebo_sim
            "use_jsp_gui": "false",  # Disable JSP GUI in gazebo_sim
        }.items(),
    )

    # Include navigation_bringup.launch.py
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("arcs_cohort_navigation"),
                    "launch",
                    "navigation_bringup.launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "ekf_params": ekf_params,
            "slam_params": slam_params,
            "nav2_params": nav2_params,
            "use_ekf": use_ekf,
            "use_slam": use_slam,
            "use_nav2": use_nav2,
        }.items(),
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            declare_world_cmd,
            declare_use_sim_time_cmd,
            declare_model_package_cmd,
            declare_model_file_cmd,
            declare_robot_name_cmd,
            declare_camera_resolution_cmd,
            declare_namespace_cmd,
            declare_use_rsp_cmd,
            declare_use_jsp_cmd,
            declare_use_jsp_gui_cmd,
            declare_use_rviz_cmd,
            declare_use_rviz_config_template_cmd,
            declare_rviz_config_template_cmd,
            declare_rviz_config_cmd,
            declare_use_lidar_cmd,
            declare_lidar_update_rate_cmd,
            declare_use_ros2_control_cmd,
            declare_use_navigation_cmd,
            declare_ekf_params_cmd,
            declare_slam_params_cmd,
            declare_nav2_params_cmd,
            declare_use_ekf_cmd,
            declare_use_slam_cmd,
            declare_use_nav2_cmd,
            # Nodes
            push_namespace,
            rsp_node,
            jsp_node,
            jsp_gui_node,
            rviz_config_generator,
            rviz_node,
            # Launchers
            gazebo_sim_launch,
            navigation_launch,
        ]
    )
