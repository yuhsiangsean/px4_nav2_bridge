"""
最小測試 launch：static TF(map->odom) + px4_odom_tf + cmd_vel_bridge + 一組精簡過的 Nav2 節點.

不含 map_server / AMCL（無地圖、無 GPS，室內用 OptiTrack 或 PX4 local frame 當 map）。
所有節點 use_sim_time:=false（見交接文件「限制與決策」）。

這裡沒有直接 include nav2_bringup 的 navigation_launch.py，是因為那份 launch
預設會啟動 collision_monitor / route_server / docking_server 三個這個專案
用不到的 lifecycle node：
  - collision_monitor 需要 /scan 才能正常工作，沒有 /scan 時它的必填陣列參數
    （polygons、observation_sources）無論留空陣列 `[]` 還是整行不寫都會讓
    rclcpp 丟 InvalidParameterValueException / "not initialized" 直接 crash，
    lifecycle_manager 因此永遠卡在 configuring，Nav2 整包起不來。
  - route_server / docking_server 是給圖形路由、自動充電對接用的，這個最小測試
    完全不需要。
拿掉這三個之後，velocity_smoother 的輸出（預設 topic 叫 cmd_vel_smoothed，
原本是給 collision_monitor 轉發成 cmd_vel）要自己 remap 成 cmd_vel，
否則 cmd_vel_bridge 訂閱的 /cmd_vel 永遠收不到東西。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# /tf、/tf_static 相對名稱重映射，跟 nav2_bringup 的 navigation_launch.py 一致
TF_REMAP = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
# controller_server / behavior_server 的原始（未平滑）速度輸出 -> cmd_vel_nav
RAW_CMD_VEL_REMAP = [('cmd_vel', 'cmd_vel_nav')]
# velocity_smoother：輸入接 cmd_vel_nav，輸出直接改名成 cmd_vel（跳過 collision_monitor）
SMOOTHER_REMAP = [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]

LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'bt_navigator',
    'waypoint_follower',
]


def generate_launch_description():
    pkg_dir = get_package_share_directory('px4_nav2_bridge')
    default_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    target_alt = LaunchConfiguration('target_alt')
    topic_odometry = LaunchConfiguration('topic_odometry')
    autostart = LaunchConfiguration('autostart')

    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='nav2_params.yaml 路徑')
    declare_target_alt = DeclareLaunchArgument(
        'target_alt', default_value='1.0',
        description='cmd_vel_bridge 的飛行高度 [m]')
    declare_topic_odometry = DeclareLaunchArgument(
        'topic_odometry', default_value='/fmu/out/vehicle_odometry',
        description='PX4 vehicle_odometry topic（訊息版本化時可能要加 _v<N> 後綴）')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='lifecycle_manager 是否自動 configure/activate')

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    px4_odom_tf = Node(
        package='px4_nav2_bridge',
        executable='px4_odom_tf',
        parameters=[{'topic_odometry': topic_odometry}],
    )

    cmd_vel_bridge = Node(
        package='px4_nav2_bridge',
        executable='cmd_vel_bridge',
        parameters=[{
            'topic_odometry': topic_odometry,
            'target_alt': target_alt,
        }],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP + RAW_CMD_VEL_REMAP,
    )
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP,
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP,
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP + RAW_CMD_VEL_REMAP,
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP,
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP,
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=TF_REMAP + SMOOTHER_REMAP,
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': autostart, 'node_names': LIFECYCLE_NODES}],
    )

    return LaunchDescription([
        declare_params_file,
        declare_target_alt,
        declare_topic_odometry,
        declare_autostart,
        static_tf_map_odom,
        px4_odom_tf,
        cmd_vel_bridge,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])
