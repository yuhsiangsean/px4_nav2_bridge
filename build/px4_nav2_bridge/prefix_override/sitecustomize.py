import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/ros2_ws/src/px4_nav2_bridge/install/px4_nav2_bridge'
