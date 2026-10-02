# 模擬端指令

## 終端機 1  PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gz_x500

## 終端機 2 uXRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

## 終端機 3 開nav2
ros2 launch px4_nav2_bridge bridge_launch.py \
  topic_odometry:=/MAV4/fmu/out/vehicle_odometry \
  topic_offboard_mode:=/MAV4/fmu/in/offboard_control_mode \
  topic_setpoint:=/MAV4/fmu/in/trajectory_setpoint \
  topic_command:=/MAV4/fmu/in/vehicle_command \
  target_alt:=0.5 2>&1 | tee ~/nav2_test.log

## 終端機  起飛
ros2 service call /cmd_vel_bridge/start std_srvs/srv/Trigger

## 往目標點移動 +x:0.5
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}}}" --feedback

## 降落 
ros2 service call /cmd_vel_bridge/land std_srvs/srv/Trigger

