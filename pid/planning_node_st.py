# #!/usr/bin/env python3
# import rospy
# import math
# import numpy as np
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Path, Odometry
# # 新增导入 Autoware 消息
# from autoware_msgs.msg import Lane, Waypoint 
# from tf.transformations import quaternion_from_euler

# try:
#     from trajectory_loader import load_csv_trajectory
# except ImportError:
#     rospy.logerr("Please make sure trajectory_loader.py is in the same directory.")

# class GlobalPathPublisherNode:
#     def __init__(self):
#         rospy.init_node('global_path_publisher', anonymous=True)
        
#         # --- 1. 参数设置 ---
#         self.csv_path = rospy.get_param('~csv_path', '/home/chd/catkin_ws/scripts/vehicle_data_e2w.csv')
        
#         # --- 2. 初始化发布者 ---
        
#         # [保留] 旧话题：给 RViz 显示，以及目前的 PID 测试用
#         self.pub_visual_path = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
        
#         # [新增] 新话题：Autware 格式，给下一版 PID 用 (暂时速度填0)
#         self.pub_lane = rospy.Publisher('/global_path/lane', Lane, queue_size=1, latch=True)
        
#         # [保留] 车辆位姿转发：给 RViz 显示红色箭头
#         self.pub_current_pose = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
        
#         # --- 3. 加载并发布 ---
#         rospy.loginfo("Loading trajectory from: %s", self.csv_path)
#         raw_data = load_csv_trajectory(self.csv_path, z_height=0)
        
#         if raw_data:
#             self.publish_dual_topics(raw_data)
#             rospy.loginfo("Global Path Published: /global_path (Path) & /global_path/lane (Lane)")
#         else:
#             rospy.logerr("Failed to load trajectory!")

#         # [保留] 订阅里程计
#         self.sub_odom = rospy.Subscriber('/vehicle_odom', Odometry, self._odom_callback)

#     def _odom_callback(self, msg):
#         """收到里程计，转发为 PoseStamped 给 RViz"""
#         p = PoseStamped()
#         p.header = msg.header
#         p.pose = msg.pose.pose
#         self.pub_current_pose.publish(p)

#     def publish_dual_topics(self, raw_data):
#         current_time = rospy.Time.now()
        
#         # 准备两个消息容器
#         path_msg = Path()
#         path_msg.header.stamp = current_time
#         path_msg.header.frame_id = "world"
        
#         lane_msg = Lane()
#         lane_msg.header.stamp = current_time
#         lane_msg.header.frame_id = "world"
        
#         for wp_data, cmd_speed in raw_data:
#             # 提取坐标
#             x = wp_data.transform.location.x
#             y = wp_data.transform.location.y
            
#             # 计算方向 (如果有 yaw)
#             yaw = wp_data.transform.rotation.yaw
#             yaw_rad = math.radians(yaw)
#             q = quaternion_from_euler(0, 0, yaw_rad)
            
#             # --- 构建 PoseStamped (基础单元) ---
#             pose_stamped = PoseStamped()
#             pose_stamped.header = path_msg.header
#             pose_stamped.pose.position.x = x
#             pose_stamped.pose.position.y = y
#             pose_stamped.pose.position.z = 0
#             pose_stamped.pose.orientation.x = q[0]
#             pose_stamped.pose.orientation.y = q[1]
#             pose_stamped.pose.orientation.z = q[2]
#             pose_stamped.pose.orientation.w = q[3]
            
#             # 1. 塞入 Path
#             path_msg.poses.append(pose_stamped)
            
#             # 2. 塞入 Lane
#             waypoint = Waypoint()
#             # Waypoint 里直接复用上面的 pose_stamped
#             waypoint.pose = pose_stamped 
            
#             # 【这里先按你的要求，速度暂不赋值，或者设为0】
#             # 等这一步PID跑通了，回头把这里改成 waypoint.twist.twist.linear.x = cmd_speed 即可
#             waypoint.twist.twist.linear.x = 12.0 / 3.6
            
#             lane_msg.waypoints.append(waypoint)
            
#         # 同时发布
#         self.pub_visual_path.publish(path_msg)
#         self.pub_lane.publish(lane_msg)

# if __name__ == '__main__':
#     try:
#         node = GlobalPathPublisherNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
#!/usr/bin/env python3
# import rospy
# import math
# import numpy as np
# import os
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Path, Odometry
# from autoware_msgs.msg import Lane, Waypoint
# from tf.transformations import quaternion_from_euler

# # 引入数学计算库
# from scipy.interpolate import splprep, splev
# # 引入绘图库
# import matplotlib.pyplot as plt

# try:
#     from trajectory_loader import load_csv_trajectory
# except ImportError:
#     rospy.logerr("Please make sure trajectory_loader.py is in the same directory.")

# class GlobalPathPublisherNode:
#     def __init__(self):
#         rospy.init_node('global_path_publisher', anonymous=True)
        
#         # --- 1. 参数设置 ---
#         self.csv_path = rospy.get_param('~csv_path', '/home/chd/catkin_ws/scripts/vehicle_data_ciecle.csv')
        
#         # 目标巡航速度 (km/h)
#         self.target_speed_kmh = rospy.get_param('~target_speed', 15.0) 
        
#         # 最大侧向加速度 (m/s^2) - 你的0.1是测试值，正常跑建议改回 2.0~3.0
#         self.max_lat_accel = rospy.get_param('~max_lat_accel', 0.3) 
        
#         # 是否保存调试图片
#         self.save_debug_plot = rospy.get_param('~save_debug_plot', True)
        
#         # --- 2. 初始化发布者 ---
#         self.pub_visual_path = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
#         self.pub_lane = rospy.Publisher('/global_path/lane', Lane, queue_size=1, latch=True)
#         self.pub_current_pose = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
        
#         # --- 3. 加载并处理 ---
#         rospy.loginfo(f"Loading CSV: {self.csv_path}")
#         raw_data = load_csv_trajectory(self.csv_path, z_height=0)
        
#         if raw_data:
#             self.publish_with_dynamic_speed(raw_data)
#             rospy.loginfo("Global Path Published with B-Spline Smoothed Speed Profile!")
#         else:
#             rospy.logerr("Failed to load trajectory!")

#         self.sub_odom = rospy.Subscriber('/vehicle_odom', Odometry, self._odom_callback)

#     def _odom_callback(self, msg):
#         p = PoseStamped()
#         p.header = msg.header
#         p.pose = msg.pose.pose
#         self.pub_current_pose.publish(p)

#     def calculate_curvature_speed(self, x_list, y_list):
#         """
#         [B-Spline 稳健版]
#         核心修复：在拟合前清洗数据，移除重复点（停车点），防止报错。
#         """
#         raw_count = len(x_list)
#         target_v_ms = self.target_speed_kmh / 3.6
        
#         # --- 1. 数据清洗 (关键！) ---
#         # 很多录制的 CSV 在停车时会有大量重复点，会导致 splprep 崩溃
#         clean_x = []
#         clean_y = []
        
#         if raw_count > 0:
#             clean_x.append(x_list[0])
#             clean_y.append(y_list[0])
        
#         for i in range(1, raw_count):
#             # 计算当前点和上一个清洗点的距离
#             dist = math.hypot(x_list[i] - clean_x[-1], y_list[i] - clean_y[-1])
#             # 只有距离大于 5厘米 (0.05m) 的点才保留
#             if dist > 0.05:
#                 clean_x.append(x_list[i])
#                 clean_y.append(y_list[i])
                
#         count = len(clean_x)
#         # 如果点太少，直接返回
#         if count < 4:
#             rospy.logwarn("Not enough points after filtering!")
#             return [target_v_ms] * raw_count

#         try:
#             # --- 2. B样条拟合 ---
#             # 现在数据干净了，可以用 s=0.1 甚至 s=0.0 了
#             # s=0.1 允许极微小的平滑，消除震荡
#             tck, u = splprep([clean_x, clean_y], k=3, s=0.5) 
#         except Exception as e:
#             rospy.logwarn(f"Spline fit failed ({e}), using constant speed.")
#             return [target_v_ms] * raw_count

#         # --- 3. 计算导数 ---
#         dx, dy = splev(u, tck, der=1)
#         ddx, ddy = splev(u, tck, der=2)

#         # 这里的速度是针对 clean_x 的，后面我们需要把它映射回 raw_x
#         clean_speeds = []
        
#         max_k_debug = 0.0
#         min_v_debug = 999.0

#         for i in range(count):
#             numerator = abs(dx[i] * ddy[i] - dy[i] * ddx[i])
#             denominator = (dx[i]**2 + dy[i]**2)**1.5
            
#             curvature = 0.0
#             if denominator > 1e-6:
#                 curvature = numerator / denominator
            
#             if curvature > max_k_debug: max_k_debug = curvature

#             # 计算限速
#             v_limit = 999.0
#             # 曲率阈值：半径小于 500米 才开始算
#             if curvature > 0.002:
#                 v_limit = math.sqrt(self.max_lat_accel / curvature)
            
#             final_v = min(target_v_ms, v_limit)
#             final_v = max(1.0, final_v)
            
#             if final_v < min_v_debug: min_v_debug = final_v
            
#             clean_speeds.append(final_v)

#         # --- 4. 速度平滑 ---
#         # 简单的滑动平均
#         window = 5
#         smoothed_clean_speeds = np.convolve(clean_speeds, np.ones(window)/window, mode='same')
#         smoothed_clean_speeds[0:3] = clean_speeds[0:3]
#         smoothed_clean_speeds[-3:] = clean_speeds[-3:]

#         # --- 5. 关键步骤：把清洗后的速度映射回原始点 ---
#         # 因为我们的 Lane 消息必须包含所有原始点（不能丢点）
#         # 我们用简单的“最近邻”逻辑：原始点离哪个清洗点最近，就用那个点的速度
#         final_speed_profile = []
#         clean_idx = 0
        
#         for i in range(raw_count):
#             # 找到对应的 clean 点（也就是找当前原始点属于哪一段有效路径）
#             # 因为数据是顺序的，我们只需要向后检查即可
#             curr_raw_x = x_list[i]
#             curr_raw_y = y_list[i]
            
#             # 如果不是最后一个 clean 点，且离下一个 clean 点更近，就推进索引
#             while clean_idx < count - 1:
#                 dist_curr = math.hypot(curr_raw_x - clean_x[clean_idx], curr_raw_y - clean_y[clean_idx])
#                 dist_next = math.hypot(curr_raw_x - clean_x[clean_idx+1], curr_raw_y - clean_y[clean_idx+1])
#                 if dist_next < dist_curr:
#                     clean_idx += 1
#                 else:
#                     break
            
#             final_speed_profile.append(smoothed_clean_speeds[clean_idx])

#         # 打印调试
#         rospy.loginfo(f"--- B-Spline Debug ---")
#         rospy.loginfo(f"Original Points: {raw_count} -> Filtered: {count}")
#         rospy.loginfo(f"Max Curvature: {max_k_debug:.5f}")
#         rospy.loginfo(f"Min Speed: {min_v_debug * 3.6:.2f} km/h")

#         return final_speed_profile
#         """
#         [B-Spline 修正版]
#         修改点：
#         1. s=0.0 强制拟合贴合原始点，捕捉真实弯度。
#         2. 增加 log 输出，方便终端调试。
#         """
#         count = len(x_list)
#         target_v_ms = self.target_speed_kmh / 3.6
        
#         if count < 4:
#             return [target_v_ms] * count

#         try:
#             # 【核心修改】s=0.0 表示不进行平滑，强制曲线经过每一个控制点
#             # 这样能最大程度保留原始路径的弯曲特征
#             tck, u = splprep([x_list, y_list], k=3, s=0.1) 
#         except Exception as e:
#             rospy.logwarn(f"Spline fit failed ({e}), using constant speed.")
#             return [target_v_ms] * count

#         # 计算一阶和二阶导数
#         dx, dy = splev(u, tck, der=1)
#         ddx, ddy = splev(u, tck, der=2)

#         raw_speed_profile = []
        
#         # 用于调试统计
#         min_v = 999.0
#         max_k = 0.0

#         for i in range(count):
#             # 计算曲率
#             numerator = abs(dx[i] * ddy[i] - dy[i] * ddx[i])
#             denominator = (dx[i]**2 + dy[i]**2)**1.5
            
#             curvature = 0.0
#             if denominator > 1e-6:
#                 curvature = numerator / denominator
            
#             # 记录最大曲率用于调试
#             if curvature > max_k:
#                 max_k = curvature

#             # 计算限速
#             v_limit = 999.0
#             # 【阈值微调】只要稍微有一点弯（半径小于1000米），就开始计算
#             if curvature > 1e-3: 
#                 v_limit = math.sqrt(self.max_lat_accel / curvature)
            
#             # 取最小值
#             final_v = min(target_v_ms, v_limit)
#             final_v = max(1.0, final_v)
            
#             if final_v < min_v:
#                 min_v = final_v
            
#             raw_speed_profile.append(final_v)

#         # 打印调试信息 (非常重要！)
#         rospy.loginfo(f"--- Curvature Calc Debug ---")
#         rospy.loginfo(f"Total Points: {count}")
#         rospy.loginfo(f"Max Curvature Found: {max_k:.5f}") # 如果这个数很小(比如<0.01)，说明路径被视为直线
#         rospy.loginfo(f"Min Speed Calculated: {min_v * 3.6:.2f} km/h") # 看看最低降到了多少
#         rospy.loginfo(f"Target Speed: {self.target_speed_kmh:.2f} km/h")

#         # 3. 滑动平均滤波 (平滑速度变化)
#         window_size = 5 # 窗口改小一点，避免把短暂的减速给磨平了
#         smoothed_speed = np.convolve(raw_speed_profile, np.ones(window_size)/window_size, mode='same')
        
#         # 边缘修正
#         smoothed_speed[0:3] = raw_speed_profile[0:3]
#         smoothed_speed[-3:] = raw_speed_profile[-3:]

#         return smoothed_speed
#     def _plot_debug_info(self, x_list, y_list, speed_list):
#         rospy.loginfo("Generating debug plots...")
#         dist = [0.0]
#         for i in range(1, len(x_list)):
#             d = math.sqrt((x_list[i]-x_list[i-1])**2 + (y_list[i]-y_list[i-1])**2)
#             dist.append(dist[-1] + d)
            
#         plt.figure(figsize=(12, 10))
        
#         plt.subplot(2, 1, 1)
#         sc = plt.scatter(x_list, y_list, c=speed_list, cmap='jet', s=5, label='Waypoints')
#         plt.colorbar(sc, label='Speed (m/s)')
#         plt.plot(x_list, y_list, 'k-', alpha=0.3, linewidth=1)
#         plt.plot(x_list[0], y_list[0], 'g^', markersize=12, label='Start')
#         plt.plot(x_list[-1], y_list[-1], 'rX', markersize=12, label='End')
#         plt.axis('equal')
#         plt.title(f"Global Path (ENU) - Target: {self.target_speed_kmh}km/h, MaxLatAccel: {self.max_lat_accel}")
#         plt.grid(True)
#         plt.legend()
        
#         plt.subplot(2, 1, 2)
#         plt.plot(dist, speed_list, 'b-', linewidth=2, label='Calculated Speed Limit')
#         target_ms = self.target_speed_kmh / 3.6
#         plt.axhline(y=target_ms, color='r', linestyle='--', label='Target Cruise Speed')
#         plt.title("Speed Profile vs Distance (Smoothed)")
#         plt.xlabel("Distance along path [m]")
#         plt.ylabel("Speed limit [m/s]")
#         plt.ylim(0, target_ms * 1.2)
#         plt.grid(True)
#         plt.legend()
        
#         output_filename = os.path.expanduser('~/catkin_ws/scripts/global_path_velocity_profile_spline.png')
#         plt.tight_layout()
#         plt.savefig(output_filename)
#         plt.close()
#         rospy.loginfo(f"Debug plot saved to: {output_filename}")

#     def publish_with_dynamic_speed(self, raw_data):
#         x_raw = [wp.transform.location.x for wp, _ in raw_data]
#         y_raw = [wp.transform.location.y for wp, _ in raw_data]
        
#         # 计算速度
#         calculated_speeds = self.calculate_curvature_speed(x_raw, y_raw)
        
#         if self.save_debug_plot:
#             self._plot_debug_info(x_raw, y_raw, calculated_speeds)
        
#         current_time = rospy.Time.now()
#         path_msg = Path()
#         path_msg.header.stamp = current_time
#         path_msg.header.frame_id = "world"
        
#         lane_msg = Lane()
#         lane_msg.header.stamp = current_time
#         lane_msg.header.frame_id = "world"
        
#         for i, (wp_data, _) in enumerate(raw_data):
#             x = wp_data.transform.location.x
#             y = wp_data.transform.location.y
#             yaw = wp_data.transform.rotation.yaw
#             yaw_rad = math.radians(yaw)
            
#             dynamic_speed = calculated_speeds[i]
            
#             q = quaternion_from_euler(0, 0, yaw_rad)
#             pose_stamped = PoseStamped()
#             pose_stamped.header = path_msg.header
#             pose_stamped.pose.position.x = x
#             pose_stamped.pose.position.y = y
#             pose_stamped.pose.position.z = 0
#             pose_stamped.pose.orientation.x = q[0]
#             pose_stamped.pose.orientation.y = q[1]
#             pose_stamped.pose.orientation.z = q[2]
#             pose_stamped.pose.orientation.w = q[3]
#             path_msg.poses.append(pose_stamped)
            
#             waypoint = Waypoint()
#             waypoint.pose = pose_stamped
#             waypoint.twist.twist.linear.x = float(dynamic_speed)
#             lane_msg.waypoints.append(waypoint)
            
#         self.pub_visual_path.publish(path_msg)
#         self.pub_lane.publish(lane_msg)

# if __name__ == '__main__':
#     try:
#         node = GlobalPathPublisherNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

# #!/usr/bin/env python3
# import rospy
# import math
# import numpy as np
# import os
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Path, Odometry
# from autoware_msgs.msg import Lane, Waypoint
# from tf.transformations import quaternion_from_euler

# # 引入数学计算库
# from scipy.interpolate import splprep, splev
# # 引入绘图库
# import matplotlib.pyplot as plt

# try:
#     from trajectory_loader import load_csv_trajectory
# except ImportError:
#     rospy.logerr("Please make sure trajectory_loader.py is in the same directory.")

# class GlobalPathPublisherNode:
#     def __init__(self):
#         rospy.init_node('global_path_publisher', anonymous=True)
        
#         # --- 1. 参数设置 ---
#         self.csv_path = rospy.get_param('~csv_path', '/home/chd/catkin_ws/scripts/vehicle_data_ciecle_you.csv')
        
#         # 目标巡航速度 (km/h)
#         self.target_speed_kmh = rospy.get_param('~target_speed', 12.0) 
        
#         # 最大侧向加速度 (m/s^2) -> 弯道限速
#         self.max_lat_accel = rospy.get_param('~max_lat_accel', 0.3) 
        
#         # [新增] 最大纵向加速度 (m/s^2) -> 起步和加速限制
#         self.max_accel = rospy.get_param('~max_accel', 0.3)
        
#         # [新增] 最大纵向减速度 (m/s^2) -> 刹车限制
#         self.max_decel = rospy.get_param('~max_decel', 0.3)
        
#         # 是否保存调试图片
#         self.save_debug_plot = rospy.get_param('~save_debug_plot', True)
        
#         # --- 2. 初始化发布者 ---
#         self.pub_visual_path = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
#         self.pub_lane = rospy.Publisher('/global_path/lane', Lane, queue_size=1, latch=True)
#         self.pub_current_pose = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
        
#         # --- 3. 加载并处理 ---
#         rospy.loginfo(f"Loading CSV: {self.csv_path}")
#         raw_data = load_csv_trajectory(self.csv_path, z_height=0)
        
#         if raw_data:
#             self.publish_with_dynamic_speed(raw_data)
#             rospy.loginfo("Global Path Published with Kinematically Smoothed Profile!")
#         else:
#             rospy.logerr("Failed to load trajectory!")

#         self.sub_odom = rospy.Subscriber('/vehicle_odom', Odometry, self._odom_callback)

#     def _odom_callback(self, msg):
#         p = PoseStamped()
#         p.header = msg.header
#         p.pose = msg.pose.pose
#         self.pub_current_pose.publish(p)

#     def apply_physical_smoothing(self, speed_profile, x_list, y_list):
#         """
#         [物理平滑算法]
#         根据 max_accel 和 max_decel 限制速度变化率。
#         公式：v_end^2 = v_start^2 + 2 * a * d
#         """
#         count = len(speed_profile)
#         processed_speed = list(speed_profile)
        
#         # 1. 反向扫描 (Backward Pass) - 处理刹车
#         # 确保车子能在到达低速点之前提前减速
#         for i in range(count - 2, -1, -1):
#             dist = math.hypot(x_list[i+1] - x_list[i], y_list[i+1] - y_list[i])
#             # v_max = sqrt(v_next^2 + 2 * a_decel * dist)
#             # 意思是：如果下一点要求速度是 v_next，那我当前点最高只能跑 v_max，否则刹不住
#             allowed_v = math.sqrt(processed_speed[i+1]**2 + 2 * self.max_decel * dist)
#             processed_speed[i] = min(processed_speed[i], allowed_v)

#         # 2. 正向扫描 (Forward Pass) - 处理加速
#         # 确保车子不会加速得太离谱
#         for i in range(count - 1):
#             dist = math.hypot(x_list[i+1] - x_list[i], y_list[i+1] - y_list[i])
#             # v_max = sqrt(v_curr^2 + 2 * a_accel * dist)
#             allowed_v = math.sqrt(processed_speed[i]**2 + 2 * self.max_accel * dist)
#             processed_speed[i+1] = min(processed_speed[i+1], allowed_v)
            
#         return processed_speed

#     def calculate_curvature_speed(self, x_list, y_list):
#         raw_count = len(x_list)
#         target_v_ms = self.target_speed_kmh / 3.6
        
#         # --- 1. 数据清洗 ---
#         clean_x = []
#         clean_y = []
#         if raw_count > 0:
#             clean_x.append(x_list[0])
#             clean_y.append(y_list[0])
#         for i in range(1, raw_count):
#             dist = math.hypot(x_list[i] - clean_x[-1], y_list[i] - clean_y[-1])
#             if dist > 0.05:
#                 clean_x.append(x_list[i])
#                 clean_y.append(y_list[i])
        
#         count = len(clean_x)
#         if count < 4:
#             return [target_v_ms] * raw_count

#         try:
#             # --- 2. B样条拟合 (计算曲率限速) ---
#             tck, u = splprep([clean_x, clean_y], k=3, s=0.1) 
#         except Exception as e:
#             rospy.logwarn(f"Spline fit failed ({e}), using constant speed.")
#             return [target_v_ms] * raw_count

#         dx, dy = splev(u, tck, der=1)
#         ddx, ddy = splev(u, tck, der=2)

#         curvature_speeds = []
#         for i in range(count):
#             numerator = abs(dx[i] * ddy[i] - dy[i] * ddx[i])
#             denominator = (dx[i]**2 + dy[i]**2)**1.5
#             curvature = 0.0 if denominator <= 1e-6 else numerator / denominator
            
#             v_limit = 999.0
#             if curvature > 0.002:
#                 v_limit = math.sqrt(self.max_lat_accel / curvature)
            
#             final_v = min(target_v_ms, v_limit)
#             final_v = max(1.0, final_v)
#             curvature_speeds.append(final_v)

#         # --- 3. [关键新增] 纵向物理平滑 ---
#         # 这一步把“瞬时跳变”的速度曲线，变成了符合物理规律的梯形加减速曲线
#         physically_smoothed_speeds = self.apply_physical_smoothing(curvature_speeds, clean_x, clean_y)

#         # --- 4. 映射回原始点 ---
#         final_speed_profile = []
#         clean_idx = 0
#         for i in range(raw_count):
#             curr_raw_x = x_list[i]
#             curr_raw_y = y_list[i]
#             while clean_idx < count - 1:
#                 dist_curr = math.hypot(curr_raw_x - clean_x[clean_idx], curr_raw_y - clean_y[clean_idx])
#                 dist_next = math.hypot(curr_raw_x - clean_x[clean_idx+1], curr_raw_y - clean_y[clean_idx+1])
#                 if dist_next < dist_curr:
#                     clean_idx += 1
#                 else:
#                     break
#             final_speed_profile.append(physically_smoothed_speeds[clean_idx])

#         return final_speed_profile

#     def _plot_debug_info(self, x_list, y_list, speed_list):
#         rospy.loginfo("Generating debug plots...")
#         dist = [0.0]
#         for i in range(1, len(x_list)):
#             d = math.sqrt((x_list[i]-x_list[i-1])**2 + (y_list[i]-y_list[i-1])**2)
#             dist.append(dist[-1] + d)
            
#         plt.figure(figsize=(12, 10))
        
#         plt.subplot(2, 1, 1)
#         sc = plt.scatter(x_list, y_list, c=speed_list, cmap='jet', s=5, label='Waypoints')
#         plt.colorbar(sc, label='Speed (m/s)')
#         plt.plot(x_list, y_list, 'k-', alpha=0.3, linewidth=1)
#         plt.plot(x_list[0], y_list[0], 'g^', markersize=12, label='Start')
#         plt.plot(x_list[-1], y_list[-1], 'rX', markersize=12, label='End')
#         plt.axis('equal')
#         plt.title(f"Global Path (ENU) - Target: {self.target_speed_kmh}km/h\nLatAccel: {self.max_lat_accel}, LonAccel/Decel: {self.max_accel}")
#         plt.grid(True)
#         plt.legend()
        
#         plt.subplot(2, 1, 2)
#         plt.plot(dist, speed_list, 'b-', linewidth=2, label='Final Kinematic Speed Profile')
#         target_ms = self.target_speed_kmh / 3.6
#         plt.axhline(y=target_ms, color='r', linestyle='--', label='Target Cruise Speed')
#         plt.title("Speed Profile vs Distance (Kinematically Smoothed)")
#         plt.xlabel("Distance along path [m]")
#         plt.ylabel("Speed limit [m/s]")
#         plt.ylim(0, target_ms * 1.2)
#         plt.grid(True)
#         plt.legend()
        
#         output_filename = os.path.expanduser('~/catkin_ws/scripts/global_path_kinematic_profile.png')
#         plt.tight_layout()
#         plt.savefig(output_filename)
#         plt.close()
#         rospy.loginfo(f"Debug plot saved to: {output_filename}")

#     def publish_with_dynamic_speed(self, raw_data):
#         x_raw = [wp.transform.location.x for wp, _ in raw_data]
#         y_raw = [wp.transform.location.y for wp, _ in raw_data]
        
#         calculated_speeds = self.calculate_curvature_speed(x_raw, y_raw)
        
#         if self.save_debug_plot:
#             self._plot_debug_info(x_raw, y_raw, calculated_speeds)
        
#         current_time = rospy.Time.now()
#         path_msg = Path()
#         path_msg.header.stamp = current_time
#         path_msg.header.frame_id = "world"
#         lane_msg = Lane()
#         lane_msg.header.stamp = current_time
#         lane_msg.header.frame_id = "world"
        
#         for i, (wp_data, _) in enumerate(raw_data):
#             x = wp_data.transform.location.x
#             y = wp_data.transform.location.y
#             yaw = wp_data.transform.rotation.yaw
#             yaw_rad = math.radians(yaw)
            
#             dynamic_speed = calculated_speeds[i]
            
#             q = quaternion_from_euler(0, 0, yaw_rad)
#             pose_stamped = PoseStamped()
#             pose_stamped.header = path_msg.header
#             pose_stamped.pose.position.x = x
#             pose_stamped.pose.position.y = y
#             pose_stamped.pose.position.z = 0
#             pose_stamped.pose.orientation.x = q[0]
#             pose_stamped.pose.orientation.y = q[1]
#             pose_stamped.pose.orientation.z = q[2]
#             pose_stamped.pose.orientation.w = q[3]
#             path_msg.poses.append(pose_stamped)
            
#             waypoint = Waypoint()
#             waypoint.pose = pose_stamped
#             waypoint.twist.twist.linear.x = float(dynamic_speed)
#             lane_msg.waypoints.append(waypoint)
            
#         self.pub_visual_path.publish(path_msg)
#         self.pub_lane.publish(lane_msg)

# if __name__ == '__main__':
#     try:
#         node = GlobalPathPublisherNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass



#!/usr/bin/env python3
import rospy
import math
import numpy as np
import os
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from autoware_msgs.msg import Lane, Waypoint
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
# 引入数学计算库
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

try:
    from trajectory_loader import load_csv_trajectory
except ImportError:
    rospy.logerr("Please make sure trajectory_loader.py is in the same directory.")

class GlobalPathPublisherNode:
    def __init__(self):
        rospy.init_node('global_path_publisher', anonymous=True)
        
        # --- 1. 参数设置 ---
        self.csv_path = rospy.get_param('~csv_path', '/home/chd/catkin_ws/scripts/vehicle_data_odom_shun.csv')
        # self.csv_path = rospy.get_param('~csv_path', '/home/chd/catkin_ws/scripts/vehicle_data_odom_ni.csv')

        self.target_speed_kmh = rospy.get_param('~target_speed', 18.0) 
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 0.22) #0.2
        self.max_accel = rospy.get_param('~max_accel', 0.3)
        self.max_decel = rospy.get_param('~max_decel', 0.25)
        
        # [核心参数] 重采样分辨率 (每隔 0.2米 生成一个新点)
        self.sampling_resolution = 0.2 
        
        self.save_debug_plot = True
        
        # --- 2. 初始化发布者 ---
        self.pub_visual_path = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
        self.pub_lane = rospy.Publisher('/mpc/reference_path', Lane, queue_size=1, latch=True)
        self.pub_current_pose = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
        # 新增：用于显示选点的 Marker
        self.pub_marker = rospy.Publisher('/path_selection_marker', Marker, queue_size=10)
        # --- 3. 订阅者 (支持 Rviz 选点) ---
        rospy.Subscriber('/vehicle_odom', Odometry, self._odom_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self._start_callback) # Rviz 2D Pose Estimate
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._goal_callback)      # Rviz 2D Nav Goal

        # --- 4. 内部状态 ---
        self.full_path_x = [] # 存储生成的全量新地图 X
        self.full_path_y = [] # 存储生成的全量新地图 Y
        self.full_path_v = [] # 存储生成的全量新地图 速度
        
        self.start_idx = 0
        self.end_idx = -1
        
        # --- 5. 加载并生成地图 ---
        rospy.loginfo(f"Loading CSV: {self.csv_path}")
        raw_data = load_csv_trajectory(self.csv_path, z_height=0)
        
        if raw_data:
            # 第一步：把原始乱七八糟的点，变成完美的等间距点
            self.generate_new_map(raw_data)
            # 第二步：默认发布全图
            self.publish_path()
        else:
            rospy.logerr("Failed to load trajectory!")
        rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_path())

    def _odom_callback(self, msg):
        p = PoseStamped()
        p.header = msg.header
        p.pose = msg.pose.pose
        self.pub_current_pose.publish(p)

    def _start_callback(self, msg):
        if not self.full_path_x: return
        # 找最近点
        self.start_idx = self.find_closest_index(msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        # === 新增：打印距离并可视化 ===
        sel_x = self.full_path_x[self.start_idx]
        sel_y = self.full_path_y[self.start_idx]
        rospy.loginfo(f"【起点设定】点击位置吸附到 Index: {self.start_idx}")
        self.publish_selection_marker(sel_x, sel_y, 0) # 画绿球
        
        self.publish_path()

    def _goal_callback(self, msg):
        if not self.full_path_x: return
        # 找最近点
        self.end_idx = self.find_closest_index(msg.pose.position.x, msg.pose.position.y)
        
        # === 新增：打印距离并可视化 ===
        sel_x = self.full_path_x[self.end_idx]
        sel_y = self.full_path_y[self.end_idx]
        rospy.loginfo(f"【终点设定】点击位置吸附到 Index: {self.end_idx}")
        self.publish_selection_marker(sel_x, sel_y, 1) # 画红球
        
        self.publish_path()

    def publish_selection_marker(self, x, y, type_id):
            """
            在 Rviz 上画一个球，告诉你系统到底选中了哪个点
            type_id: 0=起点(绿色), 1=终点(红色)
            """
            marker = Marker()
            marker.header.frame_id = "world" # 确保跟你的路径 frame 一致
            marker.header.stamp = rospy.Time.now()
            marker.ns = "selection"
            marker.id = type_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 1.0 # 稍微悬空一点，防遮挡
            marker.scale.x = 2.0 # 球的大小，设大一点显眼
            marker.scale.y = 2.0
            marker.scale.z = 2.0
            
            if type_id == 0: # 起点是绿色
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else: # 终点是红色
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                
            self.pub_marker.publish(marker)

    def find_closest_index(self, x, y):
        min_d = float('inf')
        idx = 0
        for i in range(len(self.full_path_x)):
            d = (x - self.full_path_x[i])**2 + (y - self.full_path_y[i])**2
            if d < min_d:
                min_d = d
                idx = i
        return idx

    def generate_new_map(self, raw_data):
        """
        [移植自 MPCLocalPlanner]
        拟合 -> 重采样 -> 计算速度 -> 纵向平滑
        """
        x_raw = [wp.transform.location.x for wp, _ in raw_data]
        y_raw = [wp.transform.location.y for wp, _ in raw_data]
        
        # 1. 去重 (Clean)
        clean_x, clean_y = [x_raw[0]], [y_raw[0]]
        for i in range(1, len(x_raw)):
            if math.hypot(x_raw[i]-clean_x[-1], y_raw[i]-clean_y[-1]) > 0.05:
                clean_x.append(x_raw[i])
                clean_y.append(y_raw[i])
                
        # 2. B样条拟合 (Fit)
        # s=1.0: 允许一定程度的平滑，既然我们要重新生成点，就让它滑一点
        try:
            tck, u = splprep([clean_x, clean_y], k=3, s=1.0)
        except Exception as e:
            rospy.logerr(f"Spline Fit Failed: {e}")
            return

        # 3. 计算总长度并重采样 (Resample)
        # 先用密集点估算长度
        u_temp = np.linspace(0, 1, 10000)
        x_temp, y_temp = splev(u_temp, tck)
        total_len = 0
        for i in range(1, len(x_temp)):
            total_len += math.hypot(x_temp[i]-x_temp[i-1], y_temp[i]-y_temp[i-1])
        
        # 决定生成多少个新点
        num_points = int(total_len / self.sampling_resolution)
        u_new = np.linspace(0, 1, num_points)
        
        # 得到全新的坐标
        new_x, new_y = splev(u_new, tck)
        
        # 4. 计算速度 (Calculate Speed)
        dx, dy = splev(u_new, tck, der=1)
        ddx, ddy = splev(u_new, tck, der=2)
        target_v_ms = self.target_speed_kmh / 3.6
        raw_speeds = []
        
        for i in range(num_points):
            k_num = abs(dx[i]*ddy[i] - dy[i]*ddx[i])
            k_den = (dx[i]**2 + dy[i]**2)**1.5
            k = k_num / k_den if k_den > 1e-6 else 0.0
            
            v = target_v_ms
            if k > 0.002:
                v = math.sqrt(self.max_lat_accel / k)
            
            raw_speeds.append(min(target_v_ms, v))
        # 强制起终点速度为 0
        if raw_speeds:
            raw_speeds[0] = 0.0   # 起步从 0 开始 
            raw_speeds[-1] = 0.5  # 到终点停车   
        # 5. 纵向平滑 (Smooth)
        # 借用之前的逻辑，防止急刹车
        final_speeds = self.apply_physical_smoothing(raw_speeds, new_x, new_y)
        
        # 存入全局变量
        self.full_path_x = new_x.tolist() 
        self.full_path_y = new_y.tolist()
        self.full_path_v = final_speeds
        self.end_idx = len(new_x) - 1 # 默认终点是最后
        
        rospy.loginfo(f"地图生成完毕！重采样后点数: {len(new_x)} (分辨率 {self.sampling_resolution}m)")
        
        if self.save_debug_plot:
            self._plot_debug_info(new_x, new_y, final_speeds)

    def apply_physical_smoothing(self, speeds, x, y):
        # 简单的双向物理平滑
        processed = list(speeds)
        count = len(speeds)
        # 反向 (刹车限制)
        for i in range(count-2, -1, -1):
            d = math.hypot(x[i+1]-x[i], y[i+1]-y[i])
            v_lim = math.sqrt(processed[i+1]**2 + 2*self.max_decel*d)
            processed[i] = min(processed[i], v_lim)
        # 正向 (加速限制)
        for i in range(count-1):
            d = math.hypot(x[i+1]-x[i], y[i+1]-y[i])
            v_lim = math.sqrt(processed[i]**2 + 2*self.max_accel*d)
            processed[i+1] = min(processed[i+1], v_lim)
        return processed

    def publish_path(self):
        if not self.full_path_x: return
        
        # === 核心逻辑：截取路径 ===
        # 支持闭环截取 (如果 end < start，说明跨圈了)
        total = len(self.full_path_x)
        indices = []
        
        if self.end_idx >= self.start_idx:
            # 正常截取
            indices = range(self.start_idx, self.end_idx + 1)
        else:
            # 跨圈截取 (比如起点3000，终点100)
            indices = list(range(self.start_idx, total)) + list(range(0, self.end_idx + 1))
            
        # 组装消息
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "world"
        
        lane_msg = Lane()
        lane_msg.header = path_msg.header
        
        for idx in indices:
            # 安全检查
            if idx >= total: continue 
            
            x = self.full_path_x[idx]
            y = self.full_path_y[idx]
            v = self.full_path_v[idx]
            
            # 计算朝向 (简单起见，指向下一个点)
            next_i = (idx + 1) % total
            yaw = math.atan2(self.full_path_y[next_i] - y, self.full_path_x[next_i] - x)
            q = quaternion_from_euler(0, 0, yaw)
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path_msg.poses.append(pose)
            
            wp = Waypoint()
            wp.pose = pose
            wp.twist.twist.linear.x = float(v)
            # --- 新增：补齐走廊边界数据 ---
            wp.twist.twist.angular.y = 2.5  # 左安全走廊边界 (m)
            wp.twist.twist.angular.z = 2.5  # 右安全走廊边界 (m)
            lane_msg.waypoints.append(wp)
            
        self.pub_visual_path.publish(path_msg)
        self.pub_lane.publish(lane_msg)
        # rospy.loginfo(f"已发布路径: {len(indices)} 个点 (Start: {self.start_idx}, End: {self.end_idx})")

    def _plot_debug_info(self, x_list, y_list, speed_list):
        rospy.loginfo("Generating debug plots...")
        
        # 1. 计算累计距离 (为了画速度-距离图)
        dist = [0.0]
        for i in range(1, len(x_list)):
            d = math.sqrt((x_list[i]-x_list[i-1])**2 + (y_list[i]-y_list[i-1])**2)
            dist.append(dist[-1] + d)
            
        plt.figure(figsize=(12, 10))
        
        # === 子图 1: 全局路径 XY 图 (带速度颜色) ===
        plt.subplot(2, 1, 1)
        # s=5 是点的大小，cmap='jet' 是蓝-红热力图
        sc = plt.scatter(x_list, y_list, c=speed_list, cmap='jet', s=5, label='Waypoints')
        plt.colorbar(sc, label='Speed (m/s)')
        
        # 画连线和起终点
        plt.plot(x_list, y_list, 'k-', alpha=0.3, linewidth=1)
        plt.plot(x_list[0], y_list[0], 'g^', markersize=12, label='Start') # 绿色三角是起点
        plt.plot(x_list[-1], y_list[-1], 'rX', markersize=12, label='End') # 红色叉叉是终点
        
        plt.axis('equal')
        plt.title(f"Global Path (ENU) - Target: {self.target_speed_kmh}km/h\nLatAccel: {self.max_lat_accel}, LonAccel: {self.max_accel}")
        plt.grid(True)
        plt.legend()
        
        # === 子图 2: 速度-距离 曲线图 (核心调试图) ===
        plt.subplot(2, 1, 2)
        plt.plot(dist, speed_list, 'b-', linewidth=2, label='Final Kinematic Speed Profile')
        
        # 画一条红色的虚线表示目标最高速
        target_ms = self.target_speed_kmh / 3.6
        plt.axhline(y=target_ms, color='r', linestyle='--', label='Target Cruise Speed')
        
        plt.title("Speed Profile vs Distance (Kinematically Smoothed)")
        plt.xlabel("Distance along path [m]")
        plt.ylabel("Speed limit [m/s]")
        plt.ylim(0, target_ms * 1.2) # Y轴留一点余量
        plt.grid(True)
        plt.legend()
        
        # 保存路径
        output_filename = os.path.expanduser('~/catkin_ws/scripts/global_path_kinematic_profile.png')
        plt.tight_layout()
        plt.savefig(output_filename)
        plt.close() # 这一步很重要，防止内存泄漏
        
        rospy.loginfo(f"Debug plot saved to: {output_filename}")

if __name__ == '__main__':
    try:
        GlobalPathPublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
