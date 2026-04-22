# #!/usr/bin/env python3
# import rospy
# import math
# import pyproj
# import tf2_ros
# from geometry_msgs.msg import PoseStamped, TransformStamped
# from nav_msgs.msg import Odometry  # <--- 新增导入
# from tf.transformations import quaternion_from_euler

# try:
#     from chcnav.msg import string as NMEAMsg
# except ImportError:
#     from std_msgs.msg import String as NMEAMsg

# class GnssLocalizerNode:
#     def __init__(self):
#         rospy.init_node('gnss_localizer_node', anonymous=True)

#         # 参数配置
#         self.fixed_origin_lat = rospy.get_param('~origin_lat', 34.37334771)
#         self.fixed_origin_lon = rospy.get_param('~origin_lon', 108.89629468)
#         self.fixed_origin_alt = rospy.get_param('~origin_alt', 343.31)
#         self.use_fixed_origin = False 
        
#         self.crs_cgcs2000_geo = pyproj.CRS("EPSG:4490")
#         self.crs_cgcs2000_utm = pyproj.CRS("EPSG:4527")  # CGCS2000 / UTM zone 49N
#         self.transformer = pyproj.Transformer.from_crs(self.crs_cgcs2000_geo, self.crs_cgcs2000_utm, always_xy=True)

#         self.map_origin_x = 0.0
#         self.map_origin_y = 0.0
#         self.map_origin_z = 0.0
#         self.is_initialized = False

#         self.sub_nmea = rospy.Subscriber('/chcnav/nmea_sentence', NMEAMsg, self.nmea_callback)
        
#         # === 修改：同时发布 Pose 和 Odom ===
#         self.pub_pose = rospy.Publisher('/vehicle_pose', PoseStamped, queue_size=50) # 保留兼容
#         self.pub_odom = rospy.Publisher('/vehicle_odom', Odometry, queue_size=50)    # 新增核心话题
        
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster()

#         if self.use_fixed_origin:
#             self._set_origin(self.fixed_origin_lon, self.fixed_origin_lat, self.fixed_origin_alt)

#         rospy.loginfo("GNSS Localizer Node Started (Publishing Odometry).")

#     def _set_origin(self, lon, lat, alt):
#         x, y = self.transformer.transform(lon, lat)
#         self.map_origin_x = x
#         self.map_origin_y = y
#         self.map_origin_z = alt
#         self.is_initialized = True
#         rospy.loginfo(f"Map Origin Set: UTM E={x:.2f}, N={y:.2f}, Alt={alt:.2f}")

#     def calculate_meridian_convergence(self, lon_deg, lat_deg):
#         zone_number = int((lon_deg + 180) // 6) + 1
#         central_meridian_deg = (zone_number - 1) * 6 - 180 + 3
#         lat_rad = math.radians(lat_deg)
#         delta_lon_rad = math.radians(lon_deg - central_meridian_deg)
#         convergence_rad = math.atan(math.tan(delta_lon_rad) * math.sin(lat_rad))
#         return math.degrees(convergence_rad)

#     def nmea_callback(self, msg):
#     # """
#     # 处理 CHCNAV 的 $GPCHC NMEA 句子
#     # 解析 GNSS 位置、航向、速度信息，转换为局部坐标系下的 Pose 和 Odometry
#     # """
#         sentence = msg.sentence if hasattr(msg, 'sentence') else (msg.data if hasattr(msg, 'data') else str(msg))
        
#         # 只处理 $GPCHC 句头的数据
#         if '$GPCHC' not in sentence:
#             return

#         try:
#             parts = sentence.split(',')
#             if len(parts) < 20: 
#                 return

#             # === A. 解析基础 GNSS 数据 ===
#             raw_heading_deg = float(parts[3])   # 真北航向角 (度)
#             lat = float(parts[12])              # 纬度 (度)
#             lon = float(parts[13])              # 经度 (度)  
#             alt = float(parts[14])              # 海拔高度 (米)
            
#             # === B. 解析速度信息 ===
#             ve = float(parts[15])               # 东向速度 (m/s)
#             vn = float(parts[16])               # 北向速度 (m/s)
#             # vu = float(parts[17])             # 垂直速度 (可选)
            
#             # 计算合成线速度 (标量速度)
#             linear_velocity = math.sqrt(ve**2 + vn**2)

#             # 如果是第一次收到数据，设置地图原点
#             if not self.is_initialized:
#                 self._set_origin(lon, lat, alt)

#             # === C. 坐标转换：WGS84 -> UTM -> 局部坐标 ===
#             # 1. WGS84经纬度 -> UTM坐标
#             utm_x, utm_y = self.transformer.transform(lon, lat)
            
#             # 2. 转换为相对于原点的局部坐标 (ENU格式)
#             local_x = utm_x - self.map_origin_x    # 东向距离 (米)
#             local_y = utm_y - self.map_origin_y    # 北向距离 (米)
#             #local_z = alt - self.map_origin_z      # 垂直距离 (米)
#             local_z = 0  # 垂直距离 (米)
#             # === D. 角度修正：真北航向 -> ROS ENU yaw 角 ===
#             # 1. 计算该位置的子午线收敛角
#             convergence_deg = self.calculate_meridian_convergence(lon, lat)
            
#             # 2. 从真北航向角转换到网格北航向角
#             # Grid_Azimuth = True_North_Heading - Convergence
#             grid_azimuth_deg = raw_heading_deg - convergence_deg
            
#             # 3. 从网格北方位角转换为 ROS ENU 坐标系下的 yaw 角
#             # ROS ENU: East=0°, 逆时针为正
#             # 导航方位角: North=0°, 顺时针为正  
#             enu_yaw_deg = 90.0 - grid_azimuth_deg
            
#             # 4. 规范化到 [-180, 180] 区间
#             enu_yaw_deg = (enu_yaw_deg + 180) % 360 - 180

#             # 5. 转换为弧度并生成四元数
#             yaw_rad = math.radians(enu_yaw_deg)
#             quat = quaternion_from_euler(0, 0, yaw_rad)

#             # === E. 构造时间戳 ===
#             current_time = rospy.Time.now()

#             # === F. 发布 PoseStamped (用于可视化兼容) ===
#             pose_msg = PoseStamped()
#             pose_msg.header.stamp = current_time
#             pose_msg.header.frame_id = "world"
#             pose_msg.pose.position.x = local_x
#             pose_msg.pose.position.y = local_y
#             pose_msg.pose.position.z = local_z
#             pose_msg.pose.orientation.x = quat[0]
#             pose_msg.pose.orientation.y = quat[1]
#             pose_msg.pose.orientation.z = quat[2]
#             pose_msg.pose.orientation.w = quat[3]
#             self.pub_pose.publish(pose_msg)

#             # === G. 发布 Odometry (核心数据，包含位置和速度) ===
#             odom_msg = Odometry()
#             odom_msg.header.stamp = current_time
#             odom_msg.header.frame_id = "world"
#             odom_msg.child_frame_id = "vehicle"
            
#             # 位置信息
#             odom_msg.pose.pose = pose_msg.pose
            
#             # 速度信息 (在车身坐标系下，这里简化处理)
#             # linear.x 存储合成线速度，用于 MPC 规划
#             odom_msg.twist.twist.linear.x = linear_velocity 
            
#             self.pub_odom.publish(odom_msg)

#            # === H. 发布 TF 变换 ===
#             t = TransformStamped()
#             t.header.stamp = current_time
#             t.header.frame_id = "world"
#             t.child_frame_id = "vehicle"
#             t.transform.translation.x = local_x
#             t.transform.translation.y = local_y
#             t.transform.translation.z = local_z
#             t.transform.rotation.x = quat[0]
#             t.transform.rotation.y = quat[1]
#             t.transform.rotation.z = quat[2]
#             t.transform.rotation.w = quat[3]
#             self.tf_broadcaster.sendTransform(t)


#             # === I. 调试信息 ===
#             # rospy.loginfo(f"Raw:{raw_heading_deg:.1f}° Convergence:{convergence_deg:.2f}° Grid:{grid_azimuth_deg:.1f}° Yaw:{enu_yaw_deg:.1f}°")

#         except Exception as e:
#             rospy.logwarn_throttle(2.0, f"GPCHC Processing Error: {e}")

# if __name__ == '__main__':
#     try:
#         node = GnssLocalizerNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass



#!/usr/bin/env python3
import rospy
import math
import pyproj
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

try:
    from chcnav.msg import string as NMEAMsg
except ImportError:
    from std_msgs.msg import String as NMEAMsg

class GnssLocalizerNode:
    def __init__(self):
        rospy.init_node('gnss_localizer_node', anonymous=True)

        # 参数配置
        self.fixed_origin_lat = rospy.get_param('~origin_lat', 34.37293449)
        self.fixed_origin_lon = rospy.get_param('~origin_lon', 108.89707638)
        self.fixed_origin_alt = rospy.get_param('~origin_alt', 340.65)
        # 如果为True，以参数为原点；如果为False，以车辆启动第一帧为原点
        self.use_fixed_origin = True  
        
        self.crs_cgcs2000_geo = pyproj.CRS("EPSG:4490")
        self.crs_cgcs2000_utm = pyproj.CRS("EPSG:4527")  # CGCS2000 / UTM zone 49N
        self.transformer = pyproj.Transformer.from_crs(self.crs_cgcs2000_geo, self.crs_cgcs2000_utm, always_xy=True)

        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_z = 0.0
        self.is_initialized = False

        self.sub_nmea = rospy.Subscriber('/chcnav/nmea_sentence', NMEAMsg, self.nmea_callback)
        
        # === 话题发布 ===
        self.pub_pose = rospy.Publisher('/vehicle_pose', PoseStamped, queue_size=50)
        self.pub_odom = rospy.Publisher('/vehicle_odom', Odometry, queue_size=50)
        
        # === TF 广播器 ===
        # 1. 动态广播器：用于发布 world -> vehicle (实时更新)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # 2. 静态广播器：用于发布 world -> map (只发一次)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.has_published_map_tf = False  # 标志位：确保静态TF只发一次

        if self.use_fixed_origin:
            self._set_origin(self.fixed_origin_lon, self.fixed_origin_lat, self.fixed_origin_alt)

        rospy.loginfo("GNSS Localizer Node Started (Lego-LOAM Alignment Ready).")

    def _set_origin(self, lon, lat, alt):
        x, y = self.transformer.transform(lon, lat)
        self.map_origin_x = x
        self.map_origin_y = y
        self.map_origin_z = alt
        self.is_initialized = True
        rospy.loginfo(f"Map Origin Set: UTM E={x:.2f}, N={y:.2f}, Alt={alt:.2f}")

    def calculate_meridian_convergence(self, lon_deg, lat_deg):
        zone_number = int((lon_deg + 180) // 6) + 1
        central_meridian_deg = (zone_number - 1) * 6 - 180 + 3
        lat_rad = math.radians(lat_deg)
        delta_lon_rad = math.radians(lon_deg - central_meridian_deg)
        convergence_rad = math.atan(math.tan(delta_lon_rad) * math.sin(lat_rad))
        return math.degrees(convergence_rad)

    def nmea_callback(self, msg):
        """
        处理 CHCNAV 的 $GPCHC NMEA 句子
        解析 GNSS 位置、航向、速度信息，转换为局部坐标系下的 Pose 和 Odometry
        """
        sentence = msg.sentence if hasattr(msg, 'sentence') else (msg.data if hasattr(msg, 'data') else str(msg))
        
        if '$GPCHC' not in sentence:
            return

        try:
            parts = sentence.split(',')
            if len(parts) < 20: 
                return

            # === A. 解析基础 GNSS 数据 ===
            raw_heading_deg = float(parts[3])   # 真北航向角 (度)
            lat = float(parts[12])              # 纬度 (度)
            lon = float(parts[13])              # 经度 (度)  
            alt = float(parts[14])              # 海拔高度 (米)
            
            # === B. 解析速度信息 ===
            ve = float(parts[15])               # 东向速度 (m/s)
            vn = float(parts[16])               # 北向速度 (m/s)
            
            # 计算合成线速度 (标量速度)
            linear_velocity = math.sqrt(ve**2 + vn**2)

            # 如果是第一次收到数据，且没有设置固定原点，则将当前位置设为原点
            if not self.is_initialized:
                self._set_origin(lon, lat, alt)

            # === C. 坐标转换：WGS84 -> UTM -> 局部坐标 ===
            # 1. WGS84经纬度 -> UTM坐标
            utm_x, utm_y = self.transformer.transform(lon, lat)
            
            # 2. 转换为相对于 world 原点的局部坐标 (ENU格式)
            local_x = utm_x - self.map_origin_x    # 东向距离 (米)
            local_y = utm_y - self.map_origin_y    # 北向距离 (米)
            # local_z = alt - self.map_origin_z    # 垂直距离 (米)
            local_z = 0  # 垂直距离 (米) - 平地测试保持为0

            # === D. 角度修正：真北航向 -> ROS ENU yaw 角 ===
            # 1. 计算该位置的子午线收敛角
            convergence_deg = self.calculate_meridian_convergence(lon, lat)
            
            # 2. 从真北航向角转换到网格北航向角 (包含收敛角补偿)
            grid_azimuth_deg = raw_heading_deg - convergence_deg
            
            # 3. 从网格北方位角转换为 ROS ENU 坐标系下的 yaw 角
            enu_yaw_deg = 90.0 - grid_azimuth_deg
            
            # 4. 规范化到 [-180, 180] 区间
            enu_yaw_deg = (enu_yaw_deg + 180) % 360 - 180

            # 5. 转换为弧度并生成四元数 (这个四元数包含了所有必要的旋转修正)
            yaw_rad = math.radians(enu_yaw_deg)
            quat = quaternion_from_euler(0, 0, yaw_rad)
            current_time = rospy.Time.now()

            # ==========================================
            # === 新增逻辑：发布 world -> map 静态变换 ===
            # ==========================================
            # 逻辑：将车辆启动的第一帧位姿（包含了收敛角补偿和坐标转换）
            # 直接作为 map 坐标系在 world 坐标系下的初始姿态。
            # 这样 Lego-LOAM 的 (0,0,0) 就对齐到了车辆的实际启动位置。
            if not self.has_published_map_tf:
                static_ts = TransformStamped()
                static_ts.header.stamp = current_time
                static_ts.header.frame_id = "world"
                static_ts.child_frame_id = "map"
                
                # 使用与车辆当前位姿完全相同的计算结果
                static_ts.transform.translation.x = local_x
                static_ts.transform.translation.y = local_y
                static_ts.transform.translation.z = local_z
                static_ts.transform.rotation.x = quat[0]
                static_ts.transform.rotation.y = quat[1]
                static_ts.transform.rotation.z = quat[2]
                static_ts.transform.rotation.w = quat[3]
                
                self.static_broadcaster.sendTransform(static_ts)
                self.has_published_map_tf = True
                rospy.loginfo(f"Initialized 'map' frame at vehicle start pose. Offset: x={local_x:.2f}, y={local_y:.2f}, yaw={enu_yaw_deg:.2f}°")

            # ==========================================
            # === 原有逻辑：发布动态数据 (world -> vehicle) ===
            # ==========================================

            # === F. 发布 PoseStamped (用于可视化兼容) ===
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = local_x
            pose_msg.pose.position.y = local_y
            pose_msg.pose.position.z = local_z
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            self.pub_pose.publish(pose_msg)

            # === G. 发布 Odometry (核心数据) ===
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = "world"
            odom_msg.child_frame_id = "vehicle"
            
            # 位置信息
            odom_msg.pose.pose = pose_msg.pose
            
            # 速度信息 (在车身坐标系下)
            odom_msg.twist.twist.linear.x = linear_velocity 
            
            self.pub_odom.publish(odom_msg)

            # === H. 发布动态 TF 变换 (world -> vehicle) ===
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = "world"
            t.child_frame_id = "vehicle"
            t.transform.translation.x = local_x
            t.transform.translation.y = local_y
            t.transform.translation.z = local_z
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            rospy.logwarn_throttle(2.0, f"GPCHC Processing Error: {e}")

if __name__ == '__main__':
    try:
        node = GnssLocalizerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
