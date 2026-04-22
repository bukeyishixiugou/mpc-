#!/usr/bin/env python3
# encoding: utf-8

import rospy
import math
import numpy as np
import sys
import select
import termios
import tty
from nav_msgs.msg import Odometry
from autoware_msgs.msg import AccelCmd, BrakeCmd, SteerCmd, Lane 
from tf.transformations import euler_from_quaternion

def getKey(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key.lower()

class PIDRunnerNode:
    def __init__(self):
        rospy.init_node('pid_runner_node', anonymous=True)
        
        self.is_running = True 
        self.goal_reached = False 
        self.last_c_idx = None # 初始化
        # === 1. 频率设置 ===
        self.control_rate = 100  
        self.dt = 1.0 / self.control_rate
        
        # === 2. 安全限位 ===
        self.limit_steer_cmd = 100  # 假设方向盘最大范围 100%
        self.limit_accel_cmd = 15   # 稍微放开一点油门上限
        self.limit_brake_cmd = 20  

        # === 3. 平滑参数 ===
        self.max_steer_step = rospy.get_param('~max_steer_step', 8) # 稍微改大，让转向反应快一点
        self.max_accel_step = rospy.get_param('~max_accel_step', 2)
        self.max_brake_step = rospy.get_param('~max_brake_step', 3)

        # === 4. 纵向 PID 参数 (速度控制) ===
        self.kp_vel = rospy.get_param('~kp_vel', 0.1) 
        self.ki_vel = rospy.get_param('~ki_vel', 0.1)
        self.kd_vel = rospy.get_param('~kd_vel', 0.18)
        
        # === 5. 横向 PID 参数 (转向控制 - 核心修改) ===
        self.kp_lat = rospy.get_param('~kp_lat', 0.04) 
        # 新增积分项：专门消除"过弯偏离中心"的稳态误差
        self.ki_lat = rospy.get_param('~ki_lat', 0.02) 
        # 新增微分项：防止转向过猛画龙
        self.kd_lat = rospy.get_param('~kd_lat', 0.16) 
        
        self.kp_head = rospy.get_param('~kp_head', 1.0) # 航向误差保持不变
        
        # === 6. 动态前视距离参数 ===
        # 逻辑：look_ahead = min_dist + velocity * speed_ratio
        self.min_look_ahead = 3.0 
        self.look_ahead_ratio = 0 # 速度越快看得越远
        
        self.goal_dist_threshold = rospy.get_param('~goal_dist_threshold', 0.1)

        self.max_steer_deg = 40.0
        self.max_accel = 1.0
        self.max_decel = 1.5

        # === 7. 内部状态 ===
        self.current_pose = None
        self.current_v = 0.0
        self.current_yaw = 0.0
        self.global_path = None 
        
        # 纵向积分误差
        self.vel_integral = 0.0
        self.prev_vel_error = 0.0
        
        # [新增] 横向积分/微分误差
        self.lat_integral = 0.0
        self.prev_lat_error = 0.0
        
        self.prev_steer_cmd = 0
        self.prev_accel_cmd = 0
        self.prev_brake_cmd = 0
        
        rospy.Subscriber('/vehicle_odom', Odometry, self.odom_callback)
        rospy.Subscriber('/global_path/lane', Lane, self.path_callback)
        
        self.pub_accel = rospy.Publisher('/accel_cmd', AccelCmd, queue_size=1)
        self.pub_brake = rospy.Publisher('/brake_cmd', BrakeCmd, queue_size=1)
        self.pub_steer = rospy.Publisher('/steer_cmd', SteerCmd, queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        
        rospy.loginfo(f"PID Runner Started. Kp_lat={self.kp_lat}, Ki_lat={self.ki_lat}")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        self.current_v = msg.twist.twist.linear.x
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])

    def path_callback(self, msg):
        self.global_path = msg.waypoints

    def check_goal_reached(self):
        if not self.global_path or not self.current_pose:
            return False
        last_pose = self.global_path[-1].pose.pose.position 
        dx = self.current_pose.x - last_pose.x
        dy = self.current_pose.y - last_pose.y
        dist = math.sqrt(dx**2 + dy**2)
        return dist < self.goal_dist_threshold

    def get_dynamic_lookahead(self):
        """计算动态前视距离"""
        # 速度越快，看得越远；速度慢（过弯），看近一点以获得更好切弯效果
        dist = self.min_look_ahead + max(0, self.current_v) * self.look_ahead_ratio
        return min(dist, 8.0) # 上限 8米

    def find_target(self):
        if not self.current_pose or not self.global_path: return None, None
        
        min_dist = float('inf')
        c_idx = -1
        cx, cy = self.current_pose.x, self.current_pose.y
        step = 2 
        
        # === 简单粗暴：每次都全图搜索 ===
        # 既然只有4000个点，全搜是最稳的，绝对不会因为“上一次索引”导致找不到
        for i in range(0, len(self.global_path), step):
            p = self.global_path[i].pose.pose.position
            d = (cx - p.x)**2 + (cy - p.y)**2
            if d < min_dist:
                min_dist = d
                c_idx = i
        
        # 只有真的找遍全图都没找到（逻辑上不可能），才返回 None
        if c_idx == -1: 
            rospy.logwarn_throttle(1, "PID: 警告 - 无法在路径上找到最近点！")
            return None, None
            
        t_idx = c_idx
        dist_acc = 0
        current_look_ahead = self.get_dynamic_lookahead()
        
        while t_idx < len(self.global_path)-1 and dist_acc < current_look_ahead:
            p1 = self.global_path[t_idx].pose.pose.position
            p2 = self.global_path[t_idx+1].pose.pose.position
            dist_acc += math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2)
            t_idx += 1
            
        return c_idx, t_idx

    def _smooth_command(self, target, current, max_step):
        diff = target - current
        clipped_diff = np.clip(diff, -max_step, max_step)
        return int(current + clipped_diff)

    def smooth_stop(self):
        self.is_running = False 
        target_brake = 30 
        current_brake = 0
        rate = rospy.Rate(20) 
        while current_brake < target_brake and not rospy.is_shutdown():
            current_brake += 1.0 
            self.pub_accel.publish(AccelCmd(accel=0))
            self.pub_brake.publish(BrakeCmd(brake=int(current_brake)))
            self.pub_steer.publish(SteerCmd(steer=self.prev_steer_cmd))
            rate.sleep()
        rospy.sleep(1.0)
        self.pub_accel.publish(AccelCmd(accel=0))
        self.pub_brake.publish(BrakeCmd(brake=0))
        self.pub_steer.publish(SteerCmd(steer=0))
        rospy.loginfo("停止完成。")

    def control_loop(self, event):
        if not self.is_running: return
        if not self.current_pose or not self.global_path: return
        
        if self.check_goal_reached():
            if not self.goal_reached: 
                rospy.loginfo(f"=== 到达终点 ===")
                self.goal_reached = True
                self.smooth_stop() 
            return

        c_idx, t_idx = self.find_target()
        if c_idx is None: return
        
        # === 1. 计算横向误差 (Lateral Error) ===
        p1 = self.global_path[c_idx].pose.pose.position
        p2 = self.global_path[min(c_idx+1, len(self.global_path)-1)].pose.pose.position
        path_yaw = math.atan2(p2.y - p1.y, p2.x - p1.x)
        dx = self.current_pose.x - p1.x
        dy = self.current_pose.y - p1.y
        
        # 计算该点相对于路径的横向距离
        lat_err = -dx * math.sin(path_yaw) + dy * math.cos(path_yaw)
        
        # === 2. 计算航向误差 (Heading Error) ===
        tp1 = self.global_path[t_idx].pose.pose.position
        tp2 = self.global_path[min(t_idx+1, len(self.global_path)-1)].pose.pose.position
        target_yaw = math.atan2(tp2.y - tp1.y, tp2.x - tp1.x)
        
        head_err = self.current_yaw - target_yaw
        while head_err > math.pi: head_err -= 2*math.pi
        while head_err < -math.pi: head_err += 2*math.pi
        
        # === 3. 横向 PID 控制 (核心修改) ===
        # 积分项：消除稳态误差 (比如在弯道一直偏离中心)
        self.lat_integral += lat_err * self.dt
        self.lat_integral = np.clip(self.lat_integral, -3.0, 3.0) # 抗积分饱和
        
        # 微分项：预测误差变化
        lat_d = (lat_err - self.prev_lat_error) / self.dt
        self.prev_lat_error = lat_err
        
        # 组合 PID
        # P项: 负责主要回正
        # I项: 负责在弯道里把车推回中心
        # D项: 负责防止回正过猛
        # head_err: 负责提前对准方向 (前馈)
        steer_rad = - ( (self.kp_lat * lat_err) + \
                        (self.ki_lat * self.lat_integral) + \
                        (self.kd_lat * lat_d) + \
                        (self.kp_head * head_err) )
        
        max_rad = math.radians(self.max_steer_deg)
        steer_rad = np.clip(steer_rad, -max_rad, max_rad)
        raw_steer_cmd = int((steer_rad / max_rad) * 100)
        raw_steer_cmd = np.clip(raw_steer_cmd, -self.limit_steer_cmd, self.limit_steer_cmd)

        # === 4. 纵向 PID 控制 ===
        # 直接读取我们在上一步算好的动态限速
        target_v = self.global_path[c_idx].twist.twist.linear.x
        target_v = max(0.0, target_v)
        
        v_err = target_v - self.current_v
        
        self.vel_integral += v_err * self.dt
        self.vel_integral = np.clip(self.vel_integral, -5.0, 5.0)
        v_d = (v_err - self.prev_vel_error) / self.dt
        self.prev_vel_error = v_err
        
        acc_val = self.kp_vel * v_err + self.ki_vel * self.vel_integral + self.kd_vel * v_d
        
        raw_accel_cmd = 0
        raw_brake_cmd = 0
        if acc_val > 0:
            ratio = acc_val / self.max_accel
            raw_accel_cmd = int(np.clip(ratio, 0, 1) * 100)
        else:
            ratio = abs(acc_val) / self.max_decel
            raw_brake_cmd = int(np.clip(ratio, 0, 1) * 100)

        # === 5. 平滑与发布 ===
        raw_accel_cmd = np.clip(raw_accel_cmd, 0, self.limit_accel_cmd)
        raw_brake_cmd = np.clip(raw_brake_cmd, 0, self.limit_brake_cmd)

        final_steer = self._smooth_command(raw_steer_cmd, self.prev_steer_cmd, self.max_steer_step)
        final_accel = self._smooth_command(raw_accel_cmd, self.prev_accel_cmd, self.max_accel_step)
        final_brake = self._smooth_command(raw_brake_cmd, self.prev_brake_cmd, self.max_brake_step)
        
        # 刹车油门互锁逻辑
        if raw_accel_cmd == 0 and self.prev_accel_cmd > 0:
            final_accel = self._smooth_command(0, self.prev_accel_cmd, self.max_accel_step * 2)
        if raw_brake_cmd == 0 and self.prev_brake_cmd > 0:
            final_brake = self._smooth_command(0, self.prev_brake_cmd, self.max_brake_step * 2)

        self.prev_steer_cmd = final_steer
        self.prev_accel_cmd = final_accel
        self.prev_brake_cmd = final_brake
        
        self.pub_accel.publish(AccelCmd(accel=final_accel))
        self.pub_brake.publish(BrakeCmd(brake=final_brake))
        self.pub_steer.publish(SteerCmd(steer=final_steer))

if __name__ == '__main__':
    try:
        node = PIDRunnerNode()
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'q':
                node.smooth_stop() 
                print("\n程序已退出。")
                break
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


# # #!/usr/bin/env python3
# # # encoding: utf-8

# # import rospy
# # import math
# # import numpy as np
# # import sys
# # import select
# # import termios
# # import tty
# # from nav_msgs.msg import Odometry, Path
# # from autoware_msgs.msg import AccelCmd, BrakeCmd, SteerCmd
# # from tf.transformations import euler_from_quaternion

# # # === 键盘读取函数 ===
# # def getKey(timeout=0.1):
# #     fd = sys.stdin.fileno()
# #     old_settings = termios.tcgetattr(fd)
# #     try:
# #         tty.setraw(fd)
# #         rlist, _, _ = select.select([sys.stdin], [], [], timeout)
# #         if rlist:
# #             key = sys.stdin.read(1)
# #         else:
# #             key = ''
# #     finally:
# #         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
# #     return key.lower()

# # class PIDRunnerNode:
# #     def __init__(self):
# #         rospy.init_node('pid_runner_node', anonymous=True)
        
# #         # === 运行状态标志位 ===
# #         self.is_running = True 
# #         self.goal_reached = False # 新增：终点标志位

# #         # === 1. 频率设置 ===
# #         self.control_rate = 100  # 50Hz (20ms)
# #         self.dt = 1.0 / self.control_rate
        
# #         # === 2. 安全限位 (Hard Limits) ===
# #         self.limit_steer_cmd = 60  
# #         self.limit_accel_cmd = 15  
# #         self.limit_brake_cmd = 15  

# #         # === 3. 平滑参数 (Slew Rate Limits) ===
# #         self.max_steer_step = rospy.get_param('~max_steer_step', 3) 
# #         self.max_accel_step = rospy.get_param('~max_accel_step', 1)
# #         self.max_brake_step = rospy.get_param('~max_brake_step', 3)

# #         # === 4. PID 参数 ===
# #         self.kp_vel = rospy.get_param('~kp_vel', 0.5)
# #         self.ki_vel = rospy.get_param('~ki_vel', 0.1)
# #         self.kd_vel = rospy.get_param('~kd_vel', 0.1)
# #         self.target_speed = rospy.get_param('~target_speed', 10.0)
        
# #         self.kp_lat = rospy.get_param('~kp_lat', 0.025)
# #         self.kp_head = rospy.get_param('~kp_head', 1.0)
# #         self.look_ahead_dist = rospy.get_param('~look_ahead_dist', 3.0)
        
# #         # === 新增：终点停车阈值 ===
# #         self.goal_dist_threshold = rospy.get_param('~goal_dist_threshold', 1.5)

# #         self.max_steer_deg = 40.0 
# #         self.max_accel = 2.0
# #         self.max_decel = 3.0

# #         # === 5. 内部状态 ===
# #         self.current_pose = None
# #         self.current_v = 0.0
# #         self.current_yaw = 0.0
# #         self.global_path = None
        
# #         self.vel_integral = 0.0
# #         self.prev_vel_error = 0.0
        
# #         self.prev_steer_cmd = 0
# #         self.prev_accel_cmd = 0
# #         self.prev_brake_cmd = 0
        
# #         # ROS 接口
# #         rospy.Subscriber('/vehicle_odom', Odometry, self.odom_callback)
# #         rospy.Subscriber('/global_path', Path, self.path_callback)
        
# #         self.pub_accel = rospy.Publisher('/accel_cmd', AccelCmd, queue_size=1)
# #         self.pub_brake = rospy.Publisher('/brake_cmd', BrakeCmd, queue_size=1)
# #         self.pub_steer = rospy.Publisher('/steer_cmd', SteerCmd, queue_size=1)
        
# #         self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        
# #         rospy.loginfo(f"PID Runner Started @ {self.control_rate}Hz")
# #         print(f"设置: 离终点 {self.goal_dist_threshold}m 内自动停车")
# #         print("按 'q' 键停止程序并执行【平滑刹车】...")

# #     def odom_callback(self, msg):
# #         self.current_pose = msg.pose.pose.position
# #         self.current_v = msg.twist.twist.linear.x
# #         orientation = msg.pose.pose.orientation
# #         _, _, self.current_yaw = euler_from_quaternion([
# #             orientation.x, orientation.y, orientation.z, orientation.w
# #         ])

# #     def path_callback(self, msg):
# #         self.global_path = msg.poses

# #     def check_goal_reached(self):
# #         if not self.global_path or not self.current_pose:
# #             return False
# #         last_pose = self.global_path[-1].pose.position
# #         dx = self.current_pose.x - last_pose.x
# #         dy = self.current_pose.y - last_pose.y
# #         dist = math.sqrt(dx**2 + dy**2)
# #         return dist < self.goal_dist_threshold

# #     def find_target(self):
# #         if not self.current_pose or not self.global_path: return None, None
# #         min_dist = float('inf')
# #         c_idx = -1
# #         cx, cy = self.current_pose.x, self.current_pose.y
# #         step = 2 
# #         for i in range(0, len(self.global_path), step):
# #             p = self.global_path[i].pose.position
# #             d = (cx - p.x)**2 + (cy - p.y)**2
# #             if d < min_dist:
# #                 min_dist = d
# #                 c_idx = i
# #         if c_idx == -1: return None, None
# #         t_idx = c_idx
# #         dist_acc = 0
# #         while t_idx < len(self.global_path)-1 and dist_acc < self.look_ahead_dist:
# #             p1 = self.global_path[t_idx].pose.position
# #             p2 = self.global_path[t_idx+1].pose.position
# #             dist_acc += math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2)
# #             t_idx += 1
# #         return c_idx, t_idx

# #     def _smooth_command(self, target, current, max_step):
# #         diff = target - current
# #         clipped_diff = np.clip(diff, -max_step, max_step)
# #         return int(current + clipped_diff)

# #     def emergency_stop(self):
# #         """急停逻辑 (保留以备不时之需)"""
# #         self.is_running = False 
# #         self.pub_accel.publish(AccelCmd(accel=0))
# #         self.pub_brake.publish(BrakeCmd(brake=0))
# #         self.pub_steer.publish(SteerCmd(steer=0))

# #     def smooth_stop(self):
# #         """【平滑舒适刹车逻辑】"""
# #         rospy.loginfo("触发停车指令，开始执行平滑刹车...")
        
# #         # 1. 停止 PID 计算，接管控制权
# #         # 这非常重要：一旦设为False，control_loop就会直接return，防止PID逻辑干扰刹车
# #         self.is_running = False 
        
# #         # 2. 设置舒适刹车参数
# #         target_brake = 30  # 目标刹车力度 (0-100)，30通常比较温和
# #         current_brake = 0
# #         rate = rospy.Rate(20) # 20Hz 更新频率
        
# #         # 3. 循环执行：逐渐增加刹车，保持油门为0，保持方向盘不变
# #         while current_brake < target_brake and not rospy.is_shutdown():
# #             current_brake += 1.0  # 每次增加1，大约1.5秒达到30
            
# #             self.pub_accel.publish(AccelCmd(accel=0))
# #             self.pub_brake.publish(BrakeCmd(brake=int(current_brake)))
            
# #             # 关键：保持上一帧的方向盘角度，防止刹车时方向盘突然回正导致车身晃动
# #             self.pub_steer.publish(SteerCmd(steer=self.prev_steer_cmd))
            
# #             rate.sleep()
        
# #         # 4. 刹车保持一小会儿，确保完全停稳
# #         rospy.sleep(1.0)
        
# #         # 5. 最后彻底归零 (松开刹车，方向盘回正)
# #         self.pub_accel.publish(AccelCmd(accel=0))
# #         self.pub_brake.publish(BrakeCmd(brake=0))
# #         self.pub_steer.publish(SteerCmd(steer=0))
# #         rospy.loginfo("车辆已平稳停止，控制量归零。")

# #     def control_loop(self, event):
# #         # 0. 如果已被停止（例如正在执行平滑刹车），不再执行 PID 逻辑
# #         if not self.is_running:
# #             return

# #         if not self.current_pose or not self.global_path: return
        
# #         # === 核心修改：到达终点 ===
# #         if self.check_goal_reached():
# #             if not self.goal_reached: 
# #                 rospy.loginfo(f"=== 到达终点 (距离<{self.goal_dist_threshold}m) ===")
# #                 self.goal_reached = True
                
# #                 # === 修改处：这里改为调用平滑刹车 ===
# #                 self.smooth_stop() 
                
# #             return

# #         c_idx, t_idx = self.find_target()
# #         if c_idx is None: return
        
# #         p1 = self.global_path[c_idx].pose.position
# #         p2 = self.global_path[min(c_idx+1, len(self.global_path)-1)].pose.position
# #         path_yaw = math.atan2(p2.y - p1.y, p2.x - p1.x)
# #         dx = self.current_pose.x - p1.x
# #         dy = self.current_pose.y - p1.y
# #         lat_err = -dx * math.sin(path_yaw) + dy * math.cos(path_yaw)
        
# #         tp1 = self.global_path[t_idx].pose.position
# #         tp2 = self.global_path[min(t_idx+1, len(self.global_path)-1)].pose.position
# #         target_yaw = math.atan2(tp2.y - tp1.y, tp2.x - tp1.x)
# #         head_err = self.current_yaw - target_yaw
# #         while head_err > math.pi: head_err -= 2*math.pi
# #         while head_err < -math.pi: head_err += 2*math.pi
        
# #         steer_rad =  (self.kp_lat * lat_err + self.kp_head * head_err)
# #         max_rad = math.radians(self.max_steer_deg)
# #         steer_rad = np.clip(steer_rad, -max_rad, max_rad)
# #         raw_steer_cmd = int((steer_rad / max_rad) * 100)
# #         raw_steer_cmd = np.clip(raw_steer_cmd, -self.limit_steer_cmd, self.limit_steer_cmd)

# #         target_v = self.target_speed / 3.6
# #         v_err = target_v - self.current_v
# #         self.vel_integral += v_err * self.dt
# #         self.vel_integral = np.clip(self.vel_integral, -5.0, 5.0)
# #         v_d = (v_err - self.prev_vel_error) / self.dt
# #         self.prev_vel_error = v_err
# #         acc_val = self.kp_vel * v_err + self.ki_vel * self.vel_integral + self.kd_vel * v_d
        
# #         raw_accel_cmd = 0
# #         raw_brake_cmd = 0
# #         if acc_val > 0:
# #             ratio = acc_val / self.max_accel
# #             raw_accel_cmd = int(np.clip(ratio, 0, 1) * 100)
# #         else:
# #             ratio = abs(acc_val) / self.max_decel
# #             raw_brake_cmd = int(np.clip(ratio, 0, 1) * 100)

# #         raw_accel_cmd = np.clip(raw_accel_cmd, 0, self.limit_accel_cmd)
# #         raw_brake_cmd = np.clip(raw_brake_cmd, 0, self.limit_brake_cmd)

# #         final_steer = self._smooth_command(raw_steer_cmd, self.prev_steer_cmd, self.max_steer_step)
# #         final_accel = self._smooth_command(raw_accel_cmd, self.prev_accel_cmd, self.max_accel_step)
# #         final_brake = self._smooth_command(raw_brake_cmd, self.prev_brake_cmd, self.max_brake_step)
        
# #         if raw_accel_cmd == 0 and self.prev_accel_cmd > 0:
# #             final_accel = self._smooth_command(0, self.prev_accel_cmd, self.max_accel_step * 2)
# #         if raw_brake_cmd == 0 and self.prev_brake_cmd > 0:
# #             final_brake = self._smooth_command(0, self.prev_brake_cmd, self.max_brake_step * 2)

# #         self.prev_steer_cmd = final_steer
# #         self.prev_accel_cmd = final_accel
# #         self.prev_brake_cmd = final_brake
        
# #         self.pub_accel.publish(AccelCmd(accel=final_accel))
# #         self.pub_brake.publish(BrakeCmd(brake=final_brake))
# #         self.pub_steer.publish(SteerCmd(steer=final_steer))

# # if __name__ == '__main__':
# #     try:
# #         node = PIDRunnerNode()
        
# #         # === 主循环检测按键 ===
# #         rate = rospy.Rate(10) 
# #         while not rospy.is_shutdown():
# #             key = getKey()
# #             if key == 'q':
# #                 # === 按键也调用平滑刹车 ===
# #                 node.smooth_stop() 
# #                 print("\n程序已退出，车辆已平稳停止。")
# #                 break
# #             rate.sleep()
            
# #     except rospy.ROSInterruptException:
# #         pass
        
# # #     4. 调试指南（非常重要）
# # # PID 算法看似简单，核心全在参数（Kp）。请按以下步骤调试：

# # # 第一步：确认符号（方向）
# # # 把车开到直路上，偏左一点。

# # # 启动节点，看方向盘是不是往右修。

# # # 如果反了，去代码里把 steer_rad = - (...) 那个负号去掉或加上。

# # # 第二步：调航向系数 (kp_head)
# # # 把 kp_lat 设为 0。

# # # 让车在直路上跑。

# # # 如果车头晃来晃去（震荡），减小 kp_head。

# # # 如果车头回正太慢，增大 kp_head。

# # # 目标：车能顺滑地把车头对准红线，哪怕不在红线上也没关系。

# # # 第三步：调横向系数 (kp_lat)
# # # 现在加上 kp_lat。

# # # 这会让车在对准方向的同时，努力往红线上靠。

# # # 如果车在红线上画大龙（S形），减小 kp_lat。

# # # 如果车一直贴着红线边走不回去（稳态误差），增大 kp_lat 或加一点点 ki_lat（通常不需要）。

# # # 第四步：调前视距离 (look_ahead_dist)
# # # 这是 PID 的“金手指”。

# # # 如果你发现车总是画龙，调参数也压不住，把前视距离改大（比如 5米）。

# # # 看远一点，车开得就稳，但是过弯会切内弯（抄近道）。     
        

# #!/usr/bin/env python3
# # encoding: utf-8

# import rospy
# import math
# import numpy as np
# import sys
# import select
# import termios
# import tty
# from nav_msgs.msg import Odometry
# # 【修改点 1】导入 Autoware 消息类型
# from autoware_msgs.msg import AccelCmd, BrakeCmd, SteerCmd, Lane 
# from tf.transformations import euler_from_quaternion

# # === 键盘读取函数 (保持不变) ===
# def getKey(timeout=0.1):
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(fd)
#         rlist, _, _ = select.select([sys.stdin], [], [], timeout)
#         if rlist:
#             key = sys.stdin.read(1)
#         else:
#             key = ''
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return key.lower()

# class PIDRunnerNode:
#     def __init__(self):
#         rospy.init_node('pid_runner_node', anonymous=True)
        
#         # === 运行状态标志位 ===
#         self.is_running = True 
#         self.goal_reached = False 

#         # === 1. 频率设置 ===
#         self.control_rate = 100  # 50Hz (20ms)
#         self.dt = 1.0 / self.control_rate
        
#         # === 2. 安全限位 (Hard Limits) ===
#         self.limit_steer_cmd = 100  
#         self.limit_accel_cmd = 15  
#         self.limit_brake_cmd = 15  

#         # === 3. 平滑参数 (Slew Rate Limits) ===
#         self.max_steer_step = rospy.get_param('~max_steer_step', 3) 
#         self.max_accel_step = rospy.get_param('~max_accel_step', 1)
#         self.max_brake_step = rospy.get_param('~max_brake_step', 3)

#         # === 4. PID 参数 ===
#         self.kp_vel = rospy.get_param('~kp_vel', 0.25)
#         self.ki_vel = rospy.get_param('~ki_vel', 0.1)
#         self.kd_vel = rospy.get_param('~kd_vel', 0.2)
        
#         # 【保持不变】依然使用内部设定的目标速度，不使用 Topic 里的速度
#         # self.target_speed = rospy.get_param('~target_speed', 10.0)
        
#         self.kp_lat = rospy.get_param('~kp_lat', 0.05)
#         self.kp_head = rospy.get_param('~kp_head', 0.8)
#         self.look_ahead_dist = rospy.get_param('~look_ahead_dist', 3.0)
        
#         # === 新增：终点停车阈值 ===
#         self.goal_dist_threshold = rospy.get_param('~goal_dist_threshold', 1.5)

#         self.max_steer_deg = 40.0
#         self.max_accel = 2.0
#         self.max_decel = 3.0

#         # === 5. 内部状态 ===
#         self.current_pose = None
#         self.current_v = 0.0
#         self.current_yaw = 0.0
#         self.global_path = None # 这里将存储 Waypoint 列表
        
#         self.vel_integral = 0.0
#         self.prev_vel_error = 0.0
        
#         self.prev_steer_cmd = 0
#         self.prev_accel_cmd = 0
#         self.prev_brake_cmd = 0
        
#         # ROS 接口
#         rospy.Subscriber('/vehicle_odom', Odometry, self.odom_callback)
        
#         # 【修改点 2】订阅 /global_path/lane，类型为 Lane
#         rospy.Subscriber('/global_path/lane', Lane, self.path_callback)
        
#         self.pub_accel = rospy.Publisher('/accel_cmd', AccelCmd, queue_size=1)
#         self.pub_brake = rospy.Publisher('/brake_cmd', BrakeCmd, queue_size=1)
#         self.pub_steer = rospy.Publisher('/steer_cmd', SteerCmd, queue_size=1)
        
#         self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        
#         rospy.loginfo(f"PID Runner Started @ {self.control_rate}Hz")
#         print(f"设置: 离终点 {self.goal_dist_threshold}m 内自动停车")
#         print("按 'q' 键停止程序并执行【平滑刹车】...")

#     def odom_callback(self, msg):
#         self.current_pose = msg.pose.pose.position
#         self.current_v = msg.twist.twist.linear.x
#         orientation = msg.pose.pose.orientation
#         _, _, self.current_yaw = euler_from_quaternion([
#             orientation.x, orientation.y, orientation.z, orientation.w
#         ])

#     def path_callback(self, msg):
#         # 【修改点 3】Lane 消息的列表叫 waypoints，不再是 poses
#         self.global_path = msg.waypoints

#     def check_goal_reached(self):
#         if not self.global_path or not self.current_pose:
#             return False
#         # 【修改点 4】多加一层 .pose，变成 .pose.pose.position
#         # last_pose = self.global_path[-1].pose.position (旧)
#         last_pose = self.global_path[-1].pose.pose.position 
        
#         dx = self.current_pose.x - last_pose.x
#         dy = self.current_pose.y - last_pose.y
#         dist = math.sqrt(dx**2 + dy**2)
#         return dist < self.goal_dist_threshold

#     def find_target(self):
#         if not self.current_pose or not self.global_path: return None, None
#         min_dist = float('inf')
#         c_idx = -1
#         cx, cy = self.current_pose.x, self.current_pose.y
#         step = 2 
#         for i in range(0, len(self.global_path), step):
#             # 【修改点 5】多加一层 .pose
#             p = self.global_path[i].pose.pose.position
#             d = (cx - p.x)**2 + (cy - p.y)**2
#             if d < min_dist:
#                 min_dist = d
#                 c_idx = i
#         if c_idx == -1: return None, None
#         t_idx = c_idx
#         dist_acc = 0
#         while t_idx < len(self.global_path)-1 and dist_acc < self.look_ahead_dist:
#             # 【修改点 6】多加一层 .pose
#             p1 = self.global_path[t_idx].pose.pose.position
#             p2 = self.global_path[t_idx+1].pose.pose.position
#             dist_acc += math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2)
#             t_idx += 1
#         return c_idx, t_idx

#     def _smooth_command(self, target, current, max_step):
#         diff = target - current
#         clipped_diff = np.clip(diff, -max_step, max_step)
#         return int(current + clipped_diff)

#     def smooth_stop(self):
#         """【平滑舒适刹车逻辑】"""
#         rospy.loginfo("触发停车指令，开始执行平滑刹车...")
#         self.is_running = False 
        
#         target_brake = 30 
#         current_brake = 0
#         rate = rospy.Rate(20) 
        
#         while current_brake < target_brake and not rospy.is_shutdown():
#             current_brake += 1.0 
#             self.pub_accel.publish(AccelCmd(accel=0))
#             self.pub_brake.publish(BrakeCmd(brake=int(current_brake)))
#             self.pub_steer.publish(SteerCmd(steer=self.prev_steer_cmd))
#             rate.sleep()
        
#         rospy.sleep(1.0)
#         self.pub_accel.publish(AccelCmd(accel=0))
#         self.pub_brake.publish(BrakeCmd(brake=0))
#         self.pub_steer.publish(SteerCmd(steer=0))
#         rospy.loginfo("车辆已平稳停止，控制量归零。")

#     def control_loop(self, event):
#         if not self.is_running:
#             return

#         if not self.current_pose or not self.global_path: return
        
#         if self.check_goal_reached():
#             if not self.goal_reached: 
#                 rospy.loginfo(f"=== 到达终点 (距离<{self.goal_dist_threshold}m) ===")
#                 self.goal_reached = True
#                 self.smooth_stop() 
#             return

#         c_idx, t_idx = self.find_target()
#         if c_idx is None: return
        
#         # 【修改点 7】以下所有获取坐标的地方都多加了一层 .pose
#         p1 = self.global_path[c_idx].pose.pose.position
#         p2 = self.global_path[min(c_idx+1, len(self.global_path)-1)].pose.pose.position
#         path_yaw = math.atan2(p2.y - p1.y, p2.x - p1.x)
#         dx = self.current_pose.x - p1.x
#         dy = self.current_pose.y - p1.y
#         lat_err = -dx * math.sin(path_yaw) + dy * math.cos(path_yaw)
        
#         # 预瞄点坐标
#         tp1 = self.global_path[t_idx].pose.pose.position
#         tp2 = self.global_path[min(t_idx+1, len(self.global_path)-1)].pose.pose.position
#         target_yaw = math.atan2(tp2.y - tp1.y, tp2.x - tp1.x)
#         head_err = self.current_yaw - target_yaw
#         while head_err > math.pi: head_err -= 2*math.pi
#         while head_err < -math.pi: head_err += 2*math.pi
        
#         steer_rad = - (self.kp_lat * lat_err + self.kp_head * head_err)
#         max_rad = math.radians(self.max_steer_deg)
#         steer_rad = np.clip(steer_rad, -max_rad, max_rad)
#         raw_steer_cmd = int((steer_rad / max_rad) * 100)
#         raw_steer_cmd = np.clip(raw_steer_cmd, -self.limit_steer_cmd, self.limit_steer_cmd)

#         # 【保持不变】依然使用 self.target_speed (固定值)
#         target_v = self.global_path[c_idx].twist.twist.linear.x

#         # 安全保护，防止出现负值（虽然后面PID逻辑也能处理，但加上更严谨）
#         target_v = max(0.0, target_v)
#         v_err = target_v - self.current_v
        
#         self.vel_integral += v_err * self.dt
#         self.vel_integral = np.clip(self.vel_integral, -5.0, 5.0)
#         v_d = (v_err - self.prev_vel_error) / self.dt
#         self.prev_vel_error = v_err
#         acc_val = self.kp_vel * v_err + self.ki_vel * self.vel_integral + self.kd_vel * v_d
        
#         raw_accel_cmd = 0
#         raw_brake_cmd = 0
#         if acc_val > 0:
#             ratio = acc_val / self.max_accel
#             raw_accel_cmd = int(np.clip(ratio, 0, 1) * 100)
#         else:
#             ratio = abs(acc_val) / self.max_decel
#             raw_brake_cmd = int(np.clip(ratio, 0, 1) * 100)

#         raw_accel_cmd = np.clip(raw_accel_cmd, 0, self.limit_accel_cmd)
#         raw_brake_cmd = np.clip(raw_brake_cmd, 0, self.limit_brake_cmd)

#         final_steer = self._smooth_command(raw_steer_cmd, self.prev_steer_cmd, self.max_steer_step)
#         final_accel = self._smooth_command(raw_accel_cmd, self.prev_accel_cmd, self.max_accel_step)
#         final_brake = self._smooth_command(raw_brake_cmd, self.prev_brake_cmd, self.max_brake_step)
        
#         if raw_accel_cmd == 0 and self.prev_accel_cmd > 0:
#             final_accel = self._smooth_command(0, self.prev_accel_cmd, self.max_accel_step * 2)
#         if raw_brake_cmd == 0 and self.prev_brake_cmd > 0:
#             final_brake = self._smooth_command(0, self.prev_brake_cmd, self.max_brake_step * 2)

#         self.prev_steer_cmd = final_steer
#         self.prev_accel_cmd = final_accel
#         self.prev_brake_cmd = final_brake
        
#         self.pub_accel.publish(AccelCmd(accel=final_accel))
#         self.pub_brake.publish(BrakeCmd(brake=final_brake))
#         self.pub_steer.publish(SteerCmd(steer=final_steer))

# if __name__ == '__main__':
#     try:
#         node = PIDRunnerNode()
#         rate = rospy.Rate(10) 
#         while not rospy.is_shutdown():
#             key = getKey()
#             if key == 'q':
#                 node.smooth_stop() 
#                 print("\n程序已退出，车辆已平稳停止。")
#                 break
#             rate.sleep()
            
#     except rospy.ROSInterruptException:
#         pass


