#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path  
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from autoware_msgs.msg import Lane, Waypoint

try:
    from mpc_local_planner_pure import MPCLocalPlanner
    from trajectory_loader import load_csv_trajectory
except ImportError:
    rospy.logerr("Please make sure mpc_local_planner_pure.py and trajectory_loader.py are in the same directory.")

class ENUPlanningNode:
    def __init__(self):
        rospy.init_node('enu_mpc_planning_node', anonymous=True)
        
        self.csv_path = rospy.get_param('~csv_path', '/home/chd/catkin_ws/scripts/vehicle_data_odom.csv')
        self.target_speed = rospy.get_param('~target_speed', 10.0) 
        self.planning_rate = rospy.get_param('~rate', 20)
        
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "world"
        self.current_velocity = 0.0
        self.state_received = False
        
        self.pub_local_path = rospy.Publisher('/mpc/reference_path', Lane, queue_size=1)
        self.pub_visual_local_path = rospy.Publisher('/mpc/visual_local_path', Path, queue_size=1)
        self.pub_global_path = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
        
        opt_dict = {
            'target_speed': self.target_speed,
            'max_lateral_accel': 6.0,
            'dt': 1.0 / self.planning_rate
        }
        self.planner = MPCLocalPlanner(opt_dict)
        
        rospy.loginfo("Loading trajectory from: %s", self.csv_path)
        global_plan = load_csv_trajectory(self.csv_path, z_height=0)
        
        if not global_plan:
            rospy.logwarn("Failed to load trajectory! Planner will wait...")
        else:
            self.planner.set_global_plan(global_plan)
            self._publish_global_path(global_plan)

        self.sub_odom = rospy.Subscriber('/vehicle_odom', Odometry, self._odom_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0/self.planning_rate), self._run_planning)
        rospy.loginfo("ENU MPC Planning Node Started")

    def _odom_callback(self, msg):
        self.state_received = True
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        self.current_velocity = msg.twist.twist.linear.x

    def _run_planning(self, event):
        if not self.state_received:
            return
            
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        orientation = self.current_pose.pose.orientation
        roll, pitch, yaw_rad = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # 1. 注入真实状态
        self.planner.update_vehicle_state(x, y, z, yaw_rad, self.current_velocity)
        
        try:
            # 2. 调用规划器进行动态切片
            result, _ = self.planner.run_step(self.target_speed)
            
            # 3. 完整解包：提取轨迹 (mpc_traj) 和 曲率 (mpc_k)
            # mpc_local_planner 返回结构: (mpc_traj, corr, bounds, False, mpc_k, delta_refs)
            if result is not None and result[0] is not None:
                mpc_traj = result[0]
                mpc_k = result[4] 
                
                # 将轨迹和曲率一并交给发布函数
                self._publish_trajectory(mpc_traj, mpc_k)
                
        except Exception as e:
            rospy.logwarn_throttle(2, f"Planning Exception: {e}")

    def _publish_trajectory(self, trajectory, curvatures):
        """同时发布 Lane (包含曲率，给控制) 和 Path (给显示)"""
        current_time = rospy.Time.now()
        
        lane_msg = Lane()
        lane_msg.header.stamp = current_time
        lane_msg.header.frame_id = "world"
        
        path_msg = Path()
        path_msg.header.stamp = current_time
        path_msg.header.frame_id = "world"
        
        for i, point in enumerate(trajectory):
            if len(point) < 4: continue
            x, y, yaw_rad, v = point
            k = curvatures[i] # 提取对应点的曲率
            
            wp = Waypoint()
            wp.pose.header = lane_msg.header
            wp.pose.pose.position.x = float(x)
            wp.pose.pose.position.y = float(y)
            wp.pose.pose.position.z = 0 
            
            quat = quaternion_from_euler(0, 0, float(yaw_rad))
            wp.pose.pose.orientation.x = float(quat[0])
            wp.pose.pose.orientation.y = float(quat[1])
            wp.pose.pose.orientation.z = float(quat[2])
            wp.pose.pose.orientation.w = float(quat[3])
            
            # 纵向速度存放点
            wp.twist.twist.linear.x = float(v)
            
            # 【核心 Trick】：将曲率存放在闲置的角速度 Z 轴字段中
            wp.twist.twist.angular.z = float(k)
            
            lane_msg.waypoints.append(wp)
            
            # 可视化用的 PoseStamped (不含速度和曲率)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose = wp.pose.pose 
            path_msg.poses.append(pose)
            
        self.pub_local_path.publish(lane_msg)
        self.pub_visual_local_path.publish(path_msg)

    def _publish_global_path(self, global_plan):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "world"
        
        for wp, cmd in global_plan:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp.transform.location.x
            pose.pose.position.y = wp.transform.location.y
            pose.pose.position.z = 0 
            
            yaw_rad = math.radians(wp.transform.rotation.yaw)
            q = quaternion_from_euler(0, 0, yaw_rad)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            path_msg.poses.append(pose)
            
        self.pub_global_path.publish(path_msg)

if __name__ == '__main__':
    try:
        node = ENUPlanningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass