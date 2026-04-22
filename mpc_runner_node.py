#!/usr/bin/env python3
# mpc_runner_node.py
# ==========================================
# ROS 节点：MPC 控制器主节点
# 完整迁移自 mpc_runner_common._execute_control_step
# 频率：20Hz（DT=0.05s），MPC 与 LQR 同频计算
# ==========================================

import rospy
import numpy as np
import math

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from simple_mpc_controller import SimpleMPController, State, NX, NP

try:
    from autoware_msgs.msg import Lane, AccelCmd, BrakeCmd, SteerCmd
except ImportError:
    rospy.logwarn("autoware_msgs not found, please check your ROS environment.")


# ============================================================
# 常量（与 mpc_runner_common 保持一致）
# ============================================================
SIM_DT = 0.05
PLANNER_FAILURE_COST = 8000.0
OFF_ROAD_COST = 7000.0


class MPCRunnerNode:
    def __init__(self):
        rospy.init_node('mpc_runner_node', anonymous=True)

        # ----------------------------------------------------------
        # 1. 基础物理参数
        # ----------------------------------------------------------
        self.wb = rospy.get_param('~wheelbase', 2.875)
        self.control_rate = rospy.get_param('~rate', 20)          # Hz
        self.max_steer_limit = rospy.get_param('~max_steer_limit', 0.6)   # rad
        self.max_acc_limit   = rospy.get_param('~max_acc_limit',   3.0)   # m/s²
        self.max_decel_limit = rospy.get_param('~max_decel_limit', 5.0)   # m/s²
        self.brake_deadzone  = rospy.get_param('~brake_deadzone',  10.0)  # %

        # ----------------------------------------------------------
        # 2. MPC 权重（与 verify_mpc.py 的 test_params 对齐）
        # ----------------------------------------------------------
        q_d       = rospy.get_param('~q_d',       10.0)
        q_v       = rospy.get_param('~q_v',       77.9519)
        q_yaw     = rospy.get_param('~q_yaw',      5.0)
        q_d_dot   = rospy.get_param('~q_d_dot',    1.0)
        q_yaw_dot = rospy.get_param('~q_yaw_dot', 50.0)
        q_d_d_dot   = rospy.get_param('~q_d_d_dot',   0.1)
        q_yaw_d_dot = rospy.get_param('~q_yaw_d_dot', 0.1)

        r_accel  = rospy.get_param('~r_accel',   30.0)
        r_steer  = rospy.get_param('~r_steer',   50.0)
        rd_accel = rospy.get_param('~rd_accel',  30.0)
        rd_steer = rospy.get_param('~rd_steer', 100.0)

        R  = np.diag([r_accel,  r_steer])
        Rd = np.diag([rd_accel, rd_steer])

        # ----------------------------------------------------------
        # 3. Tube 配置（与 verify_mpc.py 的 tube_cfg 对齐，scaling 全 0）
        # ----------------------------------------------------------
        tube_cfg = {
            'enable_tube':    True,
            'tube_mode':      'probabilistic',
            'static_margin':  0.0,
            'enable_lqr':     True,

            # 传感器底噪
            'static_noise_std': [0.0, 0.0, 0.0, 0.0],

            # 执行器噪声
            'actuator_std_a':     0.0,
            'actuator_std_delta': 0.0,

            # 物理失配系数（测试阶段全 0）
            'vx_scaling':          0.0,
            'vy_scaling':          0.0,
            'centrifugal_scaling': 0.0,
            'yaw_rate_scaling':    0.0,

            # 外部环境噪声保底
            'external_static_noise': 0.0,

            # 预测置信度衰减
            'lambda_model': 0.95,

            # 管道半径与最大裕度阈值
            'max_margin_threshold': 0.1,
            'tube_radius':          0.3,
        }

        # ----------------------------------------------------------
        # 4. 实例化 MPC 控制器
        # ----------------------------------------------------------
        self.system_delay = rospy.get_param('~system_delay', 0.0)

        self.controller = SimpleMPController(
            wb=self.wb,
            q_d=q_d, q_v=q_v, q_yaw=q_yaw,
            q_d_dot=q_d_dot, q_yaw_dot=q_yaw_dot,
            q_d_d_dot=q_d_d_dot, q_yaw_d_dot=q_yaw_d_dot,
            R=R, Rd=Rd,
            max_steer=self.max_steer_limit,
            system_delay=self.system_delay,
            tube_config=tube_cfg,
        )

        # tube_radius 和 max_margin_threshold 供内部使用
        self.tube_radius          = tube_cfg['tube_radius']
        self.max_margin_threshold = tube_cfg['max_margin_threshold']

        # ----------------------------------------------------------
        # 5. 状态缓存
        # ----------------------------------------------------------
        self.current_state  = State()
        self.state_received = False

        # 来自 planning_node 的参考轨迹
        self.ref_traj       = None   # shape: [NX, NP+1]  [x, y, v, yaw]
        self.ref_curvatures = None   # shape: [NP+1]
        self.ref_delta_refs = None   # shape: [NP+1]
        self.ref_lanes      = None   # shape: [2, NP+1]
        self.traj_received  = False

        # 蠕行补偿参数（实车挂档后约5km/h蠕行，需轻刹抵消）
        # CREEP_BRAKE 需实车标定，当前保守默认值 12%
        self.creep_speed = rospy.get_param('~creep_speed', 1.4)  # m/s ≈ 5km/h
        self.creep_brake = rospy.get_param('~creep_brake', 12)   # %

        # 控制量历史缓冲（用于热启动与 u_history 同步，对齐 mpc_runner_common）
        self._last_applied_acc   = 0.0
        self._last_applied_steer = 0.0

        # 误差分解缓存（MPC帧写入，LQR帧复用）
        self._cached_intent   = 0.0
        self._cached_mismatch = 0.0

        # ----------------------------------------------------------
        # 6. 发布 / 订阅
        # ----------------------------------------------------------
        self.sub_odom = rospy.Subscriber(
            '/vehicle_odom', Odometry, self._odom_callback, queue_size=1)
        self.sub_path = rospy.Subscriber(
            '/mpc/reference_path', Lane, self._path_callback, queue_size=1)

        self.pub_accel = rospy.Publisher('/accel_cmd', AccelCmd, queue_size=1)
        self.pub_brake = rospy.Publisher('/brake_cmd', BrakeCmd, queue_size=1)
        self.pub_steer = rospy.Publisher('/steer_cmd', SteerCmd, queue_size=1)

        # 20Hz 控制循环
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self._control_loop)

        rospy.loginfo("[MPCRunnerNode] Initialized. Rate=%dHz, WB=%.3fm, max_steer=%.3frad",
                      self.control_rate, self.wb, self.max_steer_limit)

    # ==============================================================
    # 回调：里程计
    # ==============================================================
    def _odom_callback(self, msg):
        self.current_state.x = msg.pose.pose.position.x
        self.current_state.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_state.yaw = yaw

        # 取车体坐标系下的纵向速度（实车通常是 linear.x）
        self.current_state.v = msg.twist.twist.linear.x
        self.state_received = True

    # ==============================================================
    # 回调：参考轨迹（来自 planning_node）
    # ==============================================================
    def _path_callback(self, msg):
        """
        解析 Lane 消息，提取：
          - xref      : [NX, N]  其中 NX=[x, y, v, yaw]（行序与 simple_mpc_controller 一致）
          - curvatures: [N]
          - delta_refs: [N]  前馈转角 arctan(wb * k)
        """
        if not msg.waypoints:
            return

        points     = []
        curvatures = []

        for wp in msg.waypoints:
            x = wp.pose.pose.position.x
            y = wp.pose.pose.position.y

            q = wp.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            v = wp.twist.twist.linear.x
            k = wp.twist.twist.angular.z   # planning_node 里的 trick：曲率存在 angular.z

            # 顺序：[x, y, v, yaw]  对应 NX=4
            points.append([x, y, v, yaw])
            curvatures.append(k)

        n = len(points)
        if n == 0:
            return

        raw_traj = np.array(points).T          # [4, N]
        raw_k    = np.array(curvatures)        # [N]

        # ---------- Padding 到 NP+1 ----------
        target_len = NP + 1
        if n >= target_len:
            xref      = raw_traj[:, :target_len]
            k_ref     = raw_k[:target_len]
        else:
            pad = target_len - n
            xref  = np.pad(raw_traj, ((0, 0), (0, pad)), mode='edge')
            k_ref = np.pad(raw_k,    (0, pad),            mode='edge')
            # 末端补齐部分速度归零，防止冲出终点
            xref[2, n:] = 0.0

        # ---------- 固定 Tube 半径车道边界 ----------
        # 与 mpc_runner_common._process_planner_data 逻辑一致：
        # tube_radius > 0 时使用固定管道宽，忽略地图车道
        lane_boundaries        = np.zeros((2, target_len))
        lane_boundaries[0, :]  = -self.tube_radius   # 左边界（负值）
        lane_boundaries[1, :]  =  self.tube_radius   # 右边界

        # ---------- 前馈转角 ----------
        delta_refs = np.arctan(self.wb * k_ref)

        self.ref_traj       = xref
        self.ref_curvatures = k_ref
        self.ref_delta_refs = delta_refs
        self.ref_lanes      = lane_boundaries
        self.traj_received  = True

    # ==============================================================
    # 主控制循环（20Hz）
    # ==============================================================
    def _control_loop(self, event):
        if not (self.state_received and self.traj_received and self.ref_lanes is not None):
            return

        state = self.current_state
        xref       = self.ref_traj
        k_ref      = self.ref_curvatures
        delta_refs = self.ref_delta_refs
        lane_boundaries = self.ref_lanes

        # ------------------------------------------------------
        # Step 1：记录单步预测误差（actual - last_predicted）
        # 对应 mpc_runner_common 里 pred_state 误差分解逻辑
        # ------------------------------------------------------
        pred_state_before_solve = self.controller.last_predicted_state
        if pred_state_before_solve is not None:
            err_x   = state.x   - pred_state_before_solve[0]
            err_y   = state.y   - pred_state_before_solve[1]
            err_v   = state.v   - pred_state_before_solve[2]
            err_yaw = (state.yaw - pred_state_before_solve[3] + math.pi) % (2 * math.pi) - math.pi

            phi     = pred_state_before_solve[3]
            err_lat = -math.sin(phi) * err_x + math.cos(phi) * err_y

            rospy.logdebug("[pred_error] lat=%.4fm, total=%.4fm, v=%.4fm/s, yaw=%.4frad",
                           err_lat, math.sqrt(err_x**2 + err_y**2), err_v, err_yaw)

        # ------------------------------------------------------
        # Step 2：MPC 求解
        # 对应 mpc_runner_common 里 is_mpc_frame 大周期逻辑
        # 方案 C：无 Pass2，直接使用 planning_node 已限速的轨迹
        # ------------------------------------------------------
        try:
            mpc_res = self.controller.solve(
                state=state,
                xref=xref,
                lane_boundaries=lane_boundaries,
                curvatures=k_ref,
                delta_refs=delta_refs,
            )
        except Exception as e:
            rospy.logerr_throttle(1.0, "[MPC] solve() exception: %s", str(e))
            self._publish_control_commands(-2.0, 0.0)
            return

        oa, odelta, ox, oy, oyaw, ov, safety_margins, solve_status = mpc_res

        if oa is None or odelta is None:
            rospy.logwarn_throttle(1.0, "[MPC] Solve failed (status=%s). Emergency brake.", solve_status)
            self._publish_control_commands(-2.0, 0.0)
            return

        # 提取第 0 步名义控制量
        raw_mpc_acc   = float(oa[0])
        raw_mpc_steer = float(odelta[0])

        # ------------------------------------------------------
        # Step 3：LQR 高频补偿
        # 对应 mpc_runner_common 里 LQR 补偿段
        # ------------------------------------------------------
        lqr_acc_comp   = 0.0
        lqr_steer_comp = 0.0

        if self.controller.use_lqr and self.controller.K_lqr is not None:
            # 名义状态取第1步（下一时刻MPC预测位置），而非第0步（等于当前状态，误差为零无意义）
            x_nom  = np.array([ox[1], oy[1], ov[1], oyaw[1]])
            x_meas = np.array([state.x, state.y, state.v, state.yaw])

            ex = x_meas - x_nom
            # 航向角误差归一化
            ex[3] = (ex[3] + math.pi) % (2 * math.pi) - math.pi

            u_fb = -self.controller.K_lqr @ ex
            lqr_acc_comp   = float(u_fb[0])
            lqr_steer_comp = float(u_fb[1])

        raw_acc_cmd   = raw_mpc_acc   + lqr_acc_comp
        raw_steer_cmd = raw_mpc_steer + lqr_steer_comp

        # ------------------------------------------------------
        # Step 4：截断（仅绝对值限幅，不加 Slew Rate）
        # MPC 内部已有 Slew Rate 约束，Runner 层不重复限制
        # 对应 mpc_runner_common 注释："只做绝对值截断，不做 Slew Rate"
        # ------------------------------------------------------
        acc_cmd   = float(np.clip(raw_acc_cmd,
                                  -self.controller.max_accel,
                                   self.controller.max_accel))
        steer_cmd = float(np.clip(raw_steer_cmd,
                                  -self.controller.max_steer,
                                   self.controller.max_steer))

        # ------------------------------------------------------
        # Step 5：更新 u_history（热启动 + 下一帧延迟补偿用）
        # 对应 mpc_runner_common 里 u_history 实时同步逻辑
        # ------------------------------------------------------
        self.controller.u_history[1] = self.controller.u_history[0].copy()
        self.controller.u_history[0] = np.array([acc_cmd, steer_cmd])

        self._last_applied_acc   = acc_cmd
        self._last_applied_steer = steer_cmd

        # ------------------------------------------------------
        # Step 6：误差分解日志（lateral error & safety margin）
        # 对应 mpc_runner_common 里 e_d / intent / mismatch 计算
        # ------------------------------------------------------
        yaw_ref = xref[3, 0]
        e_d = (-math.sin(yaw_ref) * (state.x - xref[0, 0])
               + math.cos(yaw_ref) * (state.y - xref[1, 0]))

        if pred_state_before_solve is not None:
            dx_intent  = pred_state_before_solve[0] - xref[0, 0]
            dy_intent  = pred_state_before_solve[1] - xref[1, 0]
            intent_lat = (-math.sin(yaw_ref) * dx_intent
                          + math.cos(yaw_ref) * dy_intent)
            mismatch_lat = e_d - intent_lat
            self._cached_intent   = intent_lat
            self._cached_mismatch = mismatch_lat
        else:
            intent_lat   = self._cached_intent
            mismatch_lat = self._cached_mismatch

        current_margin = max(safety_margins) if safety_margins else 0.0

        rospy.logdebug(
            "[Control] acc=%.3f steer=%.3f | e_d=%.4fm margin=%.4fm intent=%.4fm mismatch=%.4fm",
            acc_cmd, steer_cmd, e_d, current_margin, intent_lat, mismatch_lat)

        if solve_status == 1:
            rospy.logwarn_throttle(2.0, "[MPC] Relaxed solve (status=1).")
        elif solve_status == 2:
            rospy.logwarn_throttle(2.0, "[MPC] Emergency solve (status=2).")

        # ------------------------------------------------------
        # Step 7：发布控制指令
        # ------------------------------------------------------
        self._publish_control_commands(acc_cmd, steer_cmd)

    # ==============================================================
    # 控制指令发布：物理量 → Autoware AccelCmd / BrakeCmd / SteerCmd
    # ==============================================================
    def _publish_control_commands(self, accel_val, steer_val):
        """
        将 [m/s², rad] 映射到 Autoware 百分比指令。

        纵向映射逻辑：
          accel_val > 0  → AccelCmd 线性映射 (0-100%)
          accel_val < 0  → BrakeCmd (deadzone + 线性映射)
          蠕行补偿：低速且 accel_val 接近 0 时，施加轻刹车抵消蠕行
                   creep_brake 默认 12%，需实车标定后调整

        横向映射逻辑：
          steer_val / max_steer → SteerCmd (-100~100%)
        """
        accel_msg = AccelCmd()
        brake_msg = BrakeCmd()
        steer_msg = SteerCmd()
        now = rospy.Time.now()

        current_v = self.current_state.v

        # ---- 纵向 ----
        if accel_val >= 0.0:
            # 蠕行补偿：车速低于蠕行速度且目标加速度很小时
            # 说明 MPC 不希望车继续加速，需要轻刹抵消蠕行力
            if current_v < self.creep_speed and accel_val < 0.1:
                accel_msg.accel = 0
                brake_msg.brake = self.creep_brake
            else:
                ratio = accel_val / self.max_acc_limit
                accel_msg.accel = int(np.clip(ratio, 0.0, 1.0) * 100)
                brake_msg.brake = 0
        else:
            # 制动区间：brake_deadzone 保证踩过死区后线性增加
            decel = abs(accel_val)
            if decel > 0.1:
                ratio   = decel / self.max_decel_limit
                percent = self.brake_deadzone + ratio * (100.0 - self.brake_deadzone)
                brake_msg.brake = int(np.clip(percent, 0.0, 100.0))
            else:
                # 制动量太小，同样用蠕行刹车兜底，防止车辆溜车
                brake_msg.brake = self.creep_brake
            accel_msg.accel = 0

        # ---- 横向 ----
        steer_ratio     = steer_val / self.max_steer_limit
        steer_msg.steer = int(np.clip(steer_ratio, -1.0, 1.0) * 100)

        # ---- 时间戳 & 发布 ----
        accel_msg.header.stamp = now
        brake_msg.header.stamp = now
        steer_msg.header.stamp = now

        self.pub_accel.publish(accel_msg)
        self.pub_brake.publish(brake_msg)
        self.pub_steer.publish(steer_msg)


# ==============================================================
# 入口
# ==============================================================
if __name__ == '__main__':
    try:
        node = MPCRunnerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass