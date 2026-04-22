#!/usr/bin/env python3
# encoding: utf-8

import csv
import os
import rospy
import sys
import select
import termios
import tty
import math
import numpy as np
from datetime import datetime
from nav_msgs.msg import Path
from autoware_msgs.msg import AccelCmd, BrakeCmd, SteerCmd

# 【核心要求1】严格遵循只 import 原文件，绝对不修改原有 MPC 源码
try:
    from mpc_runner_node import MPCRunnerNode
except ImportError:
    rospy.logerr("找不到 mpc_runner_node.py，请确保其在同一目录下或 PYTHONPATH 中。")


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


class SafeMPCRunner(MPCRunnerNode):
    def __init__(self):
        # 1. 拦截与初始化安全状态
        self.is_running = True
        self.goal_reached = False
        self.global_end_pose = None
        self._last_goal_dist = None
        self._control_ready_logged = False
        self._active_stop_reason = ""

        # 文件日志句柄与节流状态
        self.enable_file_log = False
        self.log_dir = None
        self.csv_log_path = None
        self.txt_log_path = None
        self.control_file_log_interval = 0.2
        self._csv_file = None
        self._txt_file = None
        self._csv_writer = None
        self._last_control_csv_log_sec = None
        self._last_control_txt_log_sec = None
        self._last_wait_txt_log_sec = None
        self._last_stop_txt_log_sec = None

        # 2. 正常初始化 MPC 核心父类节点
        super(SafeMPCRunner, self).__init__()

        self.goal_dist_threshold = rospy.get_param('~goal_dist_threshold', 1.5)
        self.status_log_interval = rospy.get_param('~status_log_interval', 1.0)
        self.wait_log_interval = rospy.get_param('~wait_log_interval', 5.0)
        self.stop_log_interval = rospy.get_param('~stop_log_interval', 0.5)
        self.enable_file_log = rospy.get_param('~enable_file_log', True)
        default_log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        self.log_dir = rospy.get_param('~log_dir', default_log_dir)
        self.control_file_log_interval = rospy.get_param('~control_file_log_interval', 0.2)

        self._setup_file_logging()
        rospy.on_shutdown(self._on_shutdown)

        # 3. 补充订阅全局路径（用于准确判断终点，弥补 MPC 局部视距的缺陷）
        rospy.Subscriber('/global_path', Path, self._global_path_callback, queue_size=1)

        rospy.loginfo("Safe MPC Wrapper 启动成功！已全盘继承 MPC 核心逻辑。")
        rospy.loginfo(
            "日志节流参数: status=%.1fs, wait=%.1fs, stop=%.1fs, file_control=%.1fs",
            self.status_log_interval,
            self.wait_log_interval,
            self.stop_log_interval,
            self.control_file_log_interval
        )
        if self.enable_file_log and self.csv_log_path and self.txt_log_path:
            rospy.loginfo("[SafeMPC] 文件日志路径: csv=%s | txt=%s", self.csv_log_path, self.txt_log_path)

        self._record_event("[SafeMPC] Safe MPC Wrapper 启动成功。", event='node_start')
        if self.enable_file_log and self.csv_log_path and self.txt_log_path:
            self._record_event(
                "[SafeMPC] 文件日志已启用: csv=%s | txt=%s" % (self.csv_log_path, self.txt_log_path),
                event='log_file_ready'
            )

        print(f"设置: 离终点 {self.goal_dist_threshold}m 内自动停车")
        print("按 'q' 键停止程序并执行【平滑刹车】...")

    def _setup_file_logging(self):
        if not self.enable_file_log:
            rospy.loginfo("[SafeMPC] 文件日志已禁用。")
            return

        try:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_log_path = os.path.join(self.log_dir, f"safe_mpc_control_{stamp}.csv")
            self.txt_log_path = os.path.join(self.log_dir, f"safe_mpc_control_{stamp}.txt")

            self._csv_file = open(self.csv_log_path, 'w', encoding='utf-8', newline='')
            self._txt_file = open(self.txt_log_path, 'w', encoding='utf-8', buffering=1)
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'wall_time', 'ros_time', 'event', 'note',
                'x', 'y', 'yaw', 'v', 'ref_v', 'goal_dist',
                'state_received', 'traj_received', 'goal_reached', 'is_running',
                'accel_cmd_mps2', 'steer_cmd_rad',
                'pub_accel_pct', 'pub_brake_pct', 'pub_steer_pct'
            ])
            self._csv_file.flush()
        except Exception as e:
            rospy.logwarn("[SafeMPC] 文件日志初始化失败: %s", str(e))
            self.enable_file_log = False
            self.csv_log_path = None
            self.txt_log_path = None
            self._close_log_files()

    def _close_log_files(self):
        for file_attr in ('_csv_file', '_txt_file'):
            file_obj = getattr(self, file_attr, None)
            if file_obj is not None:
                try:
                    file_obj.close()
                except Exception:
                    pass
                setattr(self, file_attr, None)
        self._csv_writer = None

    def _on_shutdown(self):
        self._record_event("[SafeMPC] 节点退出，关闭日志文件。", event='shutdown')
        self._close_log_files()

    def _should_log_by_interval(self, attr_name, interval, now_sec):
        if interval <= 0.0:
            setattr(self, attr_name, now_sec)
            return True

        last_sec = getattr(self, attr_name, None)
        if last_sec is None or (now_sec - last_sec) >= interval:
            setattr(self, attr_name, now_sec)
            return True
        return False

    def _write_text_log(self, message):
        if not self.enable_file_log or self._txt_file is None:
            return
        wall_stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self._txt_file.write(f"[{wall_stamp}] {message}\n")
        self._txt_file.flush()

    def _write_csv_row(self, event, note="", accel_val=None, steer_val=None,
                       accel_pct=None, brake_pct=None, steer_pct=None):
        if not self.enable_file_log or self._csv_writer is None:
            return

        now = rospy.Time.now()
        wall_stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        ref_v = 0.0
        if self.ref_traj is not None and self.ref_traj.shape[1] > 0:
            ref_v = float(self.ref_traj[2, 0])

        goal_dist = "" if self._last_goal_dist is None else round(float(self._last_goal_dist), 4)
        row = [
            wall_stamp,
            f"{now.to_sec():.6f}",
            event,
            note,
            round(float(self.current_state.x), 4),
            round(float(self.current_state.y), 4),
            round(float(self.current_state.yaw), 4),
            round(float(self.current_state.v), 4),
            round(ref_v, 4),
            goal_dist,
            1 if self.state_received else 0,
            1 if self.traj_received else 0,
            1 if self.goal_reached else 0,
            1 if self.is_running else 0,
            "" if accel_val is None else round(float(accel_val), 4),
            "" if steer_val is None else round(float(steer_val), 4),
            "" if accel_pct is None else int(accel_pct),
            "" if brake_pct is None else int(brake_pct),
            "" if steer_pct is None else int(steer_pct),
        ]
        self._csv_writer.writerow(row)
        self._csv_file.flush()

    def _record_event(self, message, event='event'):
        self._write_text_log(message)
        self._write_csv_row(event=event, note=message)

    def _global_path_callback(self, msg):
        if msg.poses:
            # 记录真正的全局终点坐标
            self.global_end_pose = msg.poses[-1].pose.position
            log_msg = (
                "[SafeMPC] 已接收 global_path: 点数=%d, 终点=(%.2f, %.2f)" % (
                    len(msg.poses), self.global_end_pose.x, self.global_end_pose.y
                )
            )
            rospy.loginfo(log_msg)
            self._record_event(log_msg, event='path_ready')

    def _log_waiting_state(self):
        required_missing = []
        optional_missing = []

        if not self.state_received:
            required_missing.append('/vehicle_odom')
        if not self.traj_received or self.ref_lanes is None:
            required_missing.append('/mpc/reference_path')
        if self.global_end_pose is None:
            optional_missing.append('/global_path(终点停车未启用)')

        now_sec = rospy.Time.now().to_sec()
        if required_missing:
            log_msg = "[SafeMPC] 等待必要输入: %s" % ', '.join(required_missing)
            rospy.logwarn_throttle(self.wait_log_interval, log_msg)
            if self._should_log_by_interval('_last_wait_txt_log_sec', self.wait_log_interval, now_sec):
                self._write_text_log(log_msg)
        elif optional_missing:
            log_msg = "[SafeMPC] 控制已可运行，等待可选输入: %s" % ', '.join(optional_missing)
            rospy.loginfo_throttle(self.wait_log_interval, log_msg)
            if self._should_log_by_interval('_last_wait_txt_log_sec', self.wait_log_interval, now_sec):
                self._write_text_log(log_msg)

    def _estimate_output_percentages(self, accel_val, steer_val):
        """按父类映射规则估算输出百分比，用于摘要日志。"""
        current_v = self.current_state.v

        if accel_val >= 0.0:
            if current_v < self.creep_speed and accel_val < 0.1:
                accel_pct = 0
                brake_pct = self.creep_brake
            else:
                ratio = accel_val / self.max_acc_limit
                accel_pct = int(np.clip(ratio, 0.0, 1.0) * 100)
                brake_pct = 0
        else:
            decel = abs(accel_val)
            if decel > 0.1:
                ratio = decel / self.max_decel_limit
                percent = self.brake_deadzone + ratio * (100.0 - self.brake_deadzone)
                brake_pct = int(np.clip(percent, 0.0, 100.0))
            else:
                brake_pct = self.creep_brake
            accel_pct = 0

        steer_scale = self.max_steer_limit if abs(self.max_steer_limit) > 1e-6 else 1.0
        steer_ratio = steer_val / steer_scale
        steer_pct = int(np.clip(steer_ratio, -1.0, 1.0) * 100)
        return accel_pct, brake_pct, steer_pct

    def _control_loop(self, event):
        # 拦截：如果已被停止（如正在执行平滑刹车），不再执行控制闭环
        if not self.is_running:
            return

        self._log_waiting_state()

        if not (self.state_received and self.traj_received and self.ref_lanes is not None):
            self._last_goal_dist = None
            return

        if not self._control_ready_logged:
            self._control_ready_logged = True
            self._record_event(
                "[SafeMPC] /vehicle_odom 与 /mpc/reference_path 已就绪，开始执行 MPC 控制。",
                event='control_ready'
            )

        # 注入：全局终点判断逻辑
        if self.state_received and self.global_end_pose:
            dist = math.hypot(
                self.current_state.x - self.global_end_pose.x,
                self.current_state.y - self.global_end_pose.y
            )
            self._last_goal_dist = dist
            # 如果到达终点阈值内，触发自动平滑停车
            if dist < self.goal_dist_threshold:
                if not self.goal_reached:
                    log_msg = (
                        "[SafeMPC] 到达终点阈值内，触发自动停车。"
                        f" dist={dist:.2f}m threshold={self.goal_dist_threshold:.2f}m"
                    )
                    rospy.loginfo(log_msg)
                    self._record_event(log_msg, event='goal_reached')
                    self.goal_reached = True
                    self.smooth_stop(reason='goal_reached')
                return
        else:
            self._last_goal_dist = None

        # 放行：调用原始 MPC 的控制闭环
        super(SafeMPCRunner, self)._control_loop(event)

    def smooth_stop(self, reason='external_request'):
        """【核心要求2】完全复刻 PID 系统中的平滑舒适刹车逻辑"""
        if not self.is_running:
            log_msg = "[SafeMPC] 平滑停车已在执行或已完成，忽略重复停车请求。reason=%s" % reason
            rospy.loginfo(log_msg)
            self._record_event(log_msg, event='duplicate_stop_request')
            return

        self._active_stop_reason = reason
        log_msg = "[SafeMPC] 触发停车指令，开始执行平滑刹车。reason=%s" % reason
        rospy.loginfo(log_msg)
        self._record_event(log_msg, event='smooth_stop_start')
        self.is_running = False  # 切断父类的 MPC 周期计算

        target_brake = 30  # 目标舒适刹车力度 (30%)
        current_brake = 0
        rate = rospy.Rate(self.control_rate)  # 保持与 MPC 同频 (默认 20Hz)

        # 提取最后一帧的方向盘角度，防止停车时方向盘猛然回正造成车身晃动
        steer_scale = self.max_steer_limit if abs(self.max_steer_limit) > 1e-6 else 1.0
        steer_pct = int(np.clip(self._last_applied_steer / steer_scale, -1.0, 1.0) * 100)

        while current_brake < target_brake and not rospy.is_shutdown():
            current_brake += 1.0
            self._publish_raw_commands(0, int(current_brake), steer_pct)
            rate.sleep()

        rospy.sleep(1.0)
        self._publish_raw_commands(0, 0, 0)  # 最终彻底释放控制权
        end_msg = "[SafeMPC] 车辆已平稳停止，控制量归零。reason=%s" % reason
        rospy.loginfo(end_msg)
        self._record_event(end_msg, event='smooth_stop_end')

    def _publish_control_commands(self, accel_val, steer_val):
        accel_pct, brake_pct, steer_pct = self._estimate_output_percentages(accel_val, steer_val)
        super(SafeMPCRunner, self)._publish_control_commands(accel_val, steer_val)

        ref_v = 0.0
        if self.ref_traj is not None and self.ref_traj.shape[1] > 0:
            ref_v = float(self.ref_traj[2, 0])

        goal_text = "n/a" if self._last_goal_dist is None else f"{self._last_goal_dist:.2f}m"
        summary = (
            "[SafeMPC] pose=(%.2f, %.2f) v=%.2fm/s ref_v=%.2fm/s goal=%s | "
            "cmd[a=%.2fm/s^2 steer=%.3frad] -> out[accel=%d brake=%d steer=%d]"
        ) % (
            self.current_state.x, self.current_state.y, self.current_state.v, ref_v, goal_text,
            accel_val, steer_val, accel_pct, brake_pct, steer_pct
        )
        rospy.loginfo_throttle(self.status_log_interval, summary)

        now_sec = rospy.Time.now().to_sec()
        if self._should_log_by_interval('_last_control_csv_log_sec', self.control_file_log_interval, now_sec):
            self._write_csv_row(
                event='control',
                note='mpc_control_publish',
                accel_val=accel_val,
                steer_val=steer_val,
                accel_pct=accel_pct,
                brake_pct=brake_pct,
                steer_pct=steer_pct
            )
        if self._should_log_by_interval('_last_control_txt_log_sec', self.status_log_interval, now_sec):
            self._write_text_log(summary)

    def _publish_raw_commands(self, accel_pct, brake_pct, steer_pct):
        """绕过物理映射，直接向 Autoware 发布底层百分比控制指令"""
        acc_msg, brk_msg, str_msg = AccelCmd(), BrakeCmd(), SteerCmd()
        acc_msg.accel = accel_pct
        brk_msg.brake = brake_pct
        str_msg.steer = steer_pct

        now = rospy.Time.now()
        acc_msg.header.stamp = now
        brk_msg.header.stamp = now
        str_msg.header.stamp = now

        self.pub_accel.publish(acc_msg)
        self.pub_brake.publish(brk_msg)
        self.pub_steer.publish(str_msg)

        steer_scale = self.max_steer_limit if abs(self.max_steer_limit) > 1e-6 else 1.0
        steer_val = (float(steer_pct) / 100.0) * steer_scale
        goal_text = "n/a" if self._last_goal_dist is None else f"{self._last_goal_dist:.2f}m"
        summary = (
            "[SafeMPC][stop] reason=%s goal=%s v=%.2fm/s -> out[accel=%d brake=%d steer=%d]"
        ) % (
            self._active_stop_reason or 'unknown',
            goal_text,
            self.current_state.v,
            accel_pct,
            brake_pct,
            steer_pct
        )

        rospy.loginfo_throttle(self.stop_log_interval, summary)
        self._write_csv_row(
            event='smooth_stop',
            note=self._active_stop_reason or 'smooth_stop',
            accel_val=0.0,
            steer_val=steer_val,
            accel_pct=accel_pct,
            brake_pct=brake_pct,
            steer_pct=steer_pct
        )
        if self._should_log_by_interval('_last_stop_txt_log_sec', self.stop_log_interval, now.to_sec()):
            self._write_text_log(summary)


if __name__ == '__main__':
    try:
        node = SafeMPCRunner()
        # 兼容 PID 的外部键盘循环监听机制
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            key = getKey(timeout=0.05)
            if key == 'q':
                node._record_event("[SafeMPC] 键盘收到 q，触发人工平滑停车。", event='keyboard_stop_request')
                node.smooth_stop(reason='keyboard_q')
                print("\n程序已退出，车辆已平稳停止。")
                break
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
