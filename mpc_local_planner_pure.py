import numpy as np
import math
from scipy.interpolate import splprep, splev
from structs import Location, Transform, Rotation, Waypoint, RoadOption

# ================= 配置参数 =================
NP = 30         # MPC 预测步长
DT = 0.05       # MPC 时间步长 (匹配 ROS 控制频率 20Hz)
WB = 2.875      # 车辆轴距 (根据你的 mpc_runner_node 参数调整)

# --- 离散平滑参数 ---
SMOOTH_WEIGHT = 6.0       
DATA_WEIGHT = 2.5          

# --- 弯道策略参数 ---
USE_CORNER_STRATEGY = True
NUDGE_AMOUNT = 0         
APEX_CUT_AMOUNT = 0     
# ===========================================

class MPCLocalPlanner:
    """
    高级 MPC 规划器 (纯 Python 版，适配 ROS)
    包含全局离散平滑、静态限速预计算、基于最小视野的动态重采样。
    """
    def __init__(self, opt_dict={}):
        # --- 内部状态变量 ---
        self._current_transform = Transform()
        self._current_speed = 0.0 # m/s
        
        # --- 物理约束配置 ---
        self._max_lateral_accel = opt_dict.get('max_lateral_accel', 6.0)
        self._max_decel = opt_dict.get('max_decel', 1.5)
        self._max_accel = opt_dict.get('max_accel', 1.5)
        self._max_speed = opt_dict.get('target_speed', 10.0) # m/s
        self._system_delay = opt_dict.get('system_delay', 0.15)
        
        # 静态全图数据结构
        self._global_path = None 
        self._final_destination = None
        self._last_closest_idx = 0
        
    def update_vehicle_state(self, x, y, z, yaw_rad, v):
        """【ROS 数据注入接口】"""
        self._current_transform.location = Location(x, y, z)
        self._current_transform.rotation = Rotation(yaw=math.degrees(yaw_rad))
        self._current_speed = v

    def set_global_plan(self, current_plan):
        """初始化阶段：离散平滑与静态预计算"""
        print("[Planner] 正在构建全局静态参考线 (Discrete Smooth)...")
        if not current_plan:
            return

        # 1. 提取稀疏路点
        raw_x = [wp.transform.location.x for wp, _ in current_plan]
        raw_y = [wp.transform.location.y for wp, _ in current_plan]
        
        self._final_destination = Location(x=raw_x[-1], y=raw_y[-1])

        # 2. 去重
        unique_x, unique_y = [raw_x[0]], [raw_y[0]]
        for i in range(1, len(raw_x)):
            if math.hypot(raw_x[i]-unique_x[-1], raw_y[i]-unique_y[-1]) > 0.1:
                unique_x.append(raw_x[i])
                unique_y.append(raw_y[i])

        if len(unique_x) < 4:
            print("[Planner] 错误：路径太短。")
            return

        # 3. 弯道策略
        if USE_CORNER_STRATEGY:
            anchors_x, anchors_y = self._apply_corner_strategy(unique_x, unique_y)
        else:
            anchors_x, anchors_y = unique_x, unique_y

        # 4. 离散平滑
        smooth_x, smooth_y = self._discrete_smooth(anchors_x, anchors_y)

        # 5. 高精度样条插值 (0.1m 分辨率)
        try:
            tck, u = splprep([smooth_x, smooth_y], k=3, s=0) 
            total_dist = sum(math.hypot(x2-x1, y2-y1) for x1, y1, x2, y2 
                           in zip(smooth_x[:-1], smooth_y[:-1], smooth_x[1:], smooth_y[1:]))
            num_points = int(total_dist / 0.1) 
            
            u_new = np.linspace(0, 1, num_points)
            
            x_fine, y_fine = splev(u_new, tck)
            dx, dy = splev(u_new, tck, der=1)
            ddx, ddy = splev(u_new, tck, der=2)
            
            yaw_fine = np.unwrap(np.arctan2(dy, dx))
            curvature_fine = (dx * ddy - dy * ddx) / np.power(dx**2 + dy**2, 1.5)
            
            # 6. 静态速度规划 (弯道限速 + 纵向加减速平滑)
            abs_k = np.abs(curvature_fine)
            abs_k[abs_k < 1e-4] = 1e-4
            v_limit = np.sqrt(self._max_lateral_accel / abs_k)
            v_limit = np.clip(v_limit, 0, self._max_speed) 
            
            dist_step = 0.1 
            for i in range(len(v_limit) - 2, -1, -1):
                v_next = v_limit[i+1]
                v_allowable = math.sqrt(v_next**2 + 2 * self._max_decel * dist_step)
                v_limit[i] = min(v_limit[i], v_allowable)

            v_limit[0] = 0.0
            for i in range(len(v_limit) - 1):
                v_curr = v_limit[i]
                v_allowable = math.sqrt(v_curr**2 + 2 * self._max_accel * dist_step)
                v_limit[i+1] = min(v_limit[i+1], v_allowable)

            # 存储高精地图字典
            self._global_path = {
                'x': x_fine, 'y': y_fine, 'yaw': yaw_fine, 'k': curvature_fine,
                'v_limit': v_limit, 'length': len(x_fine),
                'yaw_unwrapped': np.unwrap(yaw_fine),
                'idx_array': np.arange(len(x_fine), dtype=float)
            }
            self._last_closest_idx = 0
            print(f"[Planner] 全局参考线构建完成！点数: {len(x_fine)}")
            
        except Exception as e:
            print(f"[Planner] 路径生成失败: {e}")
            self._global_path = None

    def _apply_corner_strategy(self, x_raw, y_raw):
        """晚切弯策略 (Late Apex)"""
        x_mod = np.array(x_raw, copy=True)
        y_mod = np.array(y_raw, copy=True)
        yaws = [math.atan2(y_raw[i+1]-y_raw[i], x_raw[i+1]-x_raw[i]) for i in range(len(x_raw)-1)]
        yaws.append(yaws[-1])
        
        is_corner = np.zeros(len(x_raw), dtype=bool)
        turn_direction = np.zeros(len(x_raw)) 
        turn_intensity = np.zeros(len(x_raw)) 
        
        window = 3
        for i in range(window, len(x_raw)-window):
            diff = yaws[i+window] - yaws[i-window]
            diff = (diff + np.pi) % (2*np.pi) - np.pi
            if abs(diff) > np.deg2rad(30): 
                is_corner[i] = True
                turn_direction[i] = np.sign(diff)
                turn_intensity[i] = np.clip(abs(diff) / (np.pi / 2), 0.5, 1.5)

        nudge_window = 8
        modified_mask = np.zeros(len(x_raw), dtype=bool)

        for i in range(len(x_raw)):
            if is_corner[i]:
                strength = turn_intensity[i]
                if not modified_mask[i]:
                    shift_angle_in = yaws[i] + turn_direction[i] * (np.pi / 2)
                    dist = APEX_CUT_AMOUNT * strength
                    x_mod[i] = x_raw[i] + math.cos(shift_angle_in) * dist
                    y_mod[i] = y_raw[i] + math.sin(shift_angle_in) * dist
                    modified_mask[i] = True

                for j in range(1, nudge_window + 1):
                    idx = i - j
                    if idx >= 0 and not is_corner[idx] and not modified_mask[idx]: 
                        outward_angle = yaws[idx] - turn_direction[i] * (np.pi / 2)
                        ratio = (j - 1) / nudge_window 
                        decay_factor = 0.5 * (1 + math.cos(ratio * math.pi))
                        shift_dist = NUDGE_AMOUNT * decay_factor * strength
                        x_mod[idx] = x_raw[idx] + math.cos(outward_angle) * shift_dist
                        y_mod[idx] = y_raw[idx] + math.sin(outward_angle) * shift_dist
                        modified_mask[idx] = True
        return x_mod, y_mod

    def _discrete_smooth(self, x_in, y_in):
        """有限元离散点平滑"""
        path_len = len(x_in)
        x_out, y_out = np.copy(x_in), np.copy(y_in)
        denominator = DATA_WEIGHT + 2 * SMOOTH_WEIGHT
        for _ in range(500): 
            for i in range(1, path_len - 1):
                x_out[i] = (DATA_WEIGHT * x_in[i] + SMOOTH_WEIGHT * (x_out[i-1] + x_out[i+1])) / denominator
                y_out[i] = (DATA_WEIGHT * y_in[i] + SMOOTH_WEIGHT * (y_out[i-1] + y_out[i+1])) / denominator
        return x_out, y_out

    def run_step(self, target_speed):
        """【运行阶段】动态切片，包含防画龙机制"""
        if self._global_path is None:
            return (None, None, None, False, None, None), None

        # 1. 定位与寻点
        veh_loc = self._current_transform.location
        current_speed = self._current_speed
        
        search_range = 500 
        start_search = self._last_closest_idx
        end_search = min(start_search + search_range, self._global_path['length'])
        
        dx = self._global_path['x'][start_search:end_search] - veh_loc.x
        dy = self._global_path['y'][start_search:end_search] - veh_loc.y
        dists_sq = dx**2 + dy**2
        
        if len(dists_sq) > 0:
            self._last_closest_idx = start_search + np.argmin(dists_sq)
        
        # 2. 精确浮点投影与延迟补偿
        idx = self._last_closest_idx
        next_idx = min(idx + 1, self._global_path['length'] - 1)
        
        if next_idx > idx:
            dx_path = self._global_path['x'][next_idx] - self._global_path['x'][idx]
            dy_path = self._global_path['y'][next_idx] - self._global_path['y'][idx]
            dx_veh = veh_loc.x - self._global_path['x'][idx]
            dy_veh = veh_loc.y - self._global_path['y'][idx]
            path_sq_len = dx_path**2 + dy_path**2 + 1e-6
            proj_frac = max(0.0, min(1.0, (dx_veh * dx_path + dy_veh * dy_path) / path_sq_len))
            exact_start_idx = idx + proj_frac
        else:
            exact_start_idx = float(idx)

        idx_delay = (current_speed * self._system_delay) / 0.1
        exact_start_idx += idx_delay

        # ========================================================
        # 3. 核心修复：最小视野重采样 (防止低速近视画龙)
        # ========================================================
        MIN_LOOKAHEAD_SPEED = 5.0 # 强制让 MPC 至少看向 5.0 m/s * DT 远的地方
        sampling_speed = max(current_speed, MIN_LOOKAHEAD_SPEED)
        ds = sampling_speed * DT
        step_idx_float = ds / 0.1

        max_idx = self._global_path['length'] - 1
        float_indices = exact_start_idx + np.arange(NP + 1) * step_idx_float
        float_indices = np.minimum(float_indices, max_idx) 

        idx_array = self._global_path['idx_array']
        px_interp       = np.interp(float_indices, idx_array, self._global_path['x'])
        py_interp       = np.interp(float_indices, idx_array, self._global_path['y'])
        pyaw_interp     = np.interp(float_indices, idx_array, self._global_path['yaw_unwrapped'])
        pk_interp       = np.interp(float_indices, idx_array, self._global_path['k'])
        pv_limit_interp = np.interp(float_indices, idx_array, self._global_path['v_limit'])

        mpc_traj = []
        mpc_k = []
        for i in range(NP + 1):
            final_yaw = self._normalize_angle(pyaw_interp[i])
            if i > 0:
                diff = final_yaw - mpc_traj[i-1][2]
                if diff > math.pi: final_yaw -= 2 * math.pi
                elif diff < -math.pi: final_yaw += 2 * math.pi
            
            final_ref_v = min(target_speed, pv_limit_interp[i])
            mpc_traj.append((px_interp[i], py_interp[i], final_yaw, final_ref_v))
            mpc_k.append(pk_interp[i])
        
        # 4. 封装输出
        delta_refs = [math.atan(WB * k) for k in mpc_k]
        corridor_boundaries = [(2.5, 2.5)] * len(mpc_traj)
        boundary_types = [('hard', 'hard')] * len(mpc_traj)
        
        return (mpc_traj, corridor_boundaries, boundary_types, False, mpc_k, delta_refs), None

    def _normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def done(self):
        """到达终点判断"""
        if self._global_path is None or not self._final_destination:
            return False
        
        dist = self._current_transform.location.distance(self._final_destination)
        if self._current_speed < 1.0 and dist < 3.0:
            return True
        return self._last_closest_idx >= self._global_path['length'] - 10