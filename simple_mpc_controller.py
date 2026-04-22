import numpy as np
import math
import casadi as ca
import time
from scipy.linalg import solve_discrete_are
from scipy.stats import norm  # 新增: 用于概率分布计算

# ================== MPC 模型常量 ==================
NX = 4  # 状态变量维度: [x, y, v, yaw]
NU = 2  # 控制输入维度: [a, delta]
NP = 30  # 预测步长 (Prediction Horizon) -> 看 2.0秒
NC = 10  # 控制步长 (Control Horizon)    -> 控 1.0秒
DT = 0.05  # 时间步长


class State:
    """一个简单的数据结构，用于表示车辆的当前状态。"""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class SimpleMPController:
    """
    【基准版 MPC (含 Tube & LQR 基础)】
    - 无 RTI (只基于参考轨迹线性化)
    - 含 延时补偿 (Kinematic Propagation)
    - 含 Tube 约束 (Parameter-based Margin)
    - 含 LQR 辅助反馈
    """
    def __init__(self, wb, q_d, q_v, q_yaw, q_d_dot, q_yaw_dot, R, Rd, max_steer, q_d_d_dot, q_yaw_d_dot, max_accel=2.0,max_d_accel=2.0,max_d_steer=10.0,
                 system_delay=0.0, tube_config=None, **kwargs):
        self.WB = wb
        self.max_steer = max_steer
        self.system_delay = system_delay # 系统延时 (秒)
        
        # 0. Tube 配置 (新)
        # 默认关闭，必须显式传入配置开启
        self.tube_config = tube_config if tube_config else {}
        self.use_tube = self.tube_config.get('enable_tube', False)
        self.tube_radius = self.tube_config.get('tube_radius', 0.0) # 显式管宽 (Static Margin)
        self.use_lqr = self.tube_config.get('enable_lqr', False)
        self.decay_lambda = self.tube_config.get('decay_lambda', 0.95)         # FASD 帧间衰减
        self.lambda_horz = self.tube_config.get('lambda_horz', 0.85)          # 预测时域衰减系数 (针对反馈项)
        self.lambda_model = self.tube_config.get('lambda_model', 0.90)  
        # 【新增】LQR 控制余量 (Control Headroom)
        # 即使不开 Tube，留一点余量也是好的，防止数值解在边界震荡
        if self.use_lqr:
            # 预留 0.2 m/s^2 加速度余量
            self.acc_margin = 0.2 
            # 预留约 10%~15% 的转向余量 (例如 0.05~0.08 rad)
            self.steer_margin = self.max_steer * 0.15 
        else:
            self.acc_margin = 0.0
            self.steer_margin = 0.0
        # [新增] Tube 模式选择 ('empirical' 或 'probabilistic')
        # 默认使用 'empirical' 以保持原来的行为
        self.tube_mode = self.tube_config.get('tube_mode', 'empirical')
        
        # =========================================================
        # --- 核心概率参数 (白盒物理失配模型) ---
        # =========================================================
        
        # 1. 纵向与航向失配系数
        # 对应公式: σ_x = c_vx * |v|
        self.vx_scaling = self.tube_config.get('vx_scaling', 0.02)
        # 对应公式: σ_yaw = c_yaw * |v * δ|
        self.yaw_rate_scaling = self.tube_config.get('yaw_rate_scaling', 0.05)
        
        # 2. 横向动态失配系数 (核心)
        self.vy_scaling = self.tube_config.get('vy_scaling', 0.08)  
        
        # 对应公式项 2 (离心力外扩): c_geom * |v^2 * κ| 
        # 统一命名为 centrifugal_scaling 保持与外层配置一致
        self.centrifugal_scaling = self.tube_config.get('centrifugal_scaling', 1.5)
        
        # 3. 远期预测置信度折扣 (Credibility Discount)
        # 对应公式: W_k = W_act + (γ^k) * W_phys
        self.lambda_model = self.tube_config.get('lambda_model', 0.95)
            
        # =========================================================
        # 4. 置信区间参数 (Z-score)
        # =========================================================
        # 接收外部传入的置信度 (例如 0.95)，并计算正态分布下的 Z 分数
        self.confidence_level = kwargs.get('confidence_level', 0.95)
        # 95% 置信度对应的双边 Z 分数约为 1.96
        self.z_score = norm.ppf((1.0 + self.confidence_level) / 2.0)
            
        # [新增] 反馈项的状态记忆与预测历史

        self.last_predicted_state = None   # 用于计算预测误差的上一帧预测数据 ([x, y, v, yaw])
        
        # 1. 权重矩阵 (固定)
        self.q_d = q_d
        self.q_v = q_v
        self.q_yaw = q_yaw
        self.q_d_dot = q_d_dot
        self.q_yaw_dot = q_yaw_dot
        
        # 确保 R, Rd 是对角阵
        self.R = np.diag(R) if not isinstance(R, np.ndarray) else R
        self.Rd = np.diag(Rd) if not isinstance(Rd, np.ndarray) else Rd
        self.max_accel = max_accel
        self.max_d_steer_step = max_d_steer
        # [新增] 二阶平滑权重 (Jerk Cost)
        self.q_d_d_dot = q_d_d_dot       # 纵向加加速度权重
        self.q_yaw_d_dot = q_yaw_d_dot   # 转向加加速度权重
        
        # [新增] 纵向加速度变化率硬约束 (例如 0.02s 内最多变 0.1 m/s^2)
        # 注意：这里的 max_d_accel 通常是指每秒的变化率，如果是每步，需要 max_d_accel * DT
        self.max_d_accel_step = max_d_accel * DT 
        self.max_d_steer_step = max_d_steer * DT
        # [新增] 历史控制量缓存 [u_{k-1}, u_{k-2}]
        # 初始化为 [acc, steer]
        self.u_history = [np.zeros(NU), np.zeros(NU)]
        # 2. 计算 LQR 增益 (辅助控制器)
        if self.use_lqr:
            # 【修复】初始化时传入一个标称速度 (比如 10.0 m/s)
            # 反正这个值在运行的第一帧进入 solve() 时，就会立刻被真实的 state.v 覆盖更新
            self.K_lqr = self._compute_lqr_gain(current_v=10.0)
        else:
            self.K_lqr = None

        # 3. 预构建优化问题
        self._build_optimization_problem()
    def _compute_lqr_gain(self, current_v):
        """
        [修改] 动态计算 LQR 增益 (Gain Scheduling)
        Args:
            current_v: 当前车辆速度 (m/s)
        """
        # 1. 速度保护：防止停车时(v=0)增益爆炸
        # 设定一个最小计算速度，比如 2.0 m/s
        # 意味着：0-2m/s 期间，我们使用 2m/s 的增益，保持稳定
        v_calc = max(abs(current_v), 2.0)
        
        # 2. 线性化系统矩阵 (Linearized Dynamics)
        # x_{k+1} = A * x_k + B * u_k
        A = np.eye(4)
        A[0, 2] = DT            # x += v * dt
        A[1, 3] = v_calc * DT   # y += v * phi * dt (小角度近似)
        A[2, 2] = 1.0           # v 保持
        A[3, 3] = 1.0           # yaw 保持
        
        B = np.zeros((4, 2))
        B[2, 0] = DT            # v += a * dt
        B[3, 1] = (v_calc / self.WB) * DT  # yaw += v/L * tan(delta) * dt
        
        # 3. LQR 权重矩阵
        Q_lqr = np.diag([self.q_d, self.q_d, self.q_v, self.q_yaw])
        R_lqr = self.R

        # 4. 解 Riccati 方程
        try:
            P = solve_discrete_are(A, B, Q_lqr, R_lqr)
            # K = (R + B^T P B)^-1 B^T P A
            K = np.linalg.inv(R_lqr + B.T @ P @ B) @ (B.T @ P @ A)
            return K
        except Exception as e:
            # print(f"[LQR Fail] v={current_v:.1f}: {e}")
            return np.zeros((2, 4))
    # def _compute_lqr_gain(self):
    #     """
    #     计算标称工况下的 LQR 增益 (Static K)。
    #     用于 Tube MPC 的辅助反馈层: u = u_mpc + K * (x_meas - x_mpc)
    #     """
    #     # 线性化工作点 (v=10m/s, phi=0, delta=0)
    #     v_nom = 10.0
    #     A = np.eye(4)
    #     A[0, 2] = DT  # x += v * dt (linearized at phi=0)
    #     A[1, 2] = 0.0 # y 不随 v 线性变 (couple with phi, ignored here or 0)
    #     A[1, 3] = v_nom * DT # y += v * phi * dt (linearized)
    #     A[3, 3] = 1.0 
        
    #     B = np.zeros((4, 2))
    #     B[2, 0] = DT            # v_next += a * dt
    #     B[3, 1] = v_nom / self.WB * DT  # phi_next += v/L * delta * dt

    #     # LQR Cost Matrices (可以直接沿用 MPC 的权重)
    #     Q_lqr = np.diag([self.q_d, self.q_d, self.q_v, self.q_yaw]) # X, Y用同样的权重近似
    #     R_lqr = self.R

    #     # 解 Riccati 方程
    #     try:
    #         P = solve_discrete_are(A, B, Q_lqr, R_lqr)
    #         K = np.linalg.inv(R_lqr + B.T @ P @ B) @ (B.T @ P @ A)
    #         print(f"[TubeMPC] LQR Gain Computed:\n{K}")
    #         # u_feedback = -K @ error_state
    #         return K
    #     except Exception as e:
    #         print(f"[TubeMPC] LQR Computation Failed: {e}, using zero gain.")
    #         return np.zeros((2, 4))
            
    def _get_discrete_system_matrices(self, v, phi, delta):
        """
        获取一般的离散时间线性化系统矩阵 A, B
        用于概率协方差传播: P_{k+1} = (A+BK) P_k (A+BK)^T + Q
        """
        A = np.eye(NX)
        B = np.zeros((NX, NU))
        
        # 避免奇异值
        if abs(v) < 0.1: v = 0.1
        
        # 运动学模型 Jacobian
        # x' = v cos(phi)
        # y' = v sin(phi)
        # v' = a
        # phi' = v/L tan(delta)
        
        # A 矩阵 (df/dx)
        # partial f_x / partial phi = -v sin(phi)
        A[0, 3] = -v * math.sin(phi) * DT
        # partial f_x / partial v   = cos(phi)
        A[0, 2] = math.cos(phi) * DT
        
        # partial f_y / partial phi = v cos(phi)
        A[1, 3] = v * math.cos(phi) * DT
        # partial f_y / partial v   = sin(phi)
        A[1, 2] = math.sin(phi) * DT
        
        # partial f_phi / partial v = 1/L tan(delta)
        A[3, 2] = (1.0 / self.WB) * math.tan(delta) * DT
        
        # B 矩阵 (df/du)
        # partial f_v / partial a = 1
        B[2, 0] = DT
        
        # partial f_phi / partial delta = v/L * sec^2(delta)
        # sec^2(x) = 1 + tan^2(x)
        sec2 = 1.0 + math.tan(delta)**2
        B[3, 1] = (v / self.WB) * sec2 * DT
        
        return A, B
    def _compute_probabilistic_margins(self, xref, curvatures, current_v):
        """
        【终极纯净版】多源不确定性融合的 Tube 边界计算 (Multi-source Uncertainty Fusion)
        
        不确定性来源 (白盒解耦)：
        1. [P0] 初始状态底噪: 纯传感器观测噪声 (如 GPS 定位误差)
        2. [W_act] 执行器不确定性: W_act = B Σ_u B^T
        3. [W_phys] 物理模型失配: 轮胎侧偏 + 离心力外扩 + 固定外部干扰 (标准差直接相加，即 rho=1 最坏情况耦合)
        
        传播方程 (闭环):
            P_{k+1} = (A - BK) P_k (A - BK)^T + W_act + (γ^k) * W_phys
        """
        margins = []
        
        # =========================================================
        # 1. 参数读取与配置
        # =========================================================
        sigma_a = self.tube_config.get('actuator_std_a', 0.15) 
        sigma_delta = self.tube_config.get('actuator_std_delta', 0.005)
        Sigma_u = np.diag([sigma_a**2, sigma_delta**2])
        
        # 【新增 1】读取传感器固定底噪标准差
        static_std = self.tube_config.get('static_noise_std', [0.02, 0.02, 0.1, 0.01])
        
        # 【新增 2】读取固定的外部环境随机噪声 (如微小路面侧倾、阵风)
        external_static_noise = self.tube_config.get('external_static_noise', 0.001)
        
        self.max_sigma_lateral = 0.4 

        # =========================================================
        # 2. 初始协方差 P0 (纯净的传感器底噪常数矩阵，杜绝系统震荡)
        # =========================================================
        P = np.diag([
            static_std[0]**2,  # x 观测方差
            static_std[1]**2,  # y 观测方差
            static_std[2]**2,  # v 观测方差
            static_std[3]**2   # yaw 观测方差
        ])

        # LQR 增益 (闭环收敛的核心)
        if self.use_lqr:
            K = self._compute_lqr_gain(current_v)
        else:
            K = np.zeros((NU, NX))

        # =========================================================
        # 3. 时域传播循环
        # =========================================================
        for t in range(NP + 1):
            ref_idx = min(t, NP)
            v_ref = xref[2, ref_idx]
            phi_ref = xref[3, ref_idx]
            k_ref = curvatures[ref_idx] if curvatures is not None else 0.0
            delta_ref = math.atan(self.WB * k_ref)
            
            A, B = self._get_discrete_system_matrices(v_ref, phi_ref, delta_ref)
            
            # =========================================================
            # 4. 过程噪声 (Process Noise)
            # =========================================================
            
            # --- 源 1: 执行器噪声投影 (瞬间只影响 v 和 yaw) ---
            W_act = B @ Sigma_u @ B.T
            
            # --- 源 2: 物理模型失配 (轮胎侧偏 + 离心力外扩 + 固定外部干扰) ---
            # 【核心修正】由于轮胎侧偏和离心力同源于侧向加速度，存在极强正相关。
            # 采用 rho=1 的最坏情况耦合，标准差直接相加后再平方！
            sigma_slip = self.vy_scaling * abs(v_ref * delta_ref)
            sigma_centrifugal = self.centrifugal_scaling * (v_ref**2) * abs(k_ref)
            
            sigma_sq_y = (sigma_slip + sigma_centrifugal + external_static_noise)**2
            
            W_phys = np.diag([
                (self.vx_scaling * abs(v_ref))**2,                   # x: 纵向失配
                sigma_sq_y,                                          # y: 横向高耦合失配
                0.0,                                                 # v: 由 W_act 覆盖
                (self.yaw_rate_scaling * abs(v_ref * delta_ref))**2  # yaw: 转向响应滞后
            ])
            
            # --- 融合: 置信度折扣 (Credibility Discount) ---
            gamma_i = self.lambda_model ** t
            W_k = W_act + (W_phys * gamma_i)
            
            # =========================================================
            # 5. 边界计算与平滑截断 (只截断输出，绝不污染协方差矩阵 P)
            # =========================================================
            sin_phi = math.sin(phi_ref)
            cos_phi = math.cos(phi_ref)
            H = np.array([[-sin_phi, cos_phi, 0, 0]]) 
            
            # 计算横向方差与标准差
            var_ed = H @ P @ H.T
            sigma_ed = math.sqrt(max(0, var_ed.item()))
            
            # 单独对输出的 Margin 进行安全上限截断，保护 MPC 优化器不无解
            raw_margin = self.z_score * sigma_ed
            capped_margin = min(raw_margin, self.z_score * self.max_sigma_lateral)
            margins.append(capped_margin)
            
            # =========================================================
            # 6. 闭环协方差传播 P_{k+1} = (A-BK) P_k (A-BK)^T + W_k
            # =========================================================
            if t < NP:
                A_cl = A - B @ K
                # 自由、真实地让协方差按照物理定律生长与收敛
                P_next = A_cl @ P @ A_cl.T + W_k
                
                P = P_next
                
        return margins
    def _build_optimization_problem(self):
            """构建 CasADi Opti 计算图 (完整版 - 含 Tube, 二阶平滑 Cost 与 Slew Rate 硬约束)"""
            self.opti = ca.Opti()
            
            # =========================================
            # 1. 定义变量
            # =========================================
            # 状态轨迹 X: [4, NP+1] -> (x, y, v, yaw)
            self.x_var = self.opti.variable(NX, NP + 1)
            # 控制轨迹 U: [2, NC]   -> (acc, delta)
            self.u_var = self.opti.variable(NU, NC)
            
            # =========================================
            # 2. 定义参数 (运行时可变)
            # =========================================
            self.x0_param = self.opti.parameter(NX)            # 初始状态
            self.xref_param = self.opti.parameter(NX, NP + 1)  # 参考轨迹
            self.last_u_param = self.opti.parameter(NU)        # 上一帧控制量 u_{k-1}
            
            # [新增] 再前一时刻的控制量参数 u_{k-2} (用于计算二阶差分)
            self.second_last_u_param = self.opti.parameter(NU)
            
            # Tube 边界收缩参数 (Parameter-based)
            self.safety_margin_param = self.opti.parameter(NP + 1) 
            
            # 车道边界参数 (半宽)
            self.lane_width_param = self.opti.parameter(NP + 1) 
            
            # =========================================
            # 3. 目标函数 (Cost Function)
            # =========================================
            cost = 0.0
            
            # --- A. 状态追踪 Cost ---
            for t in range(NP + 1):
                yaw_ref = self.xref_param[3, t]
                
                dx = self.x_var[0, t] - self.xref_param[0, t]
                dy = self.x_var[1, t] - self.xref_param[1, t]
                dv = self.x_var[2, t] - self.xref_param[2, t]
                dyaw = self.x_var[3, t] - self.xref_param[3, t]
                
                # 横向误差 e_d (投影到 Frenet 法向)
                e_d = -ca.sin(yaw_ref) * dx + ca.cos(yaw_ref) * dy
                
                cost += self.q_d * (e_d**2)
                cost += self.q_v * (dv**2)
                cost += self.q_yaw * (dyaw**2)

            # --- B. 控制量 Cost (含平滑项) ---
            for t in range(NC):
                # 1. 控制量大小惩罚
                cost += self.R[0, 0] * (self.u_var[0, t]**2)
                cost += self.R[1, 1] * (self.u_var[1, t]**2)
                
                # 2. 一阶差分惩罚 (Slew Rate Cost)
                if t == 0:
                    d_u = self.u_var[:, t] - self.last_u_param
                else:
                    d_u = self.u_var[:, t] - self.u_var[:, t-1]
                
                cost += self.Rd[0, 0] * (d_u[0]**2)
                cost += self.Rd[1, 1] * (d_u[1]**2)

                # [新增] 3. 二阶差分惩罚 (Jerk / Smoothness Cost)
                # 目标：最小化加速度的变化率 (Jerk) 和 转向角速度的变化率
                # 公式: diff2 = u_t - 2*u_{t-1} + u_{t-2}
                if t == 0:
                    # u_0 - 2*u_{-1} + u_{-2}
                    dd_u = self.u_var[:, t] - 2 * self.last_u_param + self.second_last_u_param
                elif t == 1:
                    # u_1 - 2*u_0 + u_{-1}
                    dd_u = self.u_var[:, t] - 2 * self.u_var[:, t-1] + self.last_u_param
                else:
                    # u_t - 2*u_{t-1} + u_{t-2}
                    dd_u = self.u_var[:, t] - 2 * self.u_var[:, t-1] + self.u_var[:, t-2]
                
                cost += self.q_d_d_dot * (dd_u[0]**2)    # 纵向二阶 (Jerk)
                cost += self.q_yaw_d_dot * (dd_u[1]**2)  # 横向二阶 (Steering Jerk)

            self.opti.minimize(cost)
            
            # =========================================
            # 4. 约束条件 (Constraints)
            # =========================================
            
            # A. 初始状态约束
            self.opti.subject_to(self.x_var[:, 0] == self.x0_param)
            
            # B. 动力学模型约束 (Multiple Shooting)
            for t in range(NP):
                u_t = self.u_var[:, t] if t < NC else self.u_var[:, NC-1]
                x_curr = self.x_var[:, t]
                x_next = self.x_var[:, t+1]
                
                x_pred = x_curr[0] + x_curr[2] * ca.cos(x_curr[3]) * DT
                y_pred = x_curr[1] + x_curr[2] * ca.sin(x_curr[3]) * DT
                v_pred = x_curr[2] + u_t[0] * DT
                yaw_pred = x_curr[3] + (x_curr[2] / self.WB) * ca.tan(u_t[1]) * DT
                
                self.opti.subject_to(x_next[0] == x_pred)
                self.opti.subject_to(x_next[1] == y_pred)
                self.opti.subject_to(x_next[2] == v_pred)
                self.opti.subject_to(x_next[3] == yaw_pred)

            # C. 物理幅值约束 (Absolute Limits)
            # 加速度 [-2.0, 2.0]
            acc_limit =self.max_accel - self.acc_margin
            self.opti.subject_to(self.opti.bounded(-acc_limit, self.u_var[0, :], acc_limit))
            # 转向角: [-max_steer, max_steer]
            steer_limit = self.max_steer - self.steer_margin
            self.opti.subject_to(self.opti.bounded(-steer_limit, self.u_var[1, :], steer_limit))
            
            # [新增] D. 控制量变化率硬约束 (Slew Rate Hard Constraints)
            # 这是为了防止物理上无法执行的突变 (如 0.02秒内油门从0变到100%)
            for t in range(NC):
                if t == 0:
                    d_u = self.u_var[:, t] - self.last_u_param
                else:
                    d_u = self.u_var[:, t] - self.u_var[:, t-1]
                
                # (1) 纵向加速度变化率硬约束
                # 限制每一步的加速度变化量: -max_step <= da <= max_step
                self.opti.subject_to(self.opti.bounded(
                    -self.max_d_accel_step, 
                    d_u[0], 
                    self.max_d_accel_step
                ))
                
                # (2) 转向角变化率硬约束 (可选，如果 Rd 够大通常不需要硬约束，防止无解)
                # 如果加上，可以确保方向盘不会打得比物理伺服电机还快
                self.opti.subject_to(self.opti.bounded(-self.max_d_steer_step, d_u[1], self.max_d_steer_step))

            # E. 管路安全约束 (Tube Safety Constraints)
            for t in range(NP + 1):
                yaw_ref = self.xref_param[3, t]
                dx = self.x_var[0, t] - self.xref_param[0, t]
                dy = self.x_var[1, t] - self.xref_param[1, t]
                
                e_d = -ca.sin(yaw_ref) * dx + ca.cos(yaw_ref) * dy
                
                limit = self.lane_width_param[t] - self.safety_margin_param[t]
                self.opti.subject_to(self.opti.bounded(-limit, e_d, limit))

            # =========================================
            # 5. 配置求解器
            # =========================================
            p_opts = {
                'expand': True,
                'print_time': False,
                'error_on_fail': False 
            }
            s_opts = {
                'max_iter': 100, 
                'print_level': 0, 
                'sb': 'yes',
                'tol': 1e-4,
                'print_frequency_iter': 1000,
                'print_timing_statistics': 'no'
            }
            self.opti.solver('ipopt', p_opts, s_opts)

    

    def solve_maximum_safe_velocity(self, max_allowable_margin, curvatures, v_max_search=50.0):
        """
        【核心算法】基于二分法的最大安全速度反解
        Args:
            max_allowable_margin: 允许的最大不确定性宽度 (即 Predicted_Margin 的上限，例如 0.2m)
            curvatures: 当前路径的曲率剖面 (用于计算模型失配)
            v_max_search: 搜索上限 (m/s)
        Returns:
            v_safe: 满足 Predicted_Margin <= max_allowable_margin 的最大允许速度
        """
        # 1. 快速检查：如果最高速产生的不确定性都在允许范围内，直接返回最高速
        margin_at_max = self._predict_max_tube_width(v_max_search, curvatures)
        if margin_at_max <= max_allowable_margin:
            return v_max_search

        # 2. 快速检查：如果蠕行速度的不确定性都超标（说明底噪太大或阈值设太小），返回最低速
        margin_at_min = self._predict_max_tube_width(0.1, curvatures)
        if margin_at_min > max_allowable_margin:
            # print(f"[Warn] Base uncertainty {margin_at_min:.3f} exceeds limit {max_allowable_margin:.3f}!")
            return 1.0 

        # 3. 二分查找 (Bisection Search)
        # 函数 f(v) = Margin(v) 单调递增
        v_low = 0.1
        v_high = v_max_search
        epsilon = 0.1  # 精度 0.1 m/s

        for _ in range(15): # log2(50/0.1) ≈ 9，给多点余量
            if (v_high - v_low) < epsilon:
                break
            
            v_mid = (v_low + v_high) / 2.0
            current_margin = self._predict_max_tube_width(v_mid, curvatures)
            
            if current_margin > max_allowable_margin:
                # 误差太大 -> 必须减速
                v_high = v_mid
            else:
                # 误差在范围内 -> 可以加速
                v_low = v_mid
        
        return v_low

    def _predict_max_tube_width(self, v_test, curvatures):
        """
        [辅助函数] 虚拟推演：假设以 v_test 匀速行驶，最大 Margin 会是多少？
        """
        # 1. 构造虚拟参考轨迹 (Dummy Reference)
        # 只需要速度 v_test 正确，位置及航向对协方差传播的 B 矩阵影响极小 (线性化点)
        dummy_xref = np.zeros((NX, NP + 1))
        dummy_xref[2, :] = v_test 
        
        # 2. 调用现有的多源噪声传播函数
        # 复用 _compute_probabilistic_margins 逻辑，它会自动计算 FASD、Actuator Noise 等
        margins = self._compute_probabilistic_margins(dummy_xref, curvatures, v_test)
        
        # 3. 返回预测时域内的最大不确定性宽度
        return max(margins) if margins else 0.0
    def solve(self, state, xref, lane_boundaries=None, curvatures=None, delta_refs=None):
        """
        接口与复杂版 MPC 保持一致
        Args:
            lane_boundaries: (Optional) 只有在 Standard MPC 模式下才使用. 
                             在 Decoupled Tube MPC 模式下，忽略此参数，使用内部定义的 corridor_width.
        """
        # ========== 0. Delay compensation & FASD Update ==========

        
        x_delay = state.x
        y_delay = state.y
        v_delay = state.v
        yaw_delay = state.yaw

        if self.system_delay > 0.01:
            x_delay = state.x + state.v * math.cos(state.yaw) * self.system_delay
            y_delay = state.y + state.v * math.sin(state.yaw) * self.system_delay
            v_delay = state.v
            yaw_delay = state.yaw

        # Yaw unwrap
        current_yaw = (yaw_delay + math.pi) % (2 * math.pi) - math.pi
        diff = xref[3, 0] - current_yaw
        cycle_shift = round(diff / (2 * math.pi))
        xref[3, :] -= cycle_shift * 2 * math.pi
        # [新增] 在 Solve 开始时，更新一下类成员 self.K_lqr
        # 虽然这主要影响最后的补偿环节，但保持状态一致性是个好习惯
        if self.use_lqr:
            self.K_lqr = self._compute_lqr_gain(state.v)
        # set opti params
        self.opti.set_value(self.x0_param, [x_delay, y_delay, v_delay, yaw_delay])
        self.opti.set_value(self.xref_param, xref)
        # [修改] 使用类内部维护的历史记录，而不是每次都传 [0,0]
        # u_history[0] 是最近一次 (last_u, u_{k-1})
        # u_history[1] 是再前一次 (second_last, u_{k-2})
        self.opti.set_value(self.last_u_param, self.u_history[0]) 
        self.opti.set_value(self.second_last_u_param, self.u_history[1])

        # ========== 1. Corridor / Tube limit calculation ==========
        # Priority: configured tube_radius (if >0) -> planner lane_boundaries -> fallback 1.75m
        tube_r = abs(self.tube_radius) if self.tube_radius is not None else 0.0
        if tube_r > 1e-3:
            corridor_limits = np.full(NP + 1, tube_r)
            using_fixed_tube = True
        else:
            using_fixed_tube = False
            if lane_boundaries is not None:
                lb_np = np.array(lane_boundaries)
                if lb_np.ndim == 2 and lb_np.shape[0] == 2 and lb_np.shape[1] > 2:
                    lb_left, lb_right = lb_np[0, :], lb_np[1, :]
                elif lb_np.ndim == 2 and lb_np.shape[1] == 2 and lb_np.shape[0] > 2:
                    lb_left, lb_right = lb_np[:, 0], lb_np[:, 1]
                else:
                    flat = lb_np.flatten()
                    lb_left = np.full(NP + 1, flat[0])
                    lb_right = np.full(NP + 1, flat[1] if len(flat) > 1 else flat[0])

                len_l = len(lb_left)
                if len_l < NP + 1:
                    lb_left = np.pad(lb_left, (0, NP + 1 - len_l), 'edge')
                    lb_right = np.pad(lb_right, (0, NP + 1 - len_l), 'edge')

                corridor_limits = np.minimum(np.abs(lb_left[:NP + 1]), np.abs(lb_right[:NP + 1]))
            else:
                # fallback
                corridor_limits = np.full(NP + 1, 1.75)

        self.opti.set_value(self.lane_width_param, corridor_limits)

        # ========== 2. Margin calculation (probabilistic / empirical / static) ==========
        base_m = self.tube_config.get('static_margin', 0.0)
        planned_margins = np.zeros(NP + 1)
        is_dynamic_mode = False

        if self.use_tube:
            # probabilistic mode preferred
            if self.tube_mode == 'probabilistic':
                planned_margins = np.array(self._compute_probabilistic_margins(xref, curvatures, abs(v_delay)))
                if np.max(planned_margins) > 1e-6:
                    is_dynamic_mode = True

            else:
                # empirical / static fallback
                # feedback memory term (Old Version)
                k_err = self.tube_config.get('k_err', 0.0)
                margin_fb = k_err * self.prediction_error_cache

                k_v = self.tube_config.get('k_v', 0.0)
                k_dyn = self.tube_config.get('k_curv', 0.0)
                if (k_v > 1e-6 or k_dyn > 1e-6) and curvatures is not None and len(curvatures) >= NP + 1:
                    curvs_part = np.array(curvatures[:NP + 1])
                    current_v = abs(v_delay)
                    dynamic_margins = base_m + k_v * current_v + k_dyn * (current_v ** 2) * np.abs(curvs_part) + margin_fb
                    planned_margins = np.maximum(0.0, dynamic_margins)
                    is_dynamic_mode = True
                else:
                    # purely static
                    planned_margins = np.full(NP + 1, base_m + margin_fb)

        else:
            planned_margins = np.zeros(NP + 1)

        # ensure non-negative
        planned_margins = np.maximum(0.0, np.array(planned_margins))

        # ========== 3. Solve attempts (dynamic -> static -> zero) ==========
        final_sol = None
        margins_used = planned_margins
        solve_status = 0 # 0: Dynamic, 1: Static, 2: Zero, -1: Fail

        sol, info = self._attempt_solve(xref, planned_margins)
        if sol is not None:
            final_sol = sol
            solve_status = 0
        else:
            # try static fallback (only if dynamic was attempted)
            if self.use_tube and is_dynamic_mode:
                fallback_static = np.full(NP + 1, base_m)
                sol, info = self._attempt_solve(xref, fallback_static)
                if sol is not None:
                    final_sol = sol
                    margins_used = fallback_static
                    solve_status = 1

            # final zero-margin fallback
            if final_sol is None:
                fallback_zero = np.zeros(NP + 1)
                sol, info = self._attempt_solve(xref, fallback_zero)
                if sol is not None:
                    final_sol = sol
                    margins_used = fallback_zero
                    solve_status = 2
                else:
                    print("[TubeMPC] Error: all fallback solves failed")
                    return [None] * 8

        # ========== 4. Extract solution ==========
        u_opt = final_sol.value(self.u_var)
        x_opt_traj = final_sol.value(self.x_var)

        # 1. 提取纯净的名义控制指令 (Pure Nominal Commands)
        oa = u_opt[0, :].copy()
        odelta = u_opt[1, :].copy()

        # 2. 提取名义预测状态
        ox = x_opt_traj[0, :]
        oy = x_opt_traj[1, :]
        ov = x_opt_traj[2, :]
        oyaw = x_opt_traj[3, :]

        # 3. 保存预测状态用于 FASD 误差计算
        self.last_predicted_state = x_opt_traj[:, 1].tolist()
        safety_margins = margins_used.tolist() if isinstance(margins_used, np.ndarray) else margins_used

        # 【重点】：取消在此处计算 LQR、Slew Rate 和更新 u_history。
        # 一切后处理统统交接给 Runner 的 50Hz 主循环！
        return oa, odelta, ox, oy, oyaw, ov, safety_margins, solve_status

    # def _attempt_solve(self, xref, margins):
    #     """内部辅助函数：尝试一次求解"""
    #     # 1. 设置参数
    #     self.opti.set_value(self.safety_margin_param, margins)
        
    #     # 2. 设置 Initial Guess (Cold Start or Warm Start)
    #     # 简单起见，每次用 xref 初始化状态，用 0 初始化控制
    #     # 如果追求性能，可以把上次的解存下来传入
    #     self.opti.set_initial(self.x_var, xref)
    #     self.opti.set_initial(self.u_var, np.zeros((NU, NC)))
        
    #     # 3. 求解
    #     try:
    #         sol = self.opti.solve()
    #         return sol, "Success"
    #     except Exception as e:
    #         return None, str(e)

    def _attempt_solve(self, xref, margins):
        self.opti.set_value(self.safety_margin_param, margins)
        
        # 【修改】Warm Start：用上一次的解而不是冷启动
        if hasattr(self, '_last_x_opt') and self._last_x_opt is not None:
            # 上一次的解左移一步作为初始猜测
            x_warm = np.zeros((NX, NP + 1))
            x_warm[:, :-1] = self._last_x_opt[:, 1:]  # 左移一步
            x_warm[:, -1] = self._last_x_opt[:, -1]    # 最后一步复制
            self.opti.set_initial(self.x_var, x_warm)
            
            u_warm = np.zeros((NU, NC))
            if self._last_u_opt is not None:
                u_warm[:, :-1] = self._last_u_opt[:, 1:]
                u_warm[:, -1] = self._last_u_opt[:, -1]
            self.opti.set_initial(self.u_var, u_warm)
        else:
            self.opti.set_initial(self.x_var, xref)
            self.opti.set_initial(self.u_var, np.zeros((NU, NC)))
        
        try:
            sol = self.opti.solve()
            # 保存本次解用于下次 Warm Start
            self._last_x_opt = sol.value(self.x_var)
            self._last_u_opt = sol.value(self.u_var)
            return sol, "Success"
        except Exception as e:
            self._last_x_opt = None
            self._last_u_opt = None
            return None, str(e)