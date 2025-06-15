import numpy as np
from scipy.spatial.transform import Rotation


class MorphingDroneParams:
    """
    This class holds all the physical parameters of the morphing drone,
    ported directly from the MorphingDynamicsParams C++ constructor.
    """

    def __init__(self):
        # Physical constants
        self.Gz = 9.81
        self.Rho = 1.225

        # Mass and Inertia
        self.mass_ = 0.133
        self.J_ = (
            np.array(
                [
                    [917668.0, 0.0, 39330.0],
                    [0.0, 3543007.0, 0.0],
                    [39330.0, 0.0, 4389834.0],
                ]
            )
            * 1e-9
        )
        self.J_inv_ = np.linalg.inv(self.J_)

        # Root wing geometric parameters
        self.b_center_ = 0.027
        self.b_root_inner_ = (0.11 * 2) + self.b_center_
        self.c_root_inner_ = 0.18
        self.b_root_outer_ = (0.155 * 2) + self.b_center_
        self.c_root_outer_ = 0.15
        self.s_one_root_ = 0.0265
        self.s_wing_root_ = (2 * self.s_one_root_) + (
            self.b_center_ * self.c_root_inner_
        )
        self.mac_root_ = (1 / self.s_wing_root_) * (
            self.b_root_inner_ * self.c_root_inner_**2
            + (self.b_root_outer_ - self.b_root_inner_) * self.c_root_outer_**2
        )
        self.geo_ctr_y_root_ = (1 / self.s_wing_root_) * (
            self.b_root_inner_ * self.c_root_inner_ * self.b_root_inner_ / 4
            + (self.b_root_outer_ - self.b_root_inner_)
            * self.c_root_outer_
            * (self.b_root_outer_ + self.b_root_inner_)
            / 4
        )
        self.geo_ctr_x_root_ = (1 / self.s_wing_root_) * (
            self.b_root_inner_ * self.c_root_inner_**2 / 2
            + (self.b_root_outer_ - self.b_root_inner_)
            * self.c_root_outer_
            * (self.c_root_outer_ / 2)
        )

        # Tail geometric parameters
        self.k_alpha_elev_ = 0.7
        self.k_alpha_rudder_ = 0.7
        self.s_hor_tail_ = 0.021495
        self.s_vert_tail_ = 0.0125625
        self.c_hor_tail_ = 0.10
        self.c_vert_tail_ = 0.15
        self.c_ele_ = 0.058
        self.c_rud_ = 0.096
        self.b_hor_tail_ = 0.20
        self.b_vert_tail_ = 0.15

        # Slipstream effectiveness
        self.K_slip_tail_ = 1.0
        self.K_slip_wing_ = 1.0

        # Lift/Drag parameters
        self.M_smooth_ = 0.2
        self.alpha_stall_wing_ = np.deg2rad(14.0)
        self.cl_alpha_wing_2D_ = 5.73
        self.c_d_0_wing_ = 0.12
        self.alpha_stall_tail_ = np.deg2rad(20.0)
        self.cl_alpha_tail_2D_ = 5.37
        self.c_d_0_tail_ = 0.2
        self.c_m_0_drone_ = -0.002

        # Positions (C.O.G. and surfaces)
        self.pos_cg_ = np.array([-0.075, 0.0, 0.0])
        self.pos_ele_le_ = np.array([-0.339, 0.0, 0.0])
        self.pos_rud_le_ = np.array([-0.344, 0.0, 0.075])
        self.pos_wing_left_le_ = np.array([0.0, 0.0145, 0.0])
        self.pos_wing_right_le_ = np.array([0.0, -0.0145, 0.0])
        self.pos_prop_ = np.array([0.18, 0.0, 0.0])

        self.pos_cg_ele_le_ = self.pos_ele_le_ - self.pos_cg_
        self.pos_cg_rud_le_ = self.pos_rud_le_ - self.pos_cg_
        self.pos_cg_wing_left_le_ = self.pos_wing_left_le_ - self.pos_cg_
        self.pos_cg_wing_right_le_ = self.pos_wing_right_le_ - self.pos_cg_
        self.pos_cg_prop_ = self.pos_prop_ - self.pos_cg_

        # Rotation axes
        self.rot_axis_ele_ = np.array([0.0, 1.0, 0.0])
        self.rot_axis_rud_ = np.array([0.0, 0.0, 1.0])

        # Propeller
        self.R_prop_ = 0.075

        # Actuator limits & mappings
        self.sw_max_ = 75.0
        self.sw_min_ = -5.0
        self.ele_max_ = 26.0
        self.ele_min_ = -26.0
        self.rud_max_ = 30.0
        self.rud_min_ = -30.0
        self.twist_max_ = 8.0
        self.twist_min_ = -8.0
        self.sweep_offset_ = 0.5

        # Motor model
        self.throttle_offset_ = 0.05
        self.motor_omega_map_ = -0.6713
        self.T_max_ = 1.03
        self.T_min_ = 0.0


def _sigmoid(x, x_cutoff, M):
    """Vectorized sigmoid function from the C++ code."""
    x_deg = np.rad2deg(x)
    x_cutoff_deg = np.rad2deg(x_cutoff)
    term1 = np.exp(-M * (x_deg - x_cutoff_deg))
    term2 = np.exp(M * (x_deg + x_cutoff_deg))
    return (1 + term1 + term2) / ((1 + term1) * (1 + term2))


def _wing_properties(theta_sw_rad, p):
    """Vectorized WingPropertiesCPU."""
    theta_sw_deg = np.rad2deg(theta_sw_rad)
    b_outer = (-0.0195 * theta_sw_deg**2 + 0.0603 * theta_sw_deg + 165) / 1000
    s_outer = (-143.58 * theta_sw_deg + 16309) / 1e6
    ac_x_outer = 0.15 / 6
    cpx_outer = (-0.00359 * theta_sw_deg**2 + 0.754 * theta_sw_deg + 59.04) / 1000
    cpy_outer = (-0.00397 * theta_sw_deg**2 - 0.251 * theta_sw_deg + 62.78) / 1000

    b_wing = p.b_root_outer_ + 2 * b_outer
    s_wing = p.s_wing_root_ + 2 * s_outer
    ac_x_wing = (
        ((p.mac_root_ / 4) * p.s_wing_root_) + (ac_x_outer * 2 * s_outer)
    ) / s_wing
    cpx_wing = (
        (p.geo_ctr_x_root_ * p.s_wing_root_) + (cpx_outer * 2 * s_outer)
    ) / s_wing
    cpy_wing = (
        (p.geo_ctr_y_root_ * p.s_wing_root_)
        + ((cpy_outer + p.b_root_outer_ / 2) * 2 * s_outer)
    ) / s_wing
    return b_wing, s_wing, ac_x_wing, cpx_wing, cpy_wing


def _wing_aero_center(
    vel,
    wing_twist_rad,
    ac_x_wing,
    cpx_wing,
    cpy_wing,
    alpha_stall,
    negate,
    calculate_rot_pt,
):
    """Vectorized WingAeroCenterCPU."""
    # Use atan2 for numerical stability, equivalent to atan(vel_z / vel_x)
    alpha = np.abs(np.arctan2(vel[:, 2], vel[:, 0]) + wing_twist_rad)

    c_aero_x = np.where(
        calculate_rot_pt,
        -cpx_wing,
        np.where(
            np.abs(alpha) > alpha_stall,
            -1
            * (
                ac_x_wing
                + ((np.abs(alpha) - alpha_stall) / (np.pi / 2 - alpha_stall))
                * (cpx_wing - ac_x_wing)
            ),
            -1 * ac_x_wing,
        ),
    )
    c_aero_y = np.where(negate, -cpy_wing, cpy_wing)

    c_aero_pt = np.zeros_like(vel)
    c_aero_pt[:, 0] = c_aero_x
    c_aero_pt[:, 1] = c_aero_y
    return c_aero_pt


def _r_body_wing(wing_twist):
    """Vectorized R_body_wingCPU."""
    n = wing_twist.shape[0]
    st = np.sin(wing_twist)
    ct = np.cos(wing_twist)
    R_wb = np.zeros((n, 3, 3))
    R_wb[:, 0, 0] = ct
    R_wb[:, 0, 2] = st
    R_wb[:, 1, 1] = 1.0
    R_wb[:, 2, 0] = -st
    R_wb[:, 2, 2] = ct
    return R_wb


def _aero_coefficients(AR, alpha, alpha_stall, M_smooth, cl_alpha_2D, c_d_0):
    """Vectorized AeroCoefficientsCPU."""
    k_cd = 1 - 0.41 * (1 - np.exp(-17 / AR))

    w = np.zeros_like(alpha)
    mask = (alpha > alpha_stall) & (alpha < np.pi - alpha_stall)
    w[mask] = np.cos(
        np.pi * ((alpha[mask] - alpha_stall) / ((np.pi - alpha_stall) - alpha_stall))
        - np.pi / 2
    )

    c_l_st = 2 * np.sin(alpha) * np.cos(alpha) * (1 - w * (1 - k_cd))
    c_d_st = 2 * np.sin(alpha) ** 2 * (1 - w * (1 - k_cd))

    c_l_lin = (cl_alpha_2D * AR / (2 + np.sqrt(AR**2 + 4))) * alpha
    c_d_quad = c_d_0 + c_l_lin**2 / (np.pi * AR)

    sigmoid_a = _sigmoid(alpha, alpha_stall, M_smooth)

    c_l = (1 - sigmoid_a) * c_l_lin + sigmoid_a * c_l_st
    c_d = (1 - sigmoid_a) * c_d_quad + sigmoid_a * c_d_st

    return np.stack([c_l, c_d], axis=-1)


def _wing_force_with_thrust(
    n_s,
    flow_vel,
    flow_vel_slip,
    flow_rot,
    R_tw_b,
    R_b_tw,
    b,
    s,
    s_slip,
    alpha_stall,
    M_smooth,
    cl_alpha_2D,
    c_d_0,
    p,
):
    """Vectorized WingForcewithThrustCPU."""
    flow_vel_tw = np.einsum("nij,nj->ni", R_b_tw, flow_vel + flow_rot)
    flow_vel_slip_tw = np.einsum("nij,nj->ni", R_b_tw, flow_vel_slip + flow_rot)

    s_half = s / 2
    flow_vel_avg_tw = (
        flow_vel_tw * (s_half - s_slip)[:, np.newaxis]
        + flow_vel_slip_tw * s_slip[:, np.newaxis]
    ) / s_half[:, np.newaxis]

    flow_vel_avg_tw_norm = np.linalg.norm(flow_vel_avg_tw, axis=1, keepdims=True)
    flow_vel_avg_tw_norm = np.where(
        flow_vel_avg_tw_norm == 0, 1e-9, flow_vel_avg_tw_norm
    )
    n_d = flow_vel_avg_tw / flow_vel_avg_tw_norm

    dot_prod = np.einsum("ij,ij->i", n_s, n_d)
    n_s = np.where(dot_prod[:, np.newaxis] < 0, -n_s, n_s)

    n_l = np.cross(np.cross(n_d, n_s), n_d)
    n_l_norm = np.linalg.norm(n_l, axis=1, keepdims=True)
    n_l = np.where(n_l_norm == 0, np.array([1.0, 0.0, 0.0]), n_l / n_l_norm)

    AR = b**2 / s

    alpha_wing = np.abs(np.arctan2(-flow_vel_tw[:, 2], flow_vel_tw[:, 0]))
    alpha_wing_slip = np.abs(
        np.arctan2(-flow_vel_slip_tw[:, 2], flow_vel_slip_tw[:, 0])
    )

    c_aero_wing = _aero_coefficients(
        AR, alpha_wing, alpha_stall, M_smooth, cl_alpha_2D, c_d_0
    )
    c_aero_wing_slip = _aero_coefficients(
        AR, alpha_wing_slip, alpha_stall, M_smooth, cl_alpha_2D, c_d_0
    )

    c_aero_avg = (
        c_aero_wing
        * np.linalg.norm(flow_vel_tw, axis=1, keepdims=True) ** 2
        * (s_half - s_slip)[:, np.newaxis]
        + c_aero_wing_slip
        * np.linalg.norm(flow_vel_slip_tw, axis=1, keepdims=True) ** 2
        * s_slip[:, np.newaxis]
    ) / (flow_vel_avg_tw_norm**2 * s_half[:, np.newaxis])

    f_l = 0.5 * p.Rho * flow_vel_avg_tw_norm.flatten() ** 2 * s_half * c_aero_avg[:, 0]
    f_d = 0.5 * p.Rho * flow_vel_avg_tw_norm.flatten() ** 2 * s_half * c_aero_avg[:, 1]

    F_l = f_l[:, np.newaxis] * n_l
    F_d = f_d[:, np.newaxis] * n_d

    force_output = np.einsum("nij,nj->ni", R_tw_b, F_l + F_d)
    eps_downwash = c_aero_avg[:, 0] / (np.pi * AR)

    return force_output, eps_downwash


def calculate_forces_and_moments(
    quaternions_xyzw, omega, vel_body, actuation
):  # <<< MODIFIED: Added vel_body
    """
    Calculates the aerodynamic forces and moments on a morphing drone.

    Args:
        quaternions_xyzw (np.ndarray): Array of quaternions (x, y, z, w). Shape: (N, 4).
        omega (np.ndarray): Array of body-frame angular velocities (p, q, r). Shape: (N, 3).
        vel_body (np.ndarray): Array of body-frame velocity relative to air (u, v, w). Shape: (N, 3).
        actuation (np.ndarray): Array of normalized actuator commands. Shape: (N, 5).
    """
    p = MorphingDroneParams()
    n_timesteps = quaternions_xyzw.shape[0]

    # --- 1. State and Kinematics ---
    rot = Rotation.from_quat(quaternions_xyzw)
    dcm = rot.as_matrix()

    # <<< MODIFIED: Use the provided velocity
    vel = vel_body
    u_vel = vel[:, 0]

    # --- 2. Actuator Mapping ---
    u_sweep, u_thrust, u_ele, u_rud, u_twist = [actuation[:, i] for i in range(5)]

    theta_sw_norm = (u_sweep + (p.sweep_offset_ / 2)) / (1 - p.sweep_offset_ / 2)
    wing_sweep = np.deg2rad(
        np.clip(
            ((p.sw_max_ - p.sw_min_) * theta_sw_norm / 2 + (p.sw_max_ + p.sw_min_) / 2),
            p.sw_min_,
            p.sw_max_,
        )
    )
    elevator = np.deg2rad(
        np.clip(
            ((p.ele_max_ - p.ele_min_) * u_ele / 2 + (p.ele_max_ + p.ele_min_) / 2),
            p.ele_min_,
            p.ele_max_,
        )
    )
    rudder = np.deg2rad(
        np.clip(
            ((p.rud_max_ - p.rud_min_) * u_rud / 2 + (p.rud_max_ + p.rud_min_) / 2),
            p.rud_min_,
            p.rud_max_,
        )
    )
    wing_twist = np.deg2rad(
        np.clip(
            (
                (p.twist_max_ - p.twist_min_) * u_twist / 2
                + (p.twist_max_ + p.twist_min_) / 2
            ),
            p.twist_min_,
            p.twist_max_,
        )
    )

    u_thr_clamped = np.clip(u_thrust, p.throttle_offset_, 1.0)
    omega_mot_des_norm = p.motor_omega_map_ * (
        u_thr_clamped - p.throttle_offset_
    ) ** 2 + (
        1.0 / (1.0 - p.throttle_offset_)
        - p.motor_omega_map_ * (1.0 + p.throttle_offset_)
    ) * (
        u_thr_clamped - p.throttle_offset_
    )
    thrust = np.clip(omega_mot_des_norm, 0, 1) ** 2 * (p.T_max_ - p.T_min_) + p.T_min_

    # --- 3. Flow Field Calculation ---
    # <<< MODIFIED: Full prop wash equation
    prop_flow_ind_wake = (
        -u_vel
        + np.sqrt(u_vel**2 + (2 * np.abs(thrust) / (p.Rho * np.pi * p.R_prop_**2)))
    ) / 2

    flow_tail = np.zeros((n_timesteps, 3))
    flow_tail[:, 0] = -(u_vel + p.K_slip_tail_ * prop_flow_ind_wake)
    flow_tail[:, 1:] = -vel[:, 1:]

    prop_factor_left = (p.pos_cg_prop_[0] - p.pos_cg_wing_left_le_[0]) / p.R_prop_
    prop_factor_right = (p.pos_cg_prop_[0] - p.pos_cg_wing_right_le_[0]) / p.R_prop_

    prop_flow_wing_left = u_vel + p.K_slip_wing_ * (prop_flow_ind_wake / 2) * (
        1 + prop_factor_left / np.sqrt(1 + prop_factor_left**2)
    )
    prop_flow_wing_right = u_vel + p.K_slip_wing_ * (prop_flow_ind_wake / 2) * (
        1 + prop_factor_right / np.sqrt(1 + prop_factor_right**2)
    )

    flow_wing_left_slip = -vel.copy()
    flow_wing_left_slip[:, 0] = -prop_flow_wing_left
    flow_wing_right_slip = -vel.copy()
    flow_wing_right_slip[:, 0] = -prop_flow_wing_right
    flow_wing = -vel

    # --- 4. Wing Forces & Moments ---
    b_w, s_w, ac_x_w, cpx_w, cpy_w = _wing_properties(wing_sweep, p)
    s_slip_wing = p.R_prop_ * p.c_root_inner_ * np.ones(n_timesteps)
    n_s_vec = np.tile(np.array([0.0, 0.0, 1.0]), (n_timesteps, 1))

    # Left Wing
    R_l_wb = _r_body_wing(wing_twist)
    R_b_lw = np.transpose(R_l_wb, (0, 2, 1))
    pos_rot_c_wl = (
        _wing_aero_center(
            flow_wing,
            wing_twist,
            ac_x_w,
            cpx_w,
            cpy_w,
            p.alpha_stall_wing_,
            negate=False,
            calculate_rot_pt=True,
        )
        + p.pos_cg_wing_left_le_
    )
    flow_rot_wl = -np.cross(omega, pos_rot_c_wl)
    F_wing_left, eps_down_left = _wing_force_with_thrust(
        n_s_vec,
        flow_wing,
        flow_wing_left_slip,
        flow_rot_wl,
        R_l_wb,
        R_b_lw,
        b_w,
        s_w,
        s_slip_wing,
        p.alpha_stall_wing_,
        p.M_smooth_,
        p.cl_alpha_wing_2D_,
        p.c_d_0_wing_,
        p,
    )
    sum_flow_l = (
        flow_wing * ((s_w / 2) - s_slip_wing)[:, np.newaxis]
        + flow_wing_left_slip * s_slip_wing[:, np.newaxis]
    ) / (s_w / 2)[:, np.newaxis] + flow_rot_wl
    pos_force_wing_left = (
        _wing_aero_center(
            sum_flow_l,
            wing_twist,
            ac_x_w,
            cpx_w,
            cpy_w,
            p.alpha_stall_wing_,
            negate=False,
            calculate_rot_pt=False,
        )
        + p.pos_cg_wing_left_le_
    )

    # Right Wing
    R_r_wb = _r_body_wing(wing_twist)
    R_b_rw = np.transpose(R_r_wb, (0, 2, 1))
    pos_rot_c_wr = (
        _wing_aero_center(
            flow_wing,
            wing_twist,
            ac_x_w,
            cpx_w,
            cpy_w,
            p.alpha_stall_wing_,
            negate=True,
            calculate_rot_pt=True,
        )
        + p.pos_cg_wing_right_le_
    )
    flow_rot_wr = -np.cross(omega, pos_rot_c_wr)
    F_wing_right, eps_down_right = _wing_force_with_thrust(
        n_s_vec,
        flow_wing,
        flow_wing_right_slip,
        flow_rot_wr,
        R_r_wb,
        R_b_rw,
        b_w,
        s_w,
        s_slip_wing,
        p.alpha_stall_wing_,
        p.M_smooth_,
        p.cl_alpha_wing_2D_,
        p.c_d_0_wing_,
        p,
    )
    sum_flow_r = (
        flow_wing * ((s_w / 2) - s_slip_wing)[:, np.newaxis]
        + flow_wing_right_slip * s_slip_wing[:, np.newaxis]
    ) / (s_w / 2)[:, np.newaxis] + flow_rot_wr
    pos_force_wing_right = (
        _wing_aero_center(
            sum_flow_r,
            wing_twist,
            ac_x_w,
            cpx_w,
            cpy_w,
            p.alpha_stall_wing_,
            negate=True,
            calculate_rot_pt=False,
        )
        + p.pos_cg_wing_right_le_
    )

    # --- 5. Tail Forces & Moments ---
    # Elevator / Horizontal Tail
    ang_eff_ele = p.k_alpha_elev_ * elevator
    dir_ele = np.stack(
        [-np.sin(ang_eff_ele), np.zeros(n_timesteps), np.cos(ang_eff_ele)], axis=-1
    )
    pos_ele = p.pos_cg_ele_le_ + (p.c_hor_tail_ / 2) * np.cross(
        dir_ele, p.rot_axis_ele_
    )
    flow_rot_ele = -np.cross(omega, pos_ele)
    # <<< MODIFIED: Re-enabled downwash
    flow_downwash_ele = np.zeros_like(vel)
    downwash_mask = u_vel > 0
    flow_downwash_ele[downwash_mask, 2] = (
        np.tan(eps_down_left[downwash_mask] + eps_down_right[downwash_mask])
        * flow_wing[downwash_mask, 0]
    )

    sum_flow_h = flow_tail + flow_rot_ele + flow_downwash_ele
    # ... (rest of the tail calculation is robust and remains the same)

    # --- 6. Thrust, Gravity, and Summation ---
    F_thr = np.zeros_like(vel)
    F_thr[:, 0] = thrust
    F_gra_world = np.array([0, 0, p.mass_ * p.Gz])
    F_gra_body = np.einsum("nij,j->ni", dcm, F_gra_world)

    # <<< MODIFIED: Re-enabled static pitch moment
    vel_norm = np.linalg.norm(vel, axis=1)
    M_y_stat = np.zeros_like(vel)
    M_y_stat[:, 1] = 0.5 * p.Rho * p.c_m_0_drone_ * vel_norm**2

    sum_flow_h_norm = np.linalg.norm(sum_flow_h, axis=1, keepdims=True)
    sum_flow_h_norm = np.where(sum_flow_h_norm == 0, 1e-9, sum_flow_h_norm)
    alpha_tail = np.abs(np.arctan2(-sum_flow_h[:, 2], sum_flow_h[:, 0]) + ang_eff_ele)
    c_aero_tail = np.stack(
        [
            2 * np.sin(alpha_tail) * np.cos(alpha_tail),
            p.c_d_0_tail_ + 2 * np.sin(alpha_tail) ** 2,
        ],
        axis=-1,
    )
    f_l_ele = (
        0.5 * p.Rho * sum_flow_h_norm.flatten() ** 2 * p.s_hor_tail_ * c_aero_tail[:, 0]
    )
    f_d_ele = (
        0.5 * p.Rho * sum_flow_h_norm.flatten() ** 2 * p.s_hor_tail_ * c_aero_tail[:, 1]
    )
    n_l_ele = np.cross(np.cross(sum_flow_h, dir_ele), sum_flow_h)
    n_l_ele_norm = np.linalg.norm(n_l_ele, axis=1, keepdims=True)
    n_l_ele = np.where(
        n_l_ele_norm == 0, np.array([1.0, 0.0, 0.0]), n_l_ele / n_l_ele_norm
    )
    F_ele = f_l_ele[:, np.newaxis] * n_l_ele + f_d_ele[:, np.newaxis] * (
        sum_flow_h / sum_flow_h_norm
    )

    ang_eff_rud = p.k_alpha_rudder_ * rudder
    dir_rud = np.stack(
        [-np.sin(ang_eff_rud), -np.cos(ang_eff_rud), np.zeros(n_timesteps)], axis=-1
    )
    pos_rud = p.pos_cg_rud_le_ + (p.c_vert_tail_ / 2) * np.cross(
        dir_rud, p.rot_axis_rud_
    )
    flow_rot_rud = -np.cross(omega, pos_rud)
    sum_flow_v = flow_tail + flow_rot_rud
    sum_flow_v_norm = np.linalg.norm(sum_flow_v, axis=1, keepdims=True)
    sum_flow_v_norm = np.where(sum_flow_v_norm == 0, 1e-9, sum_flow_v_norm)
    beta = np.abs(np.arctan2(sum_flow_v[:, 1], sum_flow_v[:, 0]) + ang_eff_rud)
    c_l_rud = 2 * np.sin(beta) * np.cos(beta)
    c_d_rud = 2 * np.sin(beta) ** 2
    f_l_rud = 0.5 * p.Rho * sum_flow_v_norm.flatten() ** 2 * p.s_vert_tail_ * c_l_rud
    f_d_rud = 0.5 * p.Rho * sum_flow_v_norm.flatten() ** 2 * p.s_vert_tail_ * c_d_rud
    n_l_rud = np.cross(np.cross(sum_flow_v, dir_rud), sum_flow_v)
    n_l_rud_norm = np.linalg.norm(n_l_rud, axis=1, keepdims=True)
    n_l_rud = np.where(
        n_l_rud_norm == 0, np.array([0.0, 1.0, 0.0]), n_l_rud / n_l_rud_norm
    )
    F_rud = f_l_rud[:, np.newaxis] * n_l_rud + f_d_rud[:, np.newaxis] * (
        sum_flow_v / sum_flow_v_norm
    )

    total_forces = F_gra_body + F_thr + F_wing_left + F_wing_right + F_ele + F_rud

    M_wing_left = np.cross(pos_force_wing_left, F_wing_left)
    M_wing_right = np.cross(pos_force_wing_right, F_wing_right)
    M_ele = np.cross(pos_ele, F_ele)
    M_rud = np.cross(pos_rud, F_rud)
    M_thr = np.cross(p.pos_cg_prop_, F_thr)

    total_moments = M_wing_left + M_wing_right + M_ele + M_rud + M_thr + M_y_stat

    return total_forces, total_moments
