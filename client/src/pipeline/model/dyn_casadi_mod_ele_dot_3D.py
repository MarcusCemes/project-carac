import casadi as ca
import numpy as np
from numpy.typing import NDArray
import spatial_casadi as sc


class IndoorUAV3D:
    def __init__(self):
        self.m = 0.132  # Drone mass in [kg]
        self.g = 9.801
        self.pi = 3.14159
        self.rho = 1.225
        self.nu = 1.5e-5

        self.M_tails = []
        self.M_wings = []

        # Wing geometric paramters
        self.b_center = 0.027
        self.b_root_inner = (0.11 * 2) + self.b_center
        self.c_root_inner = 0.18
        self.b_root_outer = (0.155 * 2) + self.b_center
        self.c_root_outer = 0.15
        self.s_one_root = 0.0265

        # Wing mass parameters
        self.m_outer_wing = 0.005

        self.s_wing_root = (2 * self.s_one_root) + (self.b_center * self.c_root_inner)
        self.mac_root = (1 / self.s_wing_root) * (
            self.b_root_inner * self.c_root_inner * self.c_root_inner
            + (self.b_root_outer - self.b_root_inner)
            * self.c_root_outer
            * self.c_root_outer
        )
        self.geo_ctr_y_root = (1 / self.s_wing_root) * (
            self.b_root_inner * self.c_root_inner * self.b_root_inner / 4
            + (self.b_root_outer - self.b_root_inner)
            * self.c_root_outer
            * (self.b_root_outer + self.b_root_inner)
            / 4
        )
        self.geo_ctr_x_root = (1 / self.s_wing_root) * (
            self.b_root_inner * self.c_root_inner * self.c_root_inner / 2
            + (self.b_root_outer - self.b_root_inner)
            * self.c_root_outer
            * (self.c_root_outer / 2)
        )
        # print(self.geo_ctr_y_root)

        # Tail geometric parameters
        self.k_alpha_elev = 0.7  # Relation betwween rudder angle and effective AOA (d(alpha_eff)/d(tau_ele))
        self.k_alpha_rudder = 0.7  # Relation betwween rudder angle and effective AOA (d(alpha_eff)/d(tau_rud))
        self.s_hor_tail = 0.021495
        self.s_vert_tail = 0.0125625
        self.c_hor_tail = 0.10
        self.c_vert_tail = 0.15
        self.c_ele = 0.058
        self.c_rud = 0.096
        self.b_hor_tail = 0.20
        self.b_vert_tail = 0.15

        # Slipstream effectiveness parameters
        self.K_slip_tail = 1.0
        self.K_slip_wing = 1.0

        # Reynolds degradation parameters
        self.Re_ref = 100000
        self.M_Re = 2.5

        # Aerodynamic parameters
        self.M_smooth = 0.2  # Stall transition smoothing

        # Wing lift/drag parameters
        self.alpha_stall_wing = 14.0 * (self.pi / 180)  # Defined in Radians
        self.cl_alpha_wing_2D = (
            2 * self.pi
        )  # + 5.37)/2 #5.73 #5.73 #2D cl_alpha_coefficient
        self.c_l_0_wing = 0.0  # C_L_0 of airfoil
        self.c_d_0_wing = 0.05  # Zero AOA drag coefficient of drone
        self.c_d_0_tail = 0.2
        self.c_m_0_wing = 0.0  # Zero AOA drag coefficient of drone (M_wing_0 = 0.5*rho*c_wing*c_m_0*S_wing*V**2), TBI
        self.c_m_y_drone_static = (
            -0.0
        )  # -0.0004819 #Identified from optimization #Static moment offset coefficient (M_offset = 0.5*rho*c_m_drone*V**2), TBI
        self.c_m_x_drone_static = 0.0
        self.c_m_z_drone_static = 0.0

        # Tail lift/drag parameters
        self.alpha_stall_tail = 20.0 * (self.pi / 180)

        # Position offsets from Wing leading edge
        self.pos_cg_theta_sw_0 = np.array(
            [-0.085, 0.0, 0.0]
        )  # Distance of C.G. from wing leading edge
        # self.pos_cg = np.copy(self.pos_cg_theta_sw_0)
        self.pos_ele_le = np.array([-0.339, 0.0, 0.0])  # np.array([-.1, .0, .0]) #
        self.pos_rud_le = np.array([-0.344, 0.0, 0.075])
        self.pos_wing_left_le = np.array([0.0, 0.0145, 0.0])
        self.pos_wing_right_le = np.array([0.0, -0.0145, 0.0])
        self.pos_prop = np.array([0.18, 0.0, 0.0])

        # Position offsets from center of gravity
        # self.pos_cg_ele_le = self.pos_ele_le - self.pos_cg
        # self.pos_cg_rud_le = self.pos_rud_le - self.pos_cg
        # self.pos_cg_wing_left_le = self.pos_wing_left_le - self.pos_cg
        # self.pos_cg_wing_right_le = self.pos_wing_right_le - self.pos_cg
        # self.pos_cg_prop = self.pos_prop - self.pos_cg

        self.I_ext = ca.DM(
            [(917668.0, 0.0, 39330.0), (0.0, 3543007.0, 0.0), (39330.0, 0.0, 4389834.0)]
        )  # Inertia
        self.I_ext_inv = ca.inv(self.I_ext)  # Inertia inverse

        self.R_prop = 0.075
        self.sw_min = -5.0  # min sweep angle [deg]
        self.sw_max = 75.0  # max sweep angle [deg]
        self.ele_min = -26.0  # min elevator angle [deg]
        self.ele_max = 26.0  # max elevator angle [deg]

        # Servo Model parameters
        self.sweep_offset = (
            0.5  # Servo offset for normalized range (Typically -1.0 to 0.5)
        )
        self.A_sw = ca.DM([(0.0, 1.0), (-974.5, -44.1)])
        self.B_sw = ca.DM([(-4.71), (1174.0)])
        self.C_sw = ca.DM([(1.0, 0.0)])

        # Motor Model parameters
        self.throttle_offset = 0.05
        self.motor_omega_map = -0.6713  # -0.529 #-0.504
        self.motor_tau_inv = 2.054  # 2.20 #1.709
        self.T_min = 0  # min thrust [N]
        self.T_max = 1.03  # 1.0 #1.01 #1.01                          # max thrust [N]
        self.delay_mot = 0.0  # 0.175                           # motor input delay [s]

    def thrust_slipstream(self, thrust, vel_u):
        prop_wake = (
            -vel_u
            + ca.sqrt(vel_u**2 + 2 * thrust / (self.rho * self.pi * self.R_prop**2))
        ) / 2
        V_slip_wing = self.K_slip_wing * prop_wake
        V_slip_tail = self.K_slip_tail * prop_wake

        return V_slip_wing, V_slip_tail

    def wing_geometry(self, theta_sw_deg):
        # Ensure theta_sw_deg is defined in degrees °
        b_outer = (
            -0.0201 * theta_sw_deg**2 + 0.0904 * theta_sw_deg + 165.15
        ) / 1000  # Fit from Excel, in m
        s_outer = (-142.97 * theta_sw_deg + 16290) / (10**6)  # Fit from Excel, in m2
        ac_x_outer = (
            self.c_root_outer / 6
        )  # Assuming neutral point of a double traingular wing with base chord of 0.15 m //(0.15/4)+(2*b_outer*std::tan(theta_sw_deg*M_PI/180)/6); // Assuming outer triangle M.A.C based on base chord of 0.15 m: (2*0.15*s_outer/12) with pointed tip wing assumption
        cpx_theta_0 = 59.55 / 1000
        cpy_theta_0 = 62.78 / 1000
        cpx_outer = (
            (-0.003 * theta_sw_deg**2) + 0.7136 * theta_sw_deg
        ) / 1000 + cpx_theta_0  # Fit of x centroid from Excel, in m
        cpy_outer = (
            ((-0.00397 * theta_sw_deg**2) - 0.251 * theta_sw_deg) / 1000 + cpy_theta_0
        ) + self.b_root_outer / 2  # Fit of y centroid from Excel, in m, with offset of half root chord

        # Static C.G update
        delta_pos_cg_x = -(cpx_outer - cpx_theta_0) * (
            self.m_outer_wing / self.m
        )  # (self.pos_cg_body[0]*(self.m - 2*self.m_outer_wing) - 2*cpx_outer*self.m_outer_wing)/self.m
        delta_pos_cg_y = (cpy_outer - cpy_theta_0) * (
            self.m_outer_wing / self.m
        )  # (self.pos_cg_body[1]*(self.m - 2*self.m_outer_wing) - 2*cpy_outer*self.m_outer_wing)/self.m
        delta_pos_cg_z = 0.0

        delta_pos_cg = ca.vertcat(
            0.0, 0.0, 0.0
        )  # ca.vertcat(delta_pos_cg_x, delta_pos_cg_y, delta_pos_cg_z) #Position offset of C.G. from body frame relative to leading edge of wing offset from C.G.

        b = self.b_root_outer + 2 * b_outer
        S = self.s_wing_root + 2 * s_outer
        AR = (b**2) / S
        ac_x = (
            ((self.mac_root / 4) * self.s_wing_root) + (ac_x_outer * 2 * s_outer)
        ) / S
        cpx_wing = (
            (self.geo_ctr_x_root * self.s_wing_root) + (cpx_outer * 2 * s_outer)
        ) / S
        cpy_wing = (
            b / 4
        )  # ((self.geo_ctr_y_root*self.s_wing_root)+(cpy_outer*2*s_outer))/S
        # print(cpy_wing)
        cpz_wing = 0.0

        S_one_wing = S / 2
        b_one_wing = b / 2

        cp_wing_vec = ca.vertcat(
            cpx_wing, cpy_wing, cpz_wing
        )  # Centroid vector of wing in body frame relative to leading edge of wing offset from C.G.

        return AR, S_one_wing, b_one_wing, ac_x, cp_wing_vec, delta_pos_cg

    def sigmoid(self, x, x_cut, M):

        sig = (
            1
            + ca.exp(-M * (180 / self.pi) * (x - x_cut))
            + ca.exp(M * (180 / self.pi) * (x + x_cut))
        ) / (
            (1 + ca.exp(-M * (180 / self.pi) * (x - x_cut)))
            * (1 + ca.exp(M * (180 / self.pi) * (x + x_cut)))
        )

        return sig

    def wing_coefficients(
        self,
        alpha,
        alpha_stall,
        AR,
        ac_x,
        geo_x,
        pos_cg_from_le,
        f_re=1,
        c_d_0=0.0,
        c_l_dyn_pre_st=0.0,
    ):

        # alpha between -90° and 90°

        k_cd = 1 - 0.41 * (1 - ca.exp(-17 / AR))
        w_pos = ca.cos(
            self.pi * ((alpha - alpha_stall) / (self.pi - 2 * alpha_stall))
            - self.pi / 2
        )
        w_neg = ca.cos(
            self.pi * ((-alpha - alpha_stall) / (self.pi - 2 * alpha_stall))
            - self.pi / 2
        )
        w = (1 / (1 + ca.exp(-20 * (alpha - alpha_stall)))) * w_pos + (
            1 / (1 + ca.exp(-20 * (-alpha - alpha_stall)))
        ) * w_neg  # self.sigmoid(alpha,-alpha_stall,10)*w_pos + self.sigmoid(alpha,-alpha_stall,10)*w_neg
        c_l_st = f_re * (2 * ca.sin(alpha) * ca.cos(alpha) * (1 - w * (1 - k_cd)))
        c_d_st = 2 * f_re * (ca.sin(alpha) ** 2) * ((1 - w * (1 - k_cd)))
        # d_x_ac_cg_st = pos_cg_from_le + (w*(geo_x - ac_x) + ac_x)

        c_l_lin = f_re * (
            self.c_l_0_wing
            + ((self.cl_alpha_wing_2D * AR) / (2 + ca.sqrt(AR**2 + 4))) * (alpha)
        )
        c_d_quad = c_d_0 + (c_l_lin**2) / (self.pi * AR)
        # d_x_ac_cg_lin = pos_cg_from_le + ac_x

        c_l = (1 - self.sigmoid(alpha, alpha_stall, self.M_smooth)) * (
            c_l_lin + c_l_dyn_pre_st
        ) + self.sigmoid(alpha, alpha_stall, self.M_smooth) * c_l_st
        c_d = (
            1 - self.sigmoid(alpha, alpha_stall, self.M_smooth)
        ) * c_d_quad + self.sigmoid(alpha, alpha_stall, self.M_smooth) * c_d_st
        # d_x_m = (1-self.sigmoid(alpha,alpha_stall,self.M_smooth))*d_x_ac_cg_lin + self.sigmoid(alpha,alpha_stall,self.M_smooth)*d_x_ac_cg_st #(Unit: m)
        d_x_m = (
            -(ac_x + (2 * ca.sqrt(alpha**2) / self.pi) * (geo_x - ac_x))
            - pos_cg_from_le
        )  # Position of aerodynamic center in body frame relative to C.G. (in body frame)
        # mac = ac_x*4
        # print(mac)
        # d_x_m = pos_cg_from_le + (0.5 - 0.175*(1-2*ca.sqrt(alpha**2)/self.pi))*ac_x*4
        # print(d_x_m)

        return c_l, c_d, d_x_m

    def wing_aerodynamics(
        self, V_slip_wing, vel, ome, AR, S, ac_x, geo_center_vec, pos_cg
    ):

        S_slip = self.c_root_inner * (self.R_prop)
        S_free_wing = S - S_slip

        vel_inb = -(
            vel + ca.cross(ome, pos_cg + geo_center_vec)
        )  # Body frame wing inbound velocity corrected for quasi-steady rotation
        vel_slip_inb = vel_inb - ca.vertcat(
            V_slip_wing, 0, 0
        )  # Slipstream velocity in body frame

        V = ca.norm_2(vel_inb)  # velocity norm
        V_slip = ca.norm_2(vel_slip_inb)  # velocity norm

        # Reynolds degradation for wing segments
        f_re_wing = self.reynolds_wing_degradation(V, ac_x * 4)
        f_re_wing = ca.if_else(
            f_re_wing > 1.0, 1.0, f_re_wing
        )  # Ensure f_re_wing is not larger than 1.0, since it is a degradation factor
        f_re_slip = self.reynolds_wing_degradation(V_slip, self.mac_root)
        f_re_slip = ca.if_else(
            f_re_slip > 1.0, 1.0, f_re_slip
        )  # Ensure f_re_slip is not larger than 1.0, since it is a degradation factor

        alpha_wing_inb = ca.atan(
            -vel_inb[2] / vel_inb[0]
        )  # angle of attack, valid when vel_u > 0
        alpha_slip_inb = ca.atan(
            -vel_slip_inb[2] / vel_slip_inb[0]
        )  # angle of attack, valid when vel_u > 0
        beta_wing = ca.asin(vel_inb[1] / V)  # angle of side slip, valid when V > 0
        beta_wing_slip = ca.asin(
            vel_slip_inb[1] / V_slip
        )  # angle of side slip, valid when V > 0

        # Dynamic coefficients
        c_l_dyn_qs_wing = 0.0  # (self.pi/2)*(-ome_q*ac_x*4)/V
        c_l_dyn_qs_slip = 0.0  # (self.pi/2)*(-ome_q*ac_x*4)/V_slip

        wing_cl, wing_cd, wing_dxm = self.wing_coefficients(
            alpha_wing_inb,
            self.alpha_stall_wing,
            AR,
            ac_x,
            geo_center_vec[0],
            pos_cg[0],
            f_re=f_re_wing,
            c_d_0=self.c_d_0_wing,
            c_l_dyn_pre_st=c_l_dyn_qs_wing,
        )
        slip_cl, slip_cd, slip_dxm = self.wing_coefficients(
            alpha_slip_inb,
            self.alpha_stall_wing,
            AR,
            ac_x,
            geo_center_vec[0],
            pos_cg[0],
            f_re=f_re_slip,
            c_d_0=self.c_d_0_wing,
            c_l_dyn_pre_st=c_l_dyn_qs_slip,
        )
        F_x_slip = (
            (0.5 * self.rho * S_slip * V_slip**2)
            * (ca.cos(beta_wing_slip) ** 2)
            * (slip_cl * ca.sin(alpha_slip_inb) - slip_cd * ca.cos(alpha_slip_inb))
        )
        F_x_wing = (
            (0.5 * self.rho * S_free_wing * V**2)
            * (ca.cos(beta_wing) ** 2)
            * (wing_cl * ca.sin(alpha_wing_inb) - wing_cd * ca.cos(alpha_wing_inb))
        )  # In body frame (forward x)
        F_z_slip = (
            (0.5 * self.rho * S_slip * V_slip**2)
            * (ca.cos(beta_wing_slip) ** 2)
            * (slip_cl * ca.cos(alpha_slip_inb) + slip_cd * ca.sin(alpha_slip_inb))
        )
        F_z_wing = (
            (0.5 * self.rho * S_free_wing * V**2)
            * (ca.cos(beta_wing) ** 2)
            * (wing_cl * ca.cos(alpha_wing_inb) + wing_cd * ca.sin(alpha_wing_inb))
        )  # In body frame (upward z)
        F_y_wing = 0.0
        F_y_slip = 0.0

        F = ca.vertcat(F_x_wing, F_y_wing, F_z_wing) + ca.vertcat(
            F_x_slip, F_y_slip, F_z_slip
        )  # Force vector in body frame
        r_ac_avg = ca.vertcat(
            (wing_dxm * S_free_wing + slip_dxm * S_slip) / S,
            geo_center_vec[1],
            geo_center_vec[2],
        )  # Position vector of aerodynamic center in body frame relative to Wing L.E. (in body frame)
        # print(r_ac_avg)
        # r_ac_slip = ca.vertcat(slip_dxm, geo_center_vec[1], geo_center_vec[2]) #Position vector of aerodynamic center in body frame relative to Wing L.E. (in body frame)
        M = ca.cross(r_ac_avg, F) + ca.vertcat(
            0.0, (0.5 * self.rho * (4 * ac_x) * self.c_m_0_wing * S * V**2), 0.0
        )
        eps_down_tail = ca.vertcat(
            0.0, 0.0, 0.0
        )  # ca.if_else(alpha_wing_inb < self.alpha_stall_wing, ((2*wing_cl)/(self.pi*AR)), 0) #positive when Cl positive, when V > 0, original: 2*c_l / pi*AR, with cl here assumed as wing_cl and V as free stream velocity

        return F, M, eps_down_tail

    def reynolds_wing_degradation(self, V_inb, mac_wing):

        Re = V_inb * mac_wing / (self.nu)

        f = (
            1 - ((self.Re_ref - Re) / (self.Re_ref)) ** self.M_Re
        )  # (Re/self.Re_ref)**(self.M_Re)
        # print(f)

        return f

    def hor_tail_aerodynamics(
        self, V_slip_tail, vel, ome, vel_eps_down_tail, pos_cg, ele
    ):

        pos_ele_cg_le = self.pos_ele_le - pos_cg
        pos_ele_cg_cp = pos_ele_cg_le + ca.vertcat(
            -self.c_ele / 2, 0.0, 0.0
        )  # Position of elevator center of pressure in body frame relative to C.G.
        # print(d_x_ac_cg)

        vel_inb_slip = ca.vertcat(
            -V_slip_tail, 0, 0
        )  # Slipstream velocity in body frame
        vel_inb = (
            -(vel + ca.cross(ome, pos_ele_cg_cp)) + vel_eps_down_tail + vel_inb_slip
        )  # Body frame tail inbound velocity corrected for quasi-steady rotation and downwash

        V = ca.norm_2(vel_inb)
        alpha_hor_tail_inb = ca.atan(
            -vel_inb[2] / vel_inb[0]
        )  # - eps_down_tail # angle of attack, valid when vel_u > 0
        # beta_hor_tail_inb = ca.asin(vel_inb[1]/V) #angle of side slip, valid when V > 0
        # print(eps_down_tail)

        # print(ele)
        alpha_ele_eff = self.k_alpha_elev * ele + alpha_hor_tail_inb
        # c_l, c_d, d_x_t = self.wing_coefficients(alpha_eff, self.alpha_stall_tail, AR_tail, self.c_hor_tail/4, self.c_hor_tail/2, pos_ele_le_cg, f_re = 1.0, c_d_0 = self.c_d_0_tail, c_l_dyn_pre_st=0.0)
        c_l = 2 * ca.sin(alpha_ele_eff) * ca.cos(alpha_ele_eff)
        c_d = self.c_d_0_tail + 2 * (ca.sin(alpha_ele_eff) ** 2)
        d_x_t = self.c_hor_tail / 4 + (2 * ca.sqrt(alpha_ele_eff**2) / self.pi) * (
            self.c_hor_tail / 4
        )
        pos_ele_cg_ac = pos_ele_cg_le + ca.vertcat(-d_x_t, 0.0, 0.0)

        # print(pos_ele_cg_ac)

        F_l = 0.5 * self.rho * self.s_hor_tail * c_l * V**2
        F_d = 0.5 * self.rho * self.s_hor_tail * c_d * V**2

        F_x = F_l * ca.sin(alpha_ele_eff) - F_d * ca.cos(alpha_ele_eff)
        F_z = F_l * ca.cos(alpha_ele_eff) + F_d * ca.sin(alpha_ele_eff)

        F = ca.vertcat(F_x, 0.0, F_z)  # Force vector in body frame
        M = ca.cross(pos_ele_cg_ac, F)  # Moment vector in body frame

        return F, M

    def vertical_tail_aerodynamics(self, V_slip_tail, vel, ome, pos_cg, rud):

        pos_rud_cg_le = self.pos_rud_le - pos_cg
        pos_rud_cg_cp = pos_rud_cg_le + ca.vertcat(-self.c_rud / 2, 0.0, 0.0)

        vel_inb = -(vel + ca.cross(ome, pos_rud_cg_cp)) - ca.vertcat(
            V_slip_tail, 0, 0
        )  # Body frame tail inbound velocity corrected for quasi-steady rotation (neglect downwash)

        V = ca.norm_2(vel_inb)
        alpha_vert_tail_inb = ca.asin(
            vel_inb[1] / V
        )  # angle of attack, valid when vel_u > 0

        alpha_rud_eff = self.k_alpha_rudder * rud + alpha_vert_tail_inb
        c_l = 2 * ca.sin(alpha_rud_eff) * ca.cos(alpha_rud_eff)
        c_d = 2 * (ca.sin(alpha_rud_eff) ** 2)
        d_x_t = self.c_vert_tail / 4 + (2 * ca.sqrt(alpha_rud_eff**2) / self.pi) * (
            self.c_vert_tail / 4
        )
        pos_rud_cg_ac = pos_rud_cg_le + ca.vertcat(-d_x_t, 0.0, 0.0)

        F_l = 0.5 * self.rho * self.s_vert_tail * c_l * V**2
        F_d = 0.5 * self.rho * self.s_vert_tail * c_d * V**2

        F_y = F_l * ca.cos(alpha_rud_eff) + F_d * ca.sin(alpha_rud_eff)
        F_x = F_l * ca.sin(alpha_rud_eff) - F_d * ca.cos(alpha_rud_eff)

        F = ca.vertcat(F_x, F_y, 0.0)  # Force vector in body frame
        M = ca.cross(pos_rud_cg_ac, F)  # Moment vector in body frame

        return F, M

    def sweep_servo_dynamics(self, x, u):

        # print(x)
        # print(u)
        # x = [theta_sw_norm, theta_sw_norm_dot]
        # x0 = theta_sw_norm_between -1.0 and 1.0
        x_sw_dot = self.A_sw @ x + self.B_sw @ u
        theta_sw_norm = self.C_sw @ x

        theta_sw_norm = (theta_sw_norm + (self.sweep_offset / 2)) / (
            1 - self.sweep_offset / 2
        )
        theta_sweep = (self.sw_max - self.sw_min) * (theta_sw_norm) / 2 + (
            self.sw_max + self.sw_min
        ) / 2  # In degrees
        # Implement a delay in MPC defining function

        # print(u)
        # print(x)
        # print(theta_sw_norm)
        # print(theta_sweep)

        return theta_sweep, x_sw_dot

    def tail_servo_dynamics(self, x, u):
        x_ele_dot = u
        ele_norm = x

        ele_ang_rad = (
            (self.ele_max - self.ele_min) * (ele_norm) / 2
            + (self.ele_max + self.ele_min) / 2
        ) * (
            self.pi / 180
        )  # In radians

        return ele_ang_rad, x_ele_dot

    def motor_throttle_to_omega(self, u_thr):

        c_0 = self.motor_omega_map
        c_1 = (self.motor_omega_map * (self.throttle_offset**2 - 1) + 1) / (
            1 - self.throttle_offset
        )
        c_2 = 1 - c_0 - c_1
        omega_norm = c_0 * ((u_thr) ** 2) + c_1 * u_thr + c_2

        # print(u_thr)
        # print(omega_norm)
        # omega_norm = self.motor_omega_map * (u_thr - self.throttle_offset)**2 + (1/(1-self.throttle_offset) - self.motor_omega_map*(1+self.throttle_offset))*(u_thr- self.throttle_offset)

        return omega_norm

    def motor_dynamics(self, omega_mot_norm, u_thr):

        # u_thr = np.clip(u_thr, self.throttle_offset, 1.0)

        # u_thr is between u_thr_offset and 1 (must be clipped)
        omega_mot_des_norm = self.motor_throttle_to_omega(u_thr)
        omega_mot_norm_dot = self.motor_tau_inv * (omega_mot_des_norm - omega_mot_norm)

        thrust = (omega_mot_norm**2) * (self.T_max - self.T_min) + self.T_min
        F = ca.vertcat(
            thrust, 0.0, 0.0
        )  # Force vector in body frame (thrust in forward x)

        # Implement a delay in MPC defining function

        return F, omega_mot_norm_dot

    def dynamics_acts_numeric(self, x_act, u):
        omega_mot_norm, x_sw_sym_0, x_sw_sym_1, x_ele = (
            x_act[0],
            x_act[1],
            x_act[2],
            x_act[3],
        )
        u_thr, u_ele, u_sw_sym = u[0], u[1], u[2]

        x_sw_sym = np.array([[x_sw_sym_0], [x_sw_sym_1]])
        x_sw_sym_dot = (
            np.array(self.A_sw) @ x_sw_sym
            + np.array(self.B_sw) @ np.array([[u_sw_sym]])
        ).flatten()
        # print(x_sw_sym_dot)
        thrust, omega_mot_norm_dot = self.motor_dynamics(omega_mot_norm, u_thr)
        # print(omega_mot_norm_dot)
        x_ele_dot = u_ele

        return [omega_mot_norm_dot, x_sw_sym_dot[0], x_sw_sym_dot[1], x_ele_dot]

    def force_moment_cg_total(
        self,
        x: NDArray,
        u: NDArray,
        theta_sw_l=None,
        theta_sw_r=None,
        ele_rad=None,
        rud_rad=None,
    ):
        # State vector x ordered as: [pos_x, pos_y, pos_z, vel_u, vel_v, vel_w, q_x, q_y, q_z, q_w, ome_p, ome_q, ome_r, omega_mot_norm, x_sw_sym_l_0, x_sw_sym_l_1, x_sw_sym_r_0, x_sw_sym_r_1, x_ele, x_rud]
        # Control vector u ordered as: [u_thr, u_sw_l, u_sw_r, u_ele_dot, u_rud_dot]
        (
            pos_x,
            pos_y,
            pos_z,
            vel_u,
            vel_v,
            vel_w,
            q_x,
            q_y,
            q_z,
            q_w,
            ome_p,
            ome_q,
            ome_r,
            omega_mot_norm,
            x_sw_sym_l_0,
            x_sw_sym_l_1,
            x_sw_sym_r_0,
            x_sw_sym_r_1,
            x_ele,
            x_rud,
        ) = (
            x[0],
            x[1],
            x[2],
            x[3],
            x[4],
            x[5],
            x[6],
            x[7],
            x[8],
            x[9],
            x[10],
            x[11],
            x[12],
            x[13],
            x[14],
            x[15],
            x[16],
            x[17],
            x[18],
            x[19],
        )
        u_thr, u_sw_l, u_sw_r, u_ele_dot, u_rud_dot = u[0], u[1], u[2], u[3], u[4]

        quat = ca.vertcat(q_x, q_y, q_z, q_w)  # Quaternion vector
        vel_body = ca.vertcat(vel_u, vel_v, vel_w)  # Velocity vector in body frame
        omega_body = ca.vertcat(
            ome_p, ome_q, ome_r
        )  # Angular velocity vector in body frame

        # Quaternion to rotation matrix
        r = sc.Rotation.from_quat(quat)  # Rotation matrix from quaternion
        R = r.as_matrix()  # Rotation matrix from quaternion

        # COORDINATES USING X,Y,Z: Forward, Left, Up
        # Pitch angle defined as positive about positive Y (downwards pitch: positive)

        # Actuator model update
        if ele_rad is None:
            ele_rad, x_ele_dot = self.tail_servo_dynamics(x_ele, u_ele_dot)
        if rud_rad is None:
            rud_rad, x_rud_dot = self.tail_servo_dynamics(x_rud, u_rud_dot)

        if theta_sw_l is None:
            x_sw_l = ca.vertcat(x_sw_sym_l_0, x_sw_sym_l_1)
            theta_sw_l, x_sw_dot_r = self.sweep_servo_dynamics(x_sw_l, u_sw_l)
        if theta_sw_r is None:
            x_sw_r = ca.vertcat(x_sw_sym_r_0, x_sw_sym_r_1)
            theta_sw_r, x_sw_dot_l = self.sweep_servo_dynamics(x_sw_r, u_sw_r)

        # Wing geometries
        AR_l, S_l, b_l, ac_x_l, geo_x_l, delta_pos_cg_x_l = self.wing_geometry(
            theta_sw_l
        )
        AR_r, S_r, b_r, ac_x_r, geo_x_r, delta_pos_cg_x_r = self.wing_geometry(
            theta_sw_r
        )
        # Negate right geo_x_r and delta_pos_cg_x_r to account for right wing being mirrored
        geo_x_r[1] = -geo_x_r[1]
        delta_pos_cg_x_r[1] = -delta_pos_cg_x_r[1]

        # Correct C.G. location to account for wing sweep
        pos_cg = self.pos_cg_theta_sw_0 + (
            delta_pos_cg_x_l + delta_pos_cg_x_r
        )  # Position of C.G. in body frame relative to leading edge of wing offset from C.G., including both wing sweeps

        # Graviational force in body frame
        F_grav = ca.inv(R) @ ca.vertcat(
            0.0, 0.0, -self.m * self.g
        )  # Gravitational force in body frame (downward z in world frame acting)
        # Thrust Force
        F_thrust, omega_mot_norm_dot = self.motor_dynamics(omega_mot_norm, u_thr)
        # Wing and tail slipsteam velocities
        V_slip_wing, V_slip_tail = self.thrust_slipstream(F_thrust[0], vel_u)
        # Left wing forces and moments
        F_wing_left, M_cg_wing_left, vel_down_tail_left = self.wing_aerodynamics(
            V_slip_wing, vel_body, omega_body, AR_l, S_l, ac_x_l, geo_x_l, pos_cg
        )
        # Right wing forces and moments
        F_wing_right, M_cg_wing_right, vel_down_tail_right = self.wing_aerodynamics(
            V_slip_wing, vel_body, omega_body, AR_r, S_r, ac_x_r, geo_x_r, pos_cg
        )
        # Horizontal tail forces and moments
        F_hor_tail, M_cg_hor_tail = self.hor_tail_aerodynamics(
            V_slip_tail,
            vel_body,
            omega_body,
            vel_down_tail_left + vel_down_tail_right,
            pos_cg,
            ele_rad,
        )
        # Vertical tail forces and moments
        F_vert_tail, M_cg_vert_tail = self.vertical_tail_aerodynamics(
            V_slip_tail, vel_body, omega_body, pos_cg, rud_rad
        )

        # Total forces and moments in body frame
        F_total = (
            F_wing_left + F_wing_right + F_hor_tail + F_vert_tail + F_thrust
        )  # Force vector in body frame
        M_total = (
            M_cg_wing_left
            + M_cg_wing_right
            + M_cg_hor_tail
            + M_cg_vert_tail
            + ca.cross(self.pos_prop - pos_cg, F_thrust)
            + 0.5
            * self.rho
            * (vel_u**2 + vel_v**2 + vel_w**2)
            * ca.vertcat(
                self.c_m_x_drone_static,
                self.c_m_y_drone_static,
                self.c_m_z_drone_static,
            )
        )  # Moment vector in body frame

        return F_total, M_total
