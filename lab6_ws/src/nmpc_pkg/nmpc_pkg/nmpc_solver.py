import time
import numpy as np
import casadi as ca
from nmpc_pkg.params import VehicleParams, SolverParams
from nmpc_pkg.dynamics import build_dynamics


class NMPCSolver:
    def __init__(self, vp=VehicleParams(), sp=SolverParams()):
        self.vp = vp
        self.sp = sp
        self.N = sp.N
        self.nx = 7   # [px, py, delta, vel, yaw, yaw_rate, slip_angle]
        self.nu = 2   # [sv, accl]

        _, self.f_discrete = build_dynamics(vp, sp)

        self._warm_x = None
        self._warm_u = None

        self._build_nlp()

    def _build_nlp(self):
        vp = self.vp
        sp = self.sp
        N, nx, nu = self.N, self.nx, self.nu

        X = ca.MX.sym('X', nx, N + 1)
        U = ca.MX.sym('U', nu, N)

        # Params: x0(7) + ref_xy(2*(N+1)) + u_prev(2) + v_target(N) + ref_heading(N+1)
        n_params = nx + 2 * (N + 1) + nu + N + (N + 1)
        P = ca.MX.sym('P', n_params)

        idx = 0
        x0 = P[idx:idx + nx]; idx += nx
        p_ref = ca.reshape(P[idx:idx + 2 * (N + 1)], 2, N + 1); idx += 2 * (N + 1)
        u_prev = P[idx:idx + nu]; idx += nu
        v_target = P[idx:idx + N]; idx += N
        phi_ref = P[idx:idx + (N + 1)]

        track_margin = (sp.R_g - sp.R_c) ** 2

        cost = 0.0
        g = []
        lbg_list = []
        ubg_list = []

        g.append(X[:, 0] - x0)
        lbg_list.extend([0.0] * nx)
        ubg_list.extend([0.0] * nx)

        for k in range(N):
            # Position tracking
            dp = X[:2, k] - p_ref[:, k]
            cost += sp.Q1_stage_px * dp[0]**2 + sp.Q1_stage_py * dp[1]**2

            # Heading tracking (yaw is state index 4)
            heading_err = X[4, k] - phi_ref[k]
            cost += sp.Q4_heading * ca.sin(heading_err)**2

            # Speed tracking (vel is state index 3)
            cost += sp.Q3_speed_track * (X[3, k] - v_target[k])**2

            # Control smoothness
            if k == 0:
                du = U[:, 0] - u_prev
            else:
                du = U[:, k] - U[:, k - 1]
            cost += sp.Q2_sv * du[0]**2 + sp.Q2_accl * du[1]**2

            # Dynamics constraint
            x_next = self.f_discrete(X[:, k], U[:, k])
            g.append(X[:, k + 1] - x_next)
            lbg_list.extend([0.0] * nx)
            ubg_list.extend([0.0] * nx)

            # Track boundary constraint
            dist_sq = (X[0, k] - p_ref[0, k])**2 + (X[1, k] - p_ref[1, k])**2
            g.append(dist_sq)
            lbg_list.append(0.0)
            ubg_list.append(track_margin)

        # Terminal cost
        dp_N = X[:2, N] - p_ref[:, N]
        cost += sp.Q1_terminal_px * dp_N[0]**2 + sp.Q1_terminal_py * dp_N[1]**2
        heading_err_N = X[4, N] - phi_ref[N]
        cost += sp.Q4_heading * 3.0 * ca.sin(heading_err_N)**2

        dist_sq_N = (X[0, N] - p_ref[0, N])**2 + (X[1, N] - p_ref[1, N])**2
        g.append(dist_sq_N)
        lbg_list.append(0.0)
        ubg_list.append(track_margin)

        opt_vars = ca.vertcat(ca.reshape(X, nx * (N + 1), 1),
                              ca.reshape(U, nu * N, 1))
        g = ca.vertcat(*g)

        nlp = {'f': cost, 'x': opt_vars, 'g': g, 'p': P}

        opts = {
            'ipopt.print_level': 0,
            'ipopt.max_iter': 100,
            'ipopt.tol': 1e-4,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.warm_start_bound_push': 1e-6,
            'ipopt.warm_start_mult_bound_push': 1e-6,
            'ipopt.mu_init': 1e-5,
            'print_time': 0,
        }
        self.solver = ca.nlpsol('nmpc', 'ipopt', nlp, opts)

        n_opt = nx * (N + 1) + nu * N
        self.lbx = -np.inf * np.ones(n_opt)
        self.ubx = np.inf * np.ones(n_opt)

        for k in range(N + 1):
            base = k * nx
            self.lbx[base + 2] = vp.s_min       # delta (steering angle)
            self.ubx[base + 2] = vp.s_max
            self.lbx[base + 3] = 0.0             # vel >= 0
            self.ubx[base + 3] = vp.v_max

        self.x_offset = nx * (N + 1)
        for k in range(N):
            base = self.x_offset + k * nu
            self.lbx[base + 0] = vp.sv_min       # sv
            self.ubx[base + 0] = vp.sv_max
            self.lbx[base + 1] = -vp.a_max       # accl
            self.ubx[base + 1] = vp.a_max

        self.lbg = np.array(lbg_list)
        self.ubg = np.array(ubg_list)
        self.n_params = n_params

    def solve(self, x0_7, reference, max_speeds, ref_headings, u_prev):
        sp = self.sp
        vp = self.vp
        N, nx, nu = self.N, self.nx, self.nu

        x0 = x0_7.copy()

        # Unwrap yaw (index 4) to be continuous with warm start
        if self._warm_x is not None:
            yaw_prev = self._warm_x[4]
            x0[4] = yaw_prev + np.arctan2(np.sin(x0[4] - yaw_prev),
                                           np.cos(x0[4] - yaw_prev))

        current_speed = max(x0[3], 0.5)
        dist_per_step = current_speed * sp.T_s
        points_per_step = max(1, int(dist_per_step / sp.d_s))
        indices = np.minimum(
            np.arange(N + 1) * points_per_step,
            len(reference) - 1)
        ref_points = reference[indices]
        ref_phi = ref_headings[indices].copy()

        # Unwrap reference headings relative to current yaw
        yaw = x0[4]
        for i in range(len(ref_phi)):
            ref_phi[i] = yaw + np.arctan2(np.sin(ref_phi[i] - yaw),
                                           np.cos(ref_phi[i] - yaw))

        speed_indices = np.minimum(
            np.arange(N) * points_per_step,
            len(max_speeds) - 1)
        target_speeds = max_speeds[speed_indices]

        # Speed limit on vel state via acceleration constraints
        # (no direct v_cmd anymore — speed is controlled through accl)
        ubx = self.ubx.copy()
        for k in range(N + 1):
            ubx[k * nx + 3] = min(float(target_speeds[min(k, N-1)]),
                                  float(vp.v_max))

        p = np.concatenate([x0, ref_points.flatten(), u_prev,
                            target_speeds, ref_phi])

        if self._warm_x is not None:
            x_init = self._warm_x
            u_init = self._warm_u
        else:
            x_init = np.tile(x0, N + 1)
            u_init = np.tile(np.array([0.0, 0.0]), N)

        x0_nlp = np.concatenate([x_init, u_init])

        t0 = time.time()
        sol = self.solver(
            x0=x0_nlp,
            lbx=self.lbx, ubx=ubx,
            lbg=self.lbg, ubg=self.ubg,
            p=p,
        )
        solve_time = (time.time() - t0) * 1000.0

        opt = np.array(sol['x']).flatten()
        X_sol = opt[:nx * (N + 1)].reshape(N + 1, nx)
        U_sol = opt[nx * (N + 1):].reshape(N, nu)

        self._warm_x = np.concatenate([X_sol[1:].flatten(), X_sol[-1]])
        self._warm_u = np.concatenate([U_sol[1:].flatten(), U_sol[-1]])

        # Output: the sim expects [speed_cmd, steer_cmd]
        # We invert the PID to find what commands produce our desired [accl, sv]
        # Sim PID: accl = kp * (speed_cmd - vel), sv = sign(steer_cmd - delta) * sv_max
        #
        # For speed: speed_cmd = vel + accl / kp
        # For steering: the sim uses bang-bang sv, so we command the target angle
        #   directly — the sim will drive toward it at sv_max
        vel_now = float(X_sol[0, 3])
        accl_desired = float(U_sol[0, 1])

        if vel_now > 0:
            kp = 10.0 * vp.a_max / vp.v_max
        else:
            kp = 2.0 * vp.a_max / vp.v_max
        speed_cmd = vel_now + accl_desired / max(kp, 0.1)
        speed_cmd = np.clip(speed_cmd, 0.0, float(vp.v_max))

        # For steering: command the predicted angle at step 1
        steer_cmd = float(X_sol[1, 2])
        steer_cmd = np.clip(steer_cmd, float(vp.s_min), float(vp.s_max))

        u_opt = np.array([speed_cmd, steer_cmd])
        u_internal = np.array([float(U_sol[0, 0]), float(U_sol[0, 1])])
        info = {
            'time_ms': solve_time,
            'status': str(self.solver.stats()['return_status']),
            'cost': float(sol['f']),
            'u_internal': u_internal,
        }
        return u_opt, X_sol, info

    def reset_warm_start(self):
        self._warm_x = None
        self._warm_u = None
