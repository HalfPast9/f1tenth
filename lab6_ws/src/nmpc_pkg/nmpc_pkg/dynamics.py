import casadi as ca
from nmpc_pkg.params import VehicleParams, SolverParams

G = 9.81


def build_dynamics(vp=VehicleParams(), sp=SolverParams()):
    """Dynamic single-track model matching the f1tenth_gym simulator.

    State: [px, py, delta, vel, yaw, yaw_rate, slip_angle] (7 states)
    Control: [sv, accl] (2 inputs)
      - sv: steering velocity [rad/s]
      - accl: longitudinal acceleration [m/s²]

    At low speed (|vel| < vx_guard), uses kinematic model to avoid
    singularities in the dynamic model's 1/vel terms.

    Returns (f_continuous, f_discrete)
    """
    nx = 7
    nu = 2
    x = ca.MX.sym('x', nx)
    u = ca.MX.sym('u', nu)

    px, py, delta, vel, yaw, yaw_rate, slip = (
        x[0], x[1], x[2], x[3], x[4], x[5], x[6])
    sv, accl = u[0], u[1]

    lwb = vp.l_f + vp.l_r
    mu = vp.mu
    C_Sf = vp.C_Sf
    C_Sr = vp.C_Sr
    lf = vp.l_f
    lr = vp.l_r
    h = vp.h_cg
    m = vp.m
    I = vp.I_z

    # --- Kinematic model (low speed) ---
    f_ks = ca.vertcat(
        vel * ca.cos(yaw),
        vel * ca.sin(yaw),
        sv,
        accl,
        vel / lwb * ca.tan(delta),
        accl / lwb * ca.tan(delta) + vel / (lwb * ca.cos(delta)**2) * sv,
        0.0,
    )

    # --- Dynamic single-track model (high speed) ---
    # Guard vel to avoid division by zero (kinematic branch handles low speed)
    vel_safe = ca.fmax(ca.fabs(vel), 0.1)

    f_st = ca.vertcat(
        vel * ca.cos(slip + yaw),
        vel * ca.sin(slip + yaw),
        sv,
        accl,
        yaw_rate,
        -mu * m / (vel_safe * I * (lr + lf)) * (
            lf**2 * C_Sf * (G * lr - accl * h)
            + lr**2 * C_Sr * (G * lf + accl * h)) * yaw_rate
        + mu * m / (I * (lr + lf)) * (
            lr * C_Sr * (G * lf + accl * h)
            - lf * C_Sf * (G * lr - accl * h)) * slip
        + mu * m / (I * (lr + lf)) * lf * C_Sf * (G * lr - accl * h) * delta,
        (mu / (vel_safe**2 * (lr + lf)) * (
            C_Sr * (G * lf + accl * h) * lr
            - C_Sf * (G * lr - accl * h) * lf) - 1) * yaw_rate
        - mu / (vel_safe * (lr + lf)) * (
            C_Sr * (G * lf + accl * h)
            + C_Sf * (G * lr - accl * h)) * slip
        + mu / (vel_safe * (lr + lf)) * C_Sf * (G * lr - accl * h) * delta,
    )

    # Smooth blend: use tanh to transition between kinematic and dynamic
    blend = 0.5 * (1.0 + ca.tanh(10.0 * (ca.fabs(vel) - sp.vx_guard)))
    x_dot = (1.0 - blend) * f_ks + blend * f_st

    f_continuous = ca.Function('f_continuous', [x, u], [x_dot])
    f_discrete = ca.Function('f_discrete', [x, u],
                             [x + sp.T_s * x_dot])
    return f_continuous, f_discrete
