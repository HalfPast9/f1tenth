from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleParams:
    l_f: float = 0.15875
    l_r: float = 0.17145
    m: float = 3.74
    I_z: float = 0.04712
    h_cg: float = 0.074
    C_Sf: float = 4.718
    C_Sr: float = 5.4562
    mu: float = 1.0489
    length: float = 0.58
    width: float = 0.31

    # Actuator limits (matched to sim)
    s_min: float = -0.4189
    s_max: float = 0.4189
    sv_min: float = -3.2
    sv_max: float = 3.2
    v_switch: float = 7.319
    a_max: float = 9.51
    v_min: float = -5.0
    v_max: float = 20.0


@dataclass(frozen=True)
class SolverParams:
    N: int = 15
    T_s: float = 0.05
    P: int = 90
    d_s: float = 0.1

    # Cost weights
    Q1_stage_px: float = 8.0
    Q1_stage_py: float = 8.0
    Q1_terminal_px: float = 30.0
    Q1_terminal_py: float = 30.0
    Q2_accl: float = 0.5
    Q2_sv: float = 3.0
    Q3_speed_track: float = 1.5
    Q4_heading: float = 8.0

    # Track
    R_c: float = 0.33
    R_g: float = 1.5

    vx_guard: float = 0.5
