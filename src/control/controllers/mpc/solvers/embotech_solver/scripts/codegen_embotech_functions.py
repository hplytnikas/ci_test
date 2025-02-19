import yaml
from os.path import dirname, join, abspath
import numpy as np
import forcespro.nlp
import casadi
from casadi import SX, vertcat, sin, cos, atan2, atan, fmax, tan, fabs, log, exp, sqrt


# === Read and parse yaml file ===
def readYaml():
    constraint_param_path = join(
        dirname(abspath(__file__)),
        "../../mpc_solver_common/config/constraint_parameters.yaml",
    )
    cost_param_path = join(
        dirname(abspath(__file__)),
        "../../mpc_solver_common/config/cost_parameters.yaml",
    )
    model_param_path = join(
        dirname(abspath(__file__)),
        "../../mpc_solver_common/config/model_parameters.yaml",
    )
    embotech_solver_param_path = join(
        dirname(abspath(__file__)), "../config/embotech_solver_parameters.yaml"
    )
    common_solver_param_path = join(
        dirname(abspath(__file__)),
        "../../mpc_solver_common/config/solver_parameters.yaml",
    )

    with open(constraint_param_path, "r") as stream:
        constraint_params = yaml.safe_load(stream)

    with open(cost_param_path, "r") as stream:
        cost_params = yaml.safe_load(stream)

    with open(model_param_path, "r") as stream:
        model_params = yaml.safe_load(stream)

    with open(embotech_solver_param_path, "r") as stream:
        embotech_solver_params = yaml.safe_load(stream)

    with open(common_solver_param_path, "r") as stream:
        common_solver_params = yaml.safe_load(stream)

    for item in constraint_params:
        if item == None:
            raise ValueError("None value in constraints parameters")
    for item in cost_params:
        if item == None:
            raise ValueError("None value in cost parameters")
    for item in model_params:
        if item == None:
            raise ValueError("None value in model parameters")
    for item in embotech_solver_params:
        if item == None:
            raise ValueError("None value in embotech solver parameters")
    for item in common_solver_params:
        if item == None:
            raise ValueError("None value in common solver parameters")

    return (
        constraint_params,
        cost_params,
        model_params,
        embotech_solver_params,
        common_solver_params,
    )


### ================== Set the solver options ================== ###
def getCodeoptions():
    _, _, _, embotech_solver_params, common_solver_params = readYaml()

    codeoptions = forcespro.CodeOptions("embotech")

    codeoptions.parallel = embotech_solver_params["parallel"]

    codeoptions.printlevel = embotech_solver_params["print_level"]
    codeoptions.maxit = embotech_solver_params["max_it"]
    codeoptions.optlevel = embotech_solver_params["opt_level"]
    codeoptions.overwrite = embotech_solver_params["overwrite"]
    codeoptions.server = embotech_solver_params["server"]
    codeoptions.BuildSimulinkBlock = embotech_solver_params["simulink"]
    codeoptions.cleanup = embotech_solver_params["cleanup"]

    codeoptions.platform = embotech_solver_params["platform"]

    codeoptions.nlp.integrator.type = embotech_solver_params["integrator_type"]
    codeoptions.nlp.integrator.Ts = (
        common_solver_params["T_final"] / common_solver_params["N_horizon"]
    )
    codeoptions.nlp.integrator.nodes = embotech_solver_params["integrator_nodes_solver"]

    codeoptions.nlp.hessian_approximation = embotech_solver_params["hessian_approx"]

    codeoptions.nlp.checkFunctions = embotech_solver_params["check_functions"]

    codeoptions.nlp.linear_solver = embotech_solver_params["linear_solver"]

    codeoptions.solvemethod = embotech_solver_params["solve_method"]
    codeoptions.init = embotech_solver_params["initialisation"]
    codeoptions.mu0 = embotech_solver_params["mu_zero"]
    codeoptions.accuracy.ineq = embotech_solver_params["accuracy_ineq"]
    codeoptions.accuracy.eq = embotech_solver_params["accuracy_eq"]
    codeoptions.accuracy.mu = embotech_solver_params["accuracy_mu"]
    codeoptions.accuracy.rdgap = embotech_solver_params["accuracy_rdgap"]

    codeoptions.nlp.TolStat = embotech_solver_params["tol_stat"]
    codeoptions.nlp.TolEq = embotech_solver_params["tol_eq"]
    codeoptions.nlp.TolIneq = embotech_solver_params["tol_ineq"]
    codeoptions.nlp.TolComp = embotech_solver_params["tol_comp"]

    codeoptions.noVariableElimination = embotech_solver_params["variable_elimination"]
    codeoptions.nohash = embotech_solver_params["no_hash"]
    codeoptions.nlp.integrator.attempt_subsystem_exploitation = embotech_solver_params[
        "integrator_sub_system"
    ]
    codeoptions.nlp.BarrStrat = embotech_solver_params["barr_strat"]

    codeoptions.sse = embotech_solver_params["sse"]

    codeoptions.optimize_choleskydivision = embotech_solver_params[
        "optimize_choleskydivision"
    ]
    codeoptions.optimize_registers = embotech_solver_params["optimize_registers"]
    codeoptions.optimize_uselocalsall = embotech_solver_params["optimize_uselocalsall"]
    codeoptions.optimize_operationsrearrange = embotech_solver_params[
        "optimize_rearrange"
    ]
    codeoptions.optimize_loopunrolling = embotech_solver_params[
        "optimize_loopunrolling"
    ]
    codeoptions.optimize_enableoffset = embotech_solver_params["optimize_enableoffset"]

    return codeoptions


### ================== Dynamic model ================== ###
def getContinuousDynamics(x, u, p, idx_params, model_params):
    # Runtime parameters
    m = p[idx_params["m"]]
    l_f = p[idx_params["l_f"]]
    l_r = p[idx_params["l_r"]]
    C_d = p[idx_params["C_d"]]
    C_r = p[idx_params["C_r"]]
    C_l = p[idx_params["C_l"]]
    h_cg = p[idx_params["h_cg"]]
    B_tire_lat = p[idx_params["B_tire_lat"]]
    C_tire_lat = p[idx_params["C_tire_lat"]]
    D_tire_lat = p[idx_params["D_tire_lat"]]
    Iz = p[idx_params["Iz"]]
    k = p[idx_params["kappa_ref"]]

    # Fixed parameters
    a_front = model_params["A_front"]
    g = model_params["g"]
    rho = model_params["rho"]

    # Define the inputs
    dax_ = u[0]
    ddel_s_ = u[1]

    # Define the state
    n = x[0]
    mu = x[1]
    vx = x[2]
    vy = x[3]
    dpsi = x[4]
    ax_ = x[5]
    del_s_ = x[6]

    # Resistance Forces
    F_rollres = m * g * C_r
    F_drag = 0.5 * rho * a_front * C_d * (vx**2)
    F_res = F_rollres + F_drag

    # Slip Angles
    vx_n = fmax(1.0, vx)
    alpha_f = atan2(vy + l_f * dpsi, vx_n) - del_s_
    alpha_r = atan2(vy - l_r * dpsi, vx_n)

    # Longitudinal Tire Forces
    Fx_f = m * ax_ / 2.0
    Fx_r = m * ax_ / 2.0

    # Approximate load transfer due to acceleration
    ax_v_est = (Fx_f + Fx_r - F_res) / m
    Fz_dyn_loadtransfer = m * ax_v_est * h_cg / (l_f + l_r)

    # Lateral Tire Forces
    F_downforce_per_axle = 0.5 * rho * C_l * a_front * (vx**2) / 2.0
    Fz_f = m * g * l_r / (l_f + l_r) + F_downforce_per_axle - Fz_dyn_loadtransfer
    Fz_r = m * g * l_f / (l_f + l_r) + F_downforce_per_axle + Fz_dyn_loadtransfer

    Fy_f = -Fz_f * D_tire_lat * sin(C_tire_lat * atan(B_tire_lat * alpha_f))
    Fy_r = -Fz_r * D_tire_lat * sin(C_tire_lat * atan(B_tire_lat * alpha_r))

    # accelerations in vehicle frame
    ax_v = (Fx_f * cos(del_s_) + Fx_r - Fy_f * sin(del_s_) - F_res) / m
    ay_v = (Fx_f * sin(del_s_) + Fy_f * cos(del_s_) + Fy_r) / m

    # derivative of state w.r.t time
    dxdt_n = vx * sin(mu) + vy * cos(mu)
    dxdt_mu = dpsi - k * (vx * cos(mu) - vy * sin(mu)) / (1 - n * k)
    dxdt_vx = ax_v + vy * dpsi
    dxdt_vy = ay_v - vx * dpsi
    dxdt_dpsi = (Fx_f * sin(del_s_) * l_f + Fy_f * cos(del_s_) * l_f - Fy_r * l_r) / Iz
    dxdt_ax = dax_
    dxdt_del = ddel_s_

    return vertcat(dxdt_n, dxdt_mu, dxdt_vx, dxdt_vy, dxdt_dpsi, dxdt_ax, dxdt_del)


### ================== Inequalities constraints ================== ###
def ineq(z, p, idx_params, model_params):

    n = z[0]
    mu = z[1]
    vx = z[2]
    vy = z[3]
    dpsi = z[4]
    ax = z[5]
    dels = z[6]

    dax = z[7]
    ddels = z[8]

    s_tb_fr = z[9]
    s_tb_fl = z[10]
    s_te_f = z[11]
    s_te_r = z[12]
    s_vx = z[13]

    rho = model_params["rho"]
    a_front = model_params["A_front"]
    g = model_params["g"]

    l_f = p[idx_params["l_f"]]
    l_r = p[idx_params["l_r"]]
    h_cg = p[idx_params["h_cg"]]
    car_width = p[idx_params["car_width"]]
    l_to_front = p[idx_params["l_to_front"]]
    l_to_rear = p[idx_params["l_to_rear"]]

    C_r = p[idx_params["C_r"]]
    C_d = p[idx_params["C_d"]]
    C_l = p[idx_params["C_l"]]
    B_tire_lat = p[idx_params["B_tire_lat"]]
    C_tire_lat = p[idx_params["C_tire_lat"]]
    D_tire_lat = p[idx_params["D_tire_lat"]]
    m = p[idx_params["m"]]
    axt_abs_max = p[idx_params["axt_abs_max"]]
    ayt_abs_max = p[idx_params["ayt_abs_max"]]

    trackbound_min = p[idx_params["trackbound_min"]]
    trackbound_max = p[idx_params["trackbound_max"]]

    vx_max = p[idx_params["vx_max"]]

    dels_abs_max = p[idx_params["dels_abs_max"]]
    dax_min = p[idx_params["dax_min"]]
    dax_max = p[idx_params["dax_max"]]
    ddels_abs_max = p[idx_params["ddels_abs_max"]]

    # Resistance Forces
    F_rollres = m * g * C_r
    F_drag = 0.5 * rho * a_front * C_d * (vx**2)
    F_res = F_rollres + F_drag

    # Slip Angles
    vx_n = fmax(1.0, vx)
    alpha_f = atan2(vy + l_f * dpsi, vx_n) - dels
    alpha_r = atan2(vy - l_r * dpsi, vx_n)

    # Longitudinal Tire Forces
    Fx_f = m * ax / 2.0
    Fx_r = m * ax / 2.0

    # Approximate load transfer due to acceleration
    ax_v_est = (Fx_f + Fx_r - F_res) / m
    Fz_dyn_loadtransfer = m * ax_v_est * h_cg / (l_f + l_r)

    # Lateral Tire Forces
    F_downforce_per_axle = 0.5 * rho * C_l * a_front * (vx**2) / 2.0
    Fz_f = m * g * l_r / (l_f + l_r) + F_downforce_per_axle - Fz_dyn_loadtransfer
    Fz_r = m * g * l_f / (l_f + l_r) + F_downforce_per_axle + Fz_dyn_loadtransfer

    Fy_f = -Fz_f * D_tire_lat * sin(C_tire_lat * atan(B_tire_lat * alpha_f))
    Fy_r = -Fz_r * D_tire_lat * sin(C_tire_lat * atan(B_tire_lat * alpha_r))

    # normalize forces for tire ellipse constraints
    axt_f = Fx_f / m
    ayt_f = Fy_f / m
    axt_r = Fx_r / m
    ayt_r = Fy_r / m

    h_expr = vertcat(
        # Front Right Vehicle Corner cannot leave track
        trackbound_min - n - l_to_front * sin(mu) + car_width / 2.0 * cos(mu) - s_tb_fr,
        # Front Left Vehicle Corner cannot leave track
        -trackbound_max
        + n
        + l_to_front * sin(mu)
        + car_width / 2.0 * cos(mu)
        - s_tb_fl,
        # Tire ellipse constraints front and rear
        (axt_f / axt_abs_max) ** 2 + (ayt_f / ayt_abs_max) ** 2 - 1 - s_te_f,
        (axt_r / axt_abs_max) ** 2 + (ayt_r / ayt_abs_max) ** 2 - 1 - s_te_r,
        # Maximum velocity constraint
        vx - vx_max - s_vx,  # vx <= vx_max
        # Max. absolute steering angle
        -dels_abs_max + dels,
        -dels_abs_max - dels,
        # Rate Constraints for motor torque and steering angle
        -dax_max + dax,
        -ddels_abs_max + ddels,
        dax_min - dax,
        -ddels_abs_max - ddels,
    )

    return h_expr


### ================== Inequalities constraints ================== ###
def ineqN(z, p, idx_params, model_params):
    n = z[0]
    mu = z[1]
    vx = z[2]
    vy = z[3]
    dpsi = z[4]
    ax = z[5]
    dels = z[6]

    s_tb_fr = z[9]
    s_tb_fl = z[10]
    s_te_f = z[11]
    s_te_r = z[12]
    s_vx = z[13]

    rho = model_params["rho"]
    a_front = model_params["A_front"]
    g = model_params["g"]

    l_f = p[idx_params["l_f"]]
    l_r = p[idx_params["l_r"]]
    h_cg = p[idx_params["h_cg"]]
    car_width = p[idx_params["car_width"]]
    l_to_front = p[idx_params["l_to_front"]]
    l_to_rear = p[idx_params["l_to_rear"]]

    C_r = p[idx_params["C_r"]]
    C_d = p[idx_params["C_d"]]
    C_l = p[idx_params["C_l"]]
    B_tire_lat = p[idx_params["B_tire_lat"]]
    C_tire_lat = p[idx_params["C_tire_lat"]]
    D_tire_lat = p[idx_params["D_tire_lat"]]
    m = p[idx_params["m"]]
    axt_abs_max = p[idx_params["axt_abs_max"]]
    ayt_abs_max = p[idx_params["ayt_abs_max"]]

    trackbound_min = p[idx_params["trackbound_min"]]
    trackbound_max = p[idx_params["trackbound_max"]]

    vx_max_terminal = p[idx_params["vx_max_terminal"]]

    dels_abs_max = p[idx_params["dels_abs_max"]]

    # Resistance Forces
    F_rollres = m * g * C_r
    F_drag = 0.5 * rho * a_front * C_d * (vx**2)
    F_res = F_rollres + F_drag

    # Slip Angles
    vx_n = fmax(1.0, vx)
    alpha_f = atan2(vy + l_f * dpsi, vx_n) - dels
    alpha_r = atan2(vy - l_r * dpsi, vx_n)

    # Longitudinal Tire Forces
    Fx_f = m * ax / 2.0
    Fx_r = m * ax / 2.0

    # Approximate load transfer due to acceleration
    ax_v_est = (Fx_f + Fx_r - F_res) / m
    Fz_dyn_loadtransfer = m * ax_v_est * h_cg / (l_f + l_r)

    # Lateral Tire Forces
    F_downforce_per_axle = 0.5 * rho * C_l * a_front * (vx**2) / 2.0
    Fz_f = m * g * l_r / (l_f + l_r) + F_downforce_per_axle - Fz_dyn_loadtransfer
    Fz_r = m * g * l_f / (l_f + l_r) + F_downforce_per_axle + Fz_dyn_loadtransfer

    Fy_f = -Fz_f * D_tire_lat * sin(C_tire_lat * atan(B_tire_lat * alpha_f))
    Fy_r = -Fz_r * D_tire_lat * sin(C_tire_lat * atan(B_tire_lat * alpha_r))

    # normalize forces for tire ellipse constraints
    axt_f = Fx_f / m
    ayt_f = Fy_f / m
    axt_r = Fx_r / m
    ayt_r = Fy_r / m

    h_expr_terminal = vertcat(
        # Front Right Vehicle Corner cannot leave track
        trackbound_min - n - l_to_front * sin(mu) + car_width / 2.0 * cos(mu) - s_tb_fr,
        # Front Left Vehicle Corner cannot leave track
        -trackbound_max
        + n
        + l_to_front * sin(mu)
        + car_width / 2.0 * cos(mu)
        - s_tb_fl,
        # Tire ellipse constraints front and rear
        (axt_f / axt_abs_max) ** 2 + (ayt_f / ayt_abs_max) ** 2 - 1 - s_te_f,
        (axt_r / axt_abs_max) ** 2 + (ayt_r / ayt_abs_max) ** 2 - 1 - s_te_r,
        # Maximum velocity constraint
        vx - vx_max_terminal - s_vx,  # vx <= vx_max
        # Max. absolute steering angle
        -dels_abs_max + dels,
        -dels_abs_max - dels,
    )

    return h_expr_terminal


### ================== Cost function ================== ###
def obj(z, p, idx_params, slack_weight_dict):

    q_ds = p[idx_params["q_ds"]]
    q_n = p[idx_params["q_n"]]
    q_mu = p[idx_params["q_mu"]]
    r_dax = p[idx_params["r_dax"]]
    r_ddels = p[idx_params["r_ddels"]]

    k = p[idx_params["kappa_ref"]]

    n = z[0]
    mu = z[1]
    vx = z[2]
    vy = z[3]
    ax = z[5]
    dels = z[6]

    dax = z[7]
    ddels = z[8]

    s_tb_fr = z[9]
    s_tb_fl = z[10]
    s_te_f = z[11]
    s_te_r = z[12]
    s_vx = z[13]

    wl_tb = slack_weight_dict["lin"]["S_tb"]
    wq_tb = slack_weight_dict["quad"]["S_tb"]
    wl_te = slack_weight_dict["lin"]["S_te"]
    wq_te = slack_weight_dict["quad"]["S_te"]
    wl_vx = slack_weight_dict["lin"]["S_vx"]
    wq_vx = slack_weight_dict["quad"]["S_vx"]

    # Progress cost
    s_dot = (vx * cos(mu) - vy * sin(mu)) / (1.0 - n * k)
    cost_progress = -s_dot * q_ds

    # Input cost
    cost_input = dax**2 * r_dax + ddels**2 * r_ddels

    # State cost
    cost_state = n**2 * q_n + mu**2 * q_mu

    # Slack cost
    cost_slack = wl_tb * s_tb_fl + wq_tb * s_tb_fl**2
    cost_slack += wl_tb * s_tb_fr + wq_tb * s_tb_fr**2
    cost_slack += wl_te * s_te_f + wq_te * s_te_f**2
    cost_slack += wl_te * s_te_r + wq_te * s_te_r**2
    cost_slack += wl_vx * s_vx + wq_vx * s_vx**2

    cost = cost_progress + cost_state + cost_input + cost_slack

    return cost


### ================== Cost function ================== ###
def objN(z, p, idx_params, slack_weight_dict):

    q_ds = p[idx_params["q_ds_terminal"]]
    q_n = p[idx_params["q_n_terminal"]]
    q_mu = p[idx_params["q_mu_terminal"]]

    k = p[idx_params["kappa_ref"]]

    n = z[0]
    mu = z[1]
    vx = z[2]
    vy = z[3]
    ax = z[5]
    dels = z[6]

    s_tb_fr = z[9]
    s_tb_fl = z[10]
    s_te_f = z[11]
    s_te_r = z[12]
    s_vx = z[13]

    wl_tb = slack_weight_dict["lin"]["S_tb"]
    wq_tb = slack_weight_dict["quad"]["S_tb"]
    wl_te = slack_weight_dict["lin"]["S_te"]
    wq_te = slack_weight_dict["quad"]["S_te"]
    wl_vx = slack_weight_dict["lin"]["S_vx"]
    wq_vx = slack_weight_dict["quad"]["S_vx"]

    # Progress cost
    s_dot = (vx * cos(mu) - vy * sin(mu)) / (1.0 - n * k)
    cost_progress = -s_dot * q_ds

    # State cost
    cost_state = n**2 * q_n + mu**2 * q_mu

    # Slack cost
    cost_slack = wl_tb * s_tb_fl + wq_tb * s_tb_fl**2
    cost_slack += wl_tb * s_tb_fr + wq_tb * s_tb_fr**2
    cost_slack += wl_te * s_te_f + wq_te * s_te_f**2
    cost_slack += wl_te * s_te_r + wq_te * s_te_r**2
    cost_slack += wl_vx * s_vx + wq_vx * s_vx**2

    cost = cost_progress + cost_state + cost_slack

    return cost
