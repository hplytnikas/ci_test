from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, atan2, atan, fmax, tan, fabs, log, exp


BETA_SMOOTHMAX = 15.0
VX_MIN_SMOOTHMAX = 1.0


def get_cost(n, mu, vx, vy, dax, ddel_s, q_ds, q_n, q_mu, kappa_ref, r_dax, r_ddels):
    """STAGE Cost (model-based, slack is defined on the solver)"""

    cost_sd = -q_ds * (vx * cos(mu) - vy * sin(mu)) / ((1 - n * kappa_ref))
    cost_n = q_n * n**2
    cost_mu = q_mu * mu**2

    cost_dax = r_dax * dax**2
    cost_ddel_s = r_ddels * ddel_s**2

    stage_cost = cost_sd + cost_n + cost_dax + cost_mu + cost_ddel_s

    return stage_cost


def get_terminal_cost(
    n, mu, vx, vy, q_ds_terminal, q_n_terminal, q_mu_terminal, kappa_ref
):
    """TERMINAL Cost (model-based, slack is defined on the solver)"""

    cost_sd = -q_ds_terminal * (vx * cos(mu) - vy * sin(mu)) / ((1 - n * kappa_ref))
    cost_n = q_n_terminal * n**2
    cost_mu = q_mu_terminal * mu**2

    stage_cost = cost_sd + cost_n + cost_mu

    return stage_cost


def get_initial_constraint(dax, ddel_s, dax_min, dax_max, ddels_abs_max):
    # CONSTRAINTS
    # Define initial constraints all as h(x, u, p) <= 0

    h_expr = vertcat(
        # Rate Constraints for motor torque and steering angle
        -dax_max + dax,
        -ddels_abs_max + ddel_s,
        dax_min - dax,
        -ddels_abs_max - ddel_s,
    )

    return h_expr


def get_constraint(
    n,
    mu,
    vx,
    vy,
    dpsi,
    ax,
    del_s,
    dax,
    ddel_s,
    l_r,
    l_f,
    l_to_rear,
    l_to_front,
    rho,
    C_r,
    C_d,
    C_l,
    a_front,
    g,
    h_cg,
    m,
    B_tire_lat,
    C_tire_lat,
    D_tire_lat,
    vx_max,
    car_width,
    trackbound_min,
    trackbound_max,
    axt_abs_max,
    ayt_abs_max,
    dels_abs_max,
    dax_min,
    dax_max,
    ddels_abs_max,
):
    # CONSTRAINTS
    # Define stage constraints all as h(x, u, p) <= 0

    # Resistance Forces
    F_rollres = m * g * C_r
    F_drag = 0.5 * rho * a_front * C_d * (vx**2)
    F_res = F_rollres + F_drag

    # Slip Angles
    vx_n = (
        VX_MIN_SMOOTHMAX
        + log(exp(BETA_SMOOTHMAX * (vx - VX_MIN_SMOOTHMAX)) + VX_MIN_SMOOTHMAX)
        / BETA_SMOOTHMAX
    )
    alpha_f = atan2(vy + l_f * dpsi, vx_n) - del_s
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
        trackbound_min - n - l_to_front * sin(mu) + car_width / 2.0 * cos(mu),
        # Front Left Vehicle Corner cannot leave track
        -trackbound_max + n + l_to_front * sin(mu) + car_width / 2.0 * cos(mu),
        # Tire ellipse constraints front and rear
        (axt_f / axt_abs_max) ** 2 + (ayt_f / ayt_abs_max) ** 2 - 1,
        (axt_r / axt_abs_max) ** 2 + (ayt_r / ayt_abs_max) ** 2 - 1,
        # Maximum velocity constraint
        vx - vx_max,  # vx <= vx_max
        # Max. absolute steering angle
        -dels_abs_max + del_s,
        -dels_abs_max - del_s,
        # Rate Constraints for motor torque and steering angle
        -dax_max + dax,
        -ddels_abs_max + ddel_s,
        dax_min - dax,
        -ddels_abs_max - ddel_s,
    )

    return h_expr


def get_terminal_constraint(
    n,
    mu,
    vx,
    vy,
    dpsi,
    ax,
    del_s,
    l_r,
    l_f,
    l_to_rear,
    l_to_front,
    rho,
    C_r,
    C_d,
    C_l,
    a_front,
    g,
    h_cg,
    m,
    B_tire_lat,
    C_tire_lat,
    D_tire_lat,
    vx_max_terminal,
    car_width,
    trackbound_min,
    trackbound_max,
    axt_abs_max,
    ayt_abs_max,
    dels_abs_max,
):
    # CONSTRAINTS
    # Define stage constraints all as h(x, u, p) <= 0

    # Resistance Forces
    F_rollres = m * g * C_r
    F_drag = 0.5 * rho * a_front * C_d * (vx**2)
    F_res = F_rollres + F_drag

    # Slip Angles
    vx_n = (
        VX_MIN_SMOOTHMAX
        + log(exp(BETA_SMOOTHMAX * (vx - VX_MIN_SMOOTHMAX)) + VX_MIN_SMOOTHMAX)
        / BETA_SMOOTHMAX
    )
    alpha_f = atan2(vy + l_f * dpsi, vx_n) - del_s
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
        trackbound_min - n - l_to_front * sin(mu) + car_width / 2.0 * cos(mu),
        # Front Left Vehicle Corner cannot leave track
        -trackbound_max + n + l_to_front * sin(mu) + car_width / 2.0 * cos(mu),
        # Tire ellipse constraints front and rear
        (axt_f / axt_abs_max) ** 2 + (ayt_f / ayt_abs_max) ** 2 - 1,
        (axt_r / axt_abs_max) ** 2 + (ayt_r / ayt_abs_max) ** 2 - 1,
        # Maximum velocity constraint
        vx - vx_max_terminal,  # vx <= vx_max
        # Max. absolute steering angle
        -dels_abs_max + del_s,
        -dels_abs_max - del_s,
    )

    return h_expr_terminal


def export_mpc_ode_model(model_params) -> AcadosModel:
    """
    Create/Export ODE model for the acados solver.

    Returns:
      AcadosModel: The exported ode model.
    """

    # model name
    model_name = "veh_dynamics_ode"

    # ============== Symbolics in CasADi ===========
    # Symbolic Parameters
    m = SX.sym("m")
    Iz = SX.sym("Iz")
    h_cg = SX.sym("h_cg")
    l_f = SX.sym("l_f")
    l_r = SX.sym("l_r")
    l_to_rear = SX.sym("l_to_rear")
    l_to_front = SX.sym("l_to_front")
    car_width = SX.sym("car_width")
    C_r = SX.sym("C_r")
    C_d = SX.sym("C_d")
    C_l = SX.sym("C_l")
    B_tire_lat = SX.sym("B_tire_lat")
    C_tire_lat = SX.sym("C_tire_lat")
    D_tire_lat = SX.sym("D_tire_lat")
    kappa_ref = SX.sym("kappa_ref")
    q_ds = SX.sym("q_ds")
    q_n = SX.sym("q_n")
    q_mu = SX.sym("q_mu")
    r_dax = SX.sym("r_dax")
    r_ddels = SX.sym("r_ddels")
    q_ds_terminal = SX.sym("q_ds_terminal")
    q_n_terminal = SX.sym("q_n_terminal")
    q_mu_terminal = SX.sym("q_mu_terminal")
    vx_max = SX.sym("vx_max")
    vx_max_terminal = SX.sym("vx_max_terminal")
    axt_abs_max = SX.sym("axt_abs_max")
    ayt_abs_max = SX.sym("ayt_abs_max")
    dels_abs_max = SX.sym("dels_abs_max")
    dax_min = SX.sym("dax_min")
    dax_max = SX.sym("dax_max")
    ddels_abs_max = SX.sym("ddels_abs_max")
    trackbound_min = SX.sym("trackbound_min")
    trackbound_max = SX.sym("trackbound_max")

    # Runtime parameter vector --> ORDER MATTERS
    p = vertcat(
        m,
        Iz,
        h_cg,
        l_f,
        l_r,
        l_to_rear,
        l_to_front,
        car_width,
        C_r,
        C_d,
        C_l,
        B_tire_lat,
        C_tire_lat,
        D_tire_lat,
        kappa_ref,
        q_ds,
        q_n,
        q_mu,
        r_dax,
        r_ddels,
        q_ds_terminal,
        q_n_terminal,
        q_mu_terminal,
        vx_max,
        vx_max_terminal,
        axt_abs_max,
        ayt_abs_max,
        dels_abs_max,
        dax_min,
        dax_max,
        ddels_abs_max,
        trackbound_min,
        trackbound_max,
    )

    # Non-runtime parameters
    rho = model_params["rho"]
    a_front = model_params["A_front"]
    g = model_params["g"]

    # Symbolic States
    n = SX.sym("n")
    mu = SX.sym("mu")
    vx = SX.sym("vx")
    vy = SX.sym("vy")
    dpsi = SX.sym("dpsi")
    ax = SX.sym("ax")
    del_s = SX.sym("del_s")
    x = vertcat(n, mu, vx, vy, dpsi, ax, del_s)

    # Symbolic Inputs
    dax = SX.sym("dax")
    ddel_s = SX.sym("ddel_s")
    u = vertcat(dax, ddel_s)

    # Symbolic State Derivative f(x,u)
    n_dot = SX.sym("n_dot")
    mu_dot = SX.sym("mu_dot")
    vx_dot = SX.sym("vx_dot")
    vy_dot = SX.sym("vy_dot")
    dpsi_dot = SX.sym("dpsi_dot")
    ax_dot = SX.sym("ax_dot")
    del_s_dot = SX.sym("del_s_dot")
    xdot = vertcat(n_dot, mu_dot, vx_dot, vy_dot, dpsi_dot, ax_dot, del_s_dot)

    # =============== dynamics ==================

    # Resistance Forces
    F_rollres = m * g * C_r
    F_drag = 0.5 * rho * a_front * C_d * (vx**2)
    F_res = F_rollres + F_drag

    # Slip Angles
    vx_n = (
        VX_MIN_SMOOTHMAX
        + log(exp(BETA_SMOOTHMAX * (vx - VX_MIN_SMOOTHMAX)) + VX_MIN_SMOOTHMAX)
        / BETA_SMOOTHMAX
    )
    alpha_f = atan2(vy + l_f * dpsi, vx_n) - del_s
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

    # accelerations in vehicle frame
    ax_v = (Fx_f * cos(del_s) + Fx_r - Fy_f * sin(del_s) - F_res) / m
    ay_v = (Fx_f * sin(del_s) + Fy_f * cos(del_s) + Fy_r) / m

    # derivative of state w.r.t time
    n_dot_expl_dyn = vx * sin(mu) + vy * cos(mu)
    mu_dot_expl_dyn = dpsi - kappa_ref * (vx * cos(mu) - vy * sin(mu)) / (
        (1 - n * kappa_ref)
    )
    vx_dot_expl_dyn = ax_v + vy * dpsi
    vy_dot_expl_dyn = ay_v - vx * dpsi
    dpsi_dot_expl_dyn = (
        Fx_f * sin(del_s) * l_f + Fy_f * cos(del_s) * l_f - Fy_r * l_r
    ) / Iz
    ax_dot_expl_dyn = dax
    del_s_dot_expl_dyn = ddel_s

    # Explicit expression
    f_expl_time = vertcat(
        n_dot_expl_dyn,
        mu_dot_expl_dyn,
        vx_dot_expl_dyn,
        vy_dot_expl_dyn,
        dpsi_dot_expl_dyn,
        ax_dot_expl_dyn,
        del_s_dot_expl_dyn,
    )

    # Implicit expression
    f_impl_time = xdot - f_expl_time  # = 0

    """ Cost """

    stage_cost = get_cost(
        n, mu, vx, vy, dax, ddel_s, q_ds, q_n, q_mu, kappa_ref, r_dax, r_ddels
    )

    terminal_cost = get_terminal_cost(
        n, mu, vx, vy, q_ds_terminal, q_n_terminal, q_mu_terminal, kappa_ref
    )

    """ Constraints """

    initial_constraints = get_initial_constraint(
        dax, ddel_s, dax_min, dax_max, ddels_abs_max
    )

    stage_constraints = get_constraint(
        n,
        mu,
        vx,
        vy,
        dpsi,
        ax,
        del_s,
        dax,
        ddel_s,
        l_r,
        l_f,
        l_to_rear,
        l_to_front,
        rho,
        C_r,
        C_d,
        C_l,
        a_front,
        g,
        h_cg,
        m,
        B_tire_lat,
        C_tire_lat,
        D_tire_lat,
        vx_max,
        car_width,
        trackbound_min,
        trackbound_max,
        axt_abs_max,
        ayt_abs_max,
        dels_abs_max,
        dax_min,
        dax_max,
        ddels_abs_max,
    )

    terminal_constraints = get_terminal_constraint(
        n,
        mu,
        vx,
        vy,
        dpsi,
        ax,
        del_s,
        l_r,
        l_f,
        l_to_rear,
        l_to_front,
        rho,
        C_r,
        C_d,
        C_l,
        a_front,
        g,
        h_cg,
        m,
        B_tire_lat,
        C_tire_lat,
        D_tire_lat,
        vx_max_terminal,
        car_width,
        trackbound_min,
        trackbound_max,
        axt_abs_max,
        ayt_abs_max,
        dels_abs_max,
    )

    # ============= ACADOS ===============
    # Acados Model Creation from CasADi symbolic expressions
    model = AcadosModel()

    model.f_impl_expr = f_impl_time
    model.f_expl_expr = f_expl_time
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p
    model.name = model_name

    model.cost_expr_ext_cost_0 = stage_cost
    model.cost_expr_ext_cost = stage_cost
    model.cost_expr_ext_cost_e = terminal_cost

    model.con_h_expr_0 = initial_constraints
    model.con_h_expr = stage_constraints
    model.con_h_expr_e = terminal_constraints

    return model
