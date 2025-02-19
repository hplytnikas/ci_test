from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from acados_template.builders import ocp_get_default_cmake_builder
from models import export_mpc_ode_model
from load_mpc_parameters import load_mpc_yaml_params
import numpy as np
from os.path import dirname, join, abspath
import pprint
import shutil
import os


def setup_ocp(x0, simulate_ocp: bool = False):
    """
    Set up the OCP ( Optimal Control Problem ) for the vehicle model.
    The model is defined in the `models.py` file.
    The OCP is defined in the `acados_template` format.
    The OCP is solved using the `AcadosOcpSolver` class.
    The OCP solver is created and returned.

    Args:
        x0 (np.array): Initial state vector.
        simulate_ocp (bool, optional): If True, the OCP is simulated. Defaults to False.
    """
    (
        ocp,
        ocp_solver_json_path,
        constraint_params,
        cost_params,
        model_params,
        acados_solver_params,
        common_solver_params,
    ) = prepare_ocp()

    """ ========== MODEL ============ """
    mpc_horizon_parameters = {}
    # final time for prediction horizon
    mpc_horizon_parameters["T_final"] = common_solver_params["T_final"]
    # number of steps along horizon
    mpc_horizon_parameters["N_horizon"] = common_solver_params["N_horizon"]

    # external cost is defined inside the model
    # Get AcadosModel form other python file
    model = export_mpc_ode_model(model_params)
    ocp.model = model

    """ ========== DIMENSIONS ============ """
    # Dimensions
    # x: state
    # u: input
    # N: prediction horizon number of stages
    ocp.dims.N = mpc_horizon_parameters["N_horizon"]

    """ ========= CONSTRAINT: INITIAL STATE =========== """
    ocp.constraints.x0 = x0

    set_initial_constraints(ocp)
    set_stage_constraints(ocp)
    set_terminal_constraints(ocp)

    """ ========= COST =========== """
    # (model cost inside ocp.model) --> cost type external
    ocp.cost.cost_type = "EXTERNAL"

    set_slack_cost(ocp, cost_params)

    """ ============ SOLVER OPTIONS ================== """
    set_solver_options(ocp, acados_solver_params, common_solver_params)

    """ ============ INITIAL PARAMETER VALUES ================== """
    m = model_params["m"]  # mass
    l_f = model_params["l_f"]
    l_r = model_params["l_r"]
    C_d = model_params["C_d"]  # effective drag coefficient
    C_r = model_params["C_r"]  # const. rolling resistance
    kappa_ref = model_params["kappa_ref"]  # reference curvature
    q_ds = cost_params["q_ds"]
    q_n = cost_params["q_n"]
    q_mu = cost_params["q_mu"]
    r_dax = cost_params["r_dax"]
    r_ddels = cost_params["r_ddels"]
    q_ds_terminal = cost_params["q_ds_terminal"]
    q_n_terminal = cost_params["q_n_terminal"]
    q_mu_terminal = cost_params["q_mu_terminal"]
    vx_max = constraint_params["vx_max"]
    vx_max_terminal = constraint_params["vx_max_terminal"]
    axt_abs_max = constraint_params["axt_abs_max"]
    ayt_abs_max = constraint_params["ayt_abs_max"]
    dels_abs_max = constraint_params["dels_abs_max"]
    dax_min = constraint_params["dax_min"]
    dax_max = constraint_params["dax_max"]
    ddels_abs_max = constraint_params["ddels_abs_max"]
    car_width = model_params["car_width"]
    trackbound_min = constraint_params["trackbound_min"]
    trackbound_max = constraint_params["trackbound_max"]
    B_tire_lat = model_params["B_tire"]
    C_tire_lat = model_params["C_tire"]
    D_tire_lat = model_params["D_tire"]
    h_cg = model_params["h_cg"]
    Iz = model_params["Iz"]
    C_l = model_params["C_l"]
    l_to_rear = model_params["l_to_rear"]
    l_to_front = model_params["l_to_front"]

    paramvec = np.array(
        (
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
    )
    ocp.parameter_values = paramvec

    """ ====== CREATE OCP AND SIM SOLVERS =========== """
    if not simulate_ocp:
        cmake_builder = ocp_get_default_cmake_builder()
    else:
        cmake_builder = None
    acados_ocp_solver = AcadosOcpSolver(
        ocp, json_file=ocp_solver_json_path, cmake_builder=cmake_builder
    )

    # create an integrator with the same settings as used in the OCP solver.
    if simulate_ocp:
        acados_integrator = AcadosSimSolver(
            ocp, json_file=ocp_solver_json_path, cmake_builder=cmake_builder
        )
    else:
        acados_integrator = None

    return acados_ocp_solver, acados_integrator


def set_initial_constraints(ocp):
    """
    Set the initial constraints for the optimal control problem (OCP).

    Args:
      ocp (AcadosOcp): The optimal control problem.

    """

    """ ========= CONSTRAINTS: h(x, u, p) <= 0 ======== """
    # constraints generally as nonlinear functions
    n_h = 4  # number of constraints
    ocp.constraints.uh_0 = np.zeros((n_h,))
    ocp.constraints.lh_0 = -1000.0 * np.ones((n_h,))

    ocp.constraints.lbu_0 = np.array(
        [
            -1000.0,  # dax
            -1000.0,  # ddels
        ]
    )
    ocp.constraints.ubu_0 = np.array(
        [
            1000.0,  # dax
            1000.0,  # ddels
        ]
    )
    ocp.constraints.idxbu_0 = np.array([0, 1])


def set_stage_constraints(ocp):
    """
    Set the stage constraints for the optimal control problem (OCP).

    Args:
      ocp (AcadosOcp): The optimal control problem.

    """

    """ ========= CONSTRAINTS: h(x, u, p) <= 0 ======== """
    # constraints generally as nonlinear functions
    n_h = 11  # number of constraints
    ocp.constraints.uh = np.zeros((n_h,))
    ocp.constraints.lh = -1000.0 * np.ones((n_h,))

    ocp.constraints.lbx = np.array(
        [
            -1.5,  # n
            -1.5,  # mu
            -0.1,  # vx
            -5.0,  # vy
            -5.0,  # r
            -100.0,  # ax
            -1.0,  # dels
        ]
    )
    ocp.constraints.ubx = np.array(
        [
            1.5,  # n
            1.5,  # mu
            25.0,  # vx
            5.0,  # vy
            5.0,  # r
            100.0,  # ax
            1.0,  # dels
        ]
    )
    ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4, 5, 6])

    ocp.constraints.lbu = np.array(
        [
            -1000.0,  # dax
            -1000.0,  # ddels
        ]
    )
    ocp.constraints.ubu = np.array(
        [
            1000.0,  # dax
            1000.0,  # ddels
        ]
    )
    ocp.constraints.idxbu = np.array([0, 1])


def set_terminal_constraints(ocp):
    """
    Set the terminal constraints for the optimal control problem (OCP).

    Args:
      ocp (AcadosOcp): The optimal control problem.

    """

    """ ========= CONSTRAINTS: h(x, u, p) <= 0 ======== """
    # constraints generally as nonlinear functions
    n_h = 7  # number of constraints
    ocp.constraints.uh_e = np.zeros((n_h,))
    ocp.constraints.lh_e = -1000.0 * np.ones((n_h,))

    ocp.constraints.lbx_e = np.array(
        [
            -1.5,  # n
            -1.5,  # mu
            -0.1,  # vx
            -5.0,  # vy
            -5.0,  # r
            -100.0,  # ax
            -1.0,  # dels
        ]
    )
    ocp.constraints.ubx_e = np.array(
        [
            1.5,  # n
            1.5,  # mu
            20.0,  # vx
            5.0,  # vy
            5.0,  # r
            100.0,  # ax
            1.0,  # dels
        ]
    )
    ocp.constraints.idxbx_e = np.array([0, 1, 2, 3, 4, 5, 6])


def set_slack_cost(ocp, cost_parameters):
    """
    Set the slack cost for the optimal control problem (OCP).

    Args:
      ocp (AcadosOcp): The optimal control problem.
      cost_parameters (dict): Dictionary containing the cost parameters.

    """

    """ ========== STAGE SLACK COST =========== """
    # Slack Weights

    slack_dict = cost_parameters["slack_penalties"]

    # Quadratic Slack Cost weights
    ocp.cost.Zu = np.diag(
        np.array(
            [
                slack_dict["quad"]["S_tb"],
                slack_dict["quad"]["S_tb"],
                slack_dict["quad"]["S_te"],
                slack_dict["quad"]["S_te"],
                slack_dict["quad"]["S_vx"],
            ]
        )
    )
    ocp.cost.Zl = np.diag(
        np.array(
            [
                slack_dict["quad"]["S_tb"],
                slack_dict["quad"]["S_tb"],
                slack_dict["quad"]["S_te"],
                slack_dict["quad"]["S_te"],
                slack_dict["quad"]["S_vx"],
            ]
        )
    )
    # Linear Slack Cost Weights
    ocp.cost.zu = np.array(
        [
            slack_dict["lin"]["S_tb"],
            slack_dict["lin"]["S_tb"],
            slack_dict["lin"]["S_te"],
            slack_dict["lin"]["S_te"],
            slack_dict["lin"]["S_vx"],
        ]
    )
    ocp.cost.zl = np.array(
        [
            slack_dict["lin"]["S_tb"],
            slack_dict["lin"]["S_tb"],
            slack_dict["lin"]["S_te"],
            slack_dict["lin"]["S_te"],
            slack_dict["lin"]["S_vx"],
        ]
    )

    # Indices of slack variables
    ocp.constraints.idxsh = np.array(
        (
            0,
            1,
            2,
            3,
            4,
        )
    )

    """ ======== TERMINAL SLACK COST ========== """
    # Quadratic Slack Cost weights
    ocp.cost.Zu_e = np.diag(
        np.array(
            [
                slack_dict["quad"]["S_tb"],
                slack_dict["quad"]["S_tb"],
                slack_dict["quad"]["S_te"],
                slack_dict["quad"]["S_te"],
                slack_dict["quad"]["S_vx"],
            ]
        )
    )
    ocp.cost.Zl_e = np.diag(
        np.array(
            [
                slack_dict["quad"]["S_tb"],
                slack_dict["quad"]["S_tb"],
                slack_dict["quad"]["S_te"],
                slack_dict["quad"]["S_te"],
                slack_dict["quad"]["S_vx"],
            ]
        )
    )
    # Linear Slack Cost Weights
    ocp.cost.zu_e = np.array(
        [
            slack_dict["lin"]["S_tb"],
            slack_dict["lin"]["S_tb"],
            slack_dict["lin"]["S_te"],
            slack_dict["lin"]["S_te"],
            slack_dict["lin"]["S_vx"],
        ]
    )
    ocp.cost.zl_e = np.array(
        [
            slack_dict["lin"]["S_tb"],
            slack_dict["lin"]["S_tb"],
            slack_dict["lin"]["S_te"],
            slack_dict["lin"]["S_te"],
            slack_dict["lin"]["S_vx"],
        ]
    )

    # Indices of slack variables
    ocp.constraints.idxsh_e = np.array(
        (
            0,
            1,
            2,
            3,
            4,
        )
    )


def prepare_ocp():
    """
    Prepare the optimal control problem (OCP) for the vehicle model by getting parameters, setting up paths, etc.

    Returns:
        AcadosOcp: The optimal control problem.
        str: The path to the OCP solver JSON file.
        dict: Dictionary containing the constraint parameters.
        dict: Dictionary containing the cost parameters.
        dict: Dictionary containing the model parameters.
        dict: Dictionary containing the solver parameters.

    """
    """ Load .yaml file parameters """
    (
        constraint_params,
        cost_params,
        model_params,
        acados_solver_params,
        common_solver_params,
    ) = load_mpc_yaml_params()
    pprint.pprint(constraint_params)
    pprint.pprint(cost_params)
    pprint.pprint(model_params)
    pprint.pprint(acados_solver_params)
    pprint.pprint(common_solver_params)

    """ ========== OCP Setup ============ """
    # Paths
    acados_path = join(
        dirname(abspath(__file__)), "../../", acados_solver_params["acados_path"]
    )
    codegen_export_dir = join(
        dirname(abspath(__file__)), "../../", acados_solver_params["codegen_export_dir"]
    )
    ocp_solver_json_path = join(
        dirname(abspath(__file__)),
        "../../",
        acados_solver_params["ocp_solver_json_path"],
        "acados_ocp.json",
    )

    # Remove codegen folder
    print("Trying to remove codegen folder...")

    try:
        shutil.rmtree(codegen_export_dir)
        print("Codegen folder removed.")
    except FileNotFoundError:
        print("Codegen folder not found and thus not removed.")

    print("Codegen directory: ", codegen_export_dir)

    # Remove codegen .json
    print("Trying to remove codegen .json...")

    try:
        os.remove(ocp_solver_json_path)
        print("Codegen .json removed.")
    except FileNotFoundError:
        print("Codegen .json not found and thus not removed.")

    print("Solver .json directory: ", ocp_solver_json_path)

    # Set up optimal control problem
    ocp = AcadosOcp()

    # Set code export directory
    ocp.code_export_directory = codegen_export_dir

    # set header paths
    ocp.acados_include_path = f"{acados_path}/include"
    ocp.acados_lib_path = f"{acados_path}/lib"

    return (
        ocp,
        ocp_solver_json_path,
        constraint_params,
        cost_params,
        model_params,
        acados_solver_params,
        common_solver_params,
    )


def set_solver_options(ocp, acados_solver_params, common_solver_params):
    """
    Set the solver options for the optimal control problem (OCP).

    Args:
      ocp (AcadosOcp): The optimal control problem.
      acados_solver_params (dict): Dictionary containing the ACADOS-specific solver parameters.
      common_solver_params (dict): Dictionary containing the common solver parameters.
    """

    """============ SOLVER OPTIONS =================="""
    ocp.solver_options.qp_solver = acados_solver_params["qp_solver"]
    ocp.solver_options.qp_tol = acados_solver_params["qp_tol"]
    ocp.solver_options.qp_solver_iter_max = acados_solver_params["qp_solver_iter_max"]
    ocp.solver_options.qp_solver_warm_start = acados_solver_params[
        "qp_solver_warm_start"
    ]
    ocp.solver_options.hpipm_mode = acados_solver_params["hpipm_mode"]
    ocp.solver_options.qp_solver_ric_alg = acados_solver_params["qp_solver_ric_alg"]

    ocp.solver_options.hessian_approx = acados_solver_params["hessian_approx"]
    ocp.solver_options.regularize_method = acados_solver_params["regularize_method"]
    ocp.solver_options.reg_epsilon = acados_solver_params["reg_epsilon"]
    ocp.solver_options.levenberg_marquardt = acados_solver_params["levenberg_marquardt"]

    ocp.solver_options.integrator_type = acados_solver_params["integrator_type"]
    ocp.solver_options.sim_method_newton_iter = acados_solver_params[
        "sim_method_newton_iter"
    ]
    ocp.solver_options.sim_method_num_steps = acados_solver_params[
        "sim_method_num_steps"
    ]

    ocp.solver_options.globalization = acados_solver_params["globalization"]
    ocp.solver_options.alpha_min = acados_solver_params["alpha_min"]
    ocp.solver_options.alpha_reduction = acados_solver_params["alpha_reduction"]

    ocp.solver_options.tol = acados_solver_params["tol"]
    ocp.solver_options.nlp_solver_step_length = acados_solver_params[
        "nlp_solver_step_length"
    ]
    ocp.solver_options.nlp_solver_type = acados_solver_params["nlp_solver_type"]
    ocp.solver_options.nlp_solver_max_iter = acados_solver_params["nlp_solver_max_iter"]
    ocp.solver_options.cost_discretization = acados_solver_params["cost_discretization"]

    ocp.solver_options.line_search_use_sufficient_descent = acados_solver_params[
        "line_search_use_sufficient_descent"
    ]
    ocp.solver_options.eps_sufficient_descent = acados_solver_params[
        "eps_sufficient_descent"
    ]

    # Set prediction horizon in time
    ocp.solver_options.tf = common_solver_params["T_final"]
