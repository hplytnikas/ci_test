import forcespro.nlp
import numpy as np
import os
import sys
from casadi import inf, pi

from codegen_embotech_functions import (
    readYaml,
    obj,
    objN,
    getCodeoptions,
    getContinuousDynamics,
    ineq,
    ineqN,
)

"""
z = [state | input | slack]

n, mu, vx, vy, dpsi, ax, dels
0, 1,  2,  3,  4,    5,  6,

dax, ddels
7,   8

--- if slack variables are added, the state vector is: ---
S_dax, S_ddels, S_s, S_n, S_mu, S_vx, S_vy, S_dpsi, S_ax, S_del_s
9,    10,     11,   12,   13,   14,   15,    16,    17,    18

                        ========== IMPORTANT ==========

->  Embotech solver returns from x0 (the initial state) to xN-1 (the second to last state). uN-1 is the last input which is always 0.0 . Therefore, we use a horizon length of N_horizon + 1 to actually get the terminal state. But in the controller, it will be seen as N.

-> You can set the dynamics either by "model.continuous_dynamics" or "model.eq". By using the second one, it codegenerates an integrator for the dynamics which can be called externally. The second one seems to be faster, but it's not proved yet.

-> The solver is generated with the following logic: if a linear slack cost is set to 0.0 in the configuration file, this specific ODE is hard constrained. If it's set to a value greater than 0.0, it's soft constrained.

-> Carefully choosing an initialisation value for the BFGS matrix is crucial for the performance of the solver. By default, it is the identity matrix. It appears as a value in the range of 0.05 - 0.2 is a good choice. Of course, it doesn't have to be set to a constant value, it can be a function of the state and input vectors.

-> The option "codeoptions.nlp.integrator.attempt_subsystem_exploitation" is set to False by default. It's a feature that allows the solver to exploit the structure of the problem to speed up the solution process. After simple tests, it does not seems to improve in every case but it's worth to try.

"""


# ========== Generate the solver into the right folder ========== #
def find_autonomous_2024_directory(start_directory):
    current_directory = start_directory
    while current_directory != "/":
        if os.path.basename(current_directory) == "autonomous_2024":
            return current_directory
        # Move up one directory level
        current_directory = os.path.dirname(current_directory)
    return None


def generate_solver():
    # Start from the current working directory
    start_directory = os.getcwd()
    autonomous_2024_path = find_autonomous_2024_directory(start_directory)

    if autonomous_2024_path is None:
        print("Error: 'autonomous_2024' directory not found in any parent directories.")
        sys.exit(1)

    # Define the rest of the path to the specific folder from autonomous_2024
    rest_of_path = (
        "src/control/controllers/mpc/solvers/codegenerated_solvers/embotech_codegen"
    )

    # Construct the full path to the specific folder
    specific_folder = os.path.join(autonomous_2024_path, rest_of_path)

    # Change to the specific folder
    try:
        os.chdir(specific_folder)
        print(f"Successfully changed directory to {specific_folder}")
    except OSError as e:
        print(f"Error: Could not change directory to {specific_folder}. {e}")
        sys.exit(1)

    # ========== Import the parameters from the yaml file ========== #
    (
        constraint_params,
        cost_params,
        model_params,
        embotech_solver_params,
        common_solver_params,
    ) = readYaml()

    # ========== Import the model template ========== #
    model = forcespro.nlp.SymbolicModel()

    # ============== Set parameter indices ============== #
    idx_params = {}
    # Vehicle parameters
    idx_params["m"] = 0
    idx_params["Iz"] = 1
    idx_params["h_cg"] = 2
    idx_params["l_f"] = 3
    idx_params["l_r"] = 4
    idx_params["l_to_rear"] = 5
    idx_params["l_to_front"] = 6
    idx_params["car_width"] = 7
    idx_params["C_r"] = 8
    idx_params["C_d"] = 9
    idx_params["C_l"] = 10
    idx_params["B_tire_lat"] = 11
    idx_params["C_tire_lat"] = 12
    idx_params["D_tire_lat"] = 13

    idx_params["kappa_ref"] = 14

    # Cost function parameters
    idx_params["q_ds"] = 15
    idx_params["q_n"] = 16
    idx_params["q_mu"] = 17
    idx_params["r_dax"] = 18
    idx_params["r_ddels"] = 19
    idx_params["q_ds_terminal"] = 20
    idx_params["q_n_terminal"] = 21
    idx_params["q_mu_terminal"] = 22

    # Constraint parameters
    idx_params["vx_max"] = 23
    idx_params["vx_max_terminal"] = 24
    idx_params["axt_abs_max"] = 25
    idx_params["ayt_abs_max"] = 26
    idx_params["dels_abs_max"] = 27
    idx_params["dax_min"] = 28
    idx_params["dax_max"] = 29
    idx_params["ddels_abs_max"] = 30
    idx_params["trackbound_min"] = 31
    idx_params["trackbound_max"] = 32

    # ========== Import the solver options ========== #
    codeoptions = getCodeoptions()

    # ========== DIMENSIONS ========== #

    N_RUNTIME_PARAM = embotech_solver_params["n_runtime_param"]
    N_STATE = embotech_solver_params["n_states"]
    N_INPUT = embotech_solver_params["n_inputs"]
    N_SLACK = embotech_solver_params["n_slacks"]

    N_VARIABLE = N_INPUT + N_STATE + N_SLACK

    model.nvar = N_VARIABLE

    print(" ")
    print("Number of variables per stage: ", N_VARIABLE)

    model.neq = N_STATE

    # ========== Horizon length ========== #
    N_HORIZON = common_solver_params["N_horizon"] + 1
    model.N = N_HORIZON

    # ========== Number of runtime parameters ========== #
    model.npar = N_RUNTIME_PARAM

    # ========== EQUALITY CONSTRAINTS ========== #
    model.eq = lambda z, p: forcespro.nlp.integrate(
        getContinuousDynamics,
        z[0:N_STATE],  # state vector
        z[N_STATE : N_STATE + N_INPUT],  # input vector
        p,
        idx_params,
        model_params,
        integrator=forcespro.nlp.integrators.RK4,
        stepsize=common_solver_params["T_final"] / (N_HORIZON - 1),
    )

    # ========== E matrix : separate states and inputs of z ========== #
    model.E = np.concatenate(
        [
            np.eye(N_STATE),
            np.zeros((N_STATE, N_INPUT + N_SLACK)),
        ],
        axis=1,
    )

    # initial state
    model.xinitidx = range(0, N_STATE)

    # ========== INEQUALITY CONSTRAINTS ========== #

    # [n, mu, vx, vy, dpsi, ax, dels, dax, ddels]
    # stage-wise bounds which are not representative of
    # actual constriants which are given as
    # runtime parameters
    # (that's why these values are larger than the actual constraints)
    model.lb = [
        -4.0,  # n
        -1.5,  # mu
        -0.1,  # vx
        -5.0,  # vy
        -5.0,  # dpsi
        -100.0,  # ax
        -1.0,  # dels
        -inf,  # dax
        -inf,  # ddels
        0.0,  # slack_tb_fr
        0.0,  # slack_tb_fl
        0.0,  # slack_te_f
        0.0,  # slack_te_r
        0.0,  # slack_vx
    ]

    model.ub = [
        4.0,  # n
        1.5,  # mu
        25.0,  # vx
        5.0,  # vy
        5.0,  # dpsi
        100.0,  # ax
        1.0,  # dels
        inf,  # dax
        inf,  # ddels
        inf,  # slack_tb_fr
        inf,  # slack_tb_fl
        inf,  # slack_te_f
        inf,  # slack_te_r
        inf,  # slack_vx
    ]

    model.lbN = [
        -1.5,
        -1.5,
        -0.1,
        -5.0,
        -5.0,
        -100.0,
        -1.0,
        -inf,
        -inf,
        0.0,  # slack_tb_fr
        0.0,  # slack_tb_fl
        0.0,  # slack_te_f
        0.0,  # slack_te_r
        0.0,  # slack_vx
    ]

    model.ubN = [
        1.5,
        1.5,
        20.0,
        5.0,
        5.0,
        100.0,
        1.0,
        inf,
        inf,
        inf,  # slack_tb_fr
        inf,  # slack_tb_fl
        inf,  # slack_te_f
        inf,  # slack_te_r
        inf,  # slack_vx
    ]

    # number of nonlinear inequality constraints
    model.nh = 11
    model.nhN = 7

    # Nonlinear constraints are -inf <= h(z, p) <= 0
    model.hl = -inf * np.ones(model.nh)
    model.hu = np.zeros(model.nh)

    # h(x, u, p) <= 0
    model.ineq = lambda z, p: ineq(z, p, idx_params, model_params)
    model.ineqN = lambda z, p: ineqN(z, p, idx_params, model_params)

    # ========== OBJECTIVE ========== #
    slack_weights_dict = cost_params["slack_penalties"]
    model.objective = lambda z, p: obj(z, p, idx_params, slack_weights_dict)
    model.objectiveN = lambda z, p: objN(z, p, idx_params, slack_weights_dict)

    # ========== BFGS initialisation ========== #
    model.bfgs_init = np.identity(N_VARIABLE) * embotech_solver_params["bfgs_init"]

    """bfgs_temp = np.identity(N_VARIABLE)
    bfgs_temp[0, 0] = 0.05
    bfgs_temp[1, 1] = 5.0
    bfgs_temp[2, 2] = 0.1
    bfgs_temp[3, 3] = 0.1
    bfgs_temp[4, 4] = 0.01
    bfgs_temp[5, 5] = 0.01
    bfgs_temp[6, 6] = 0.1
    bfgs_temp[7, 7] = 0.003
    bfgs_temp[8, 8] = 0.05

    model.bfgs_init = bfgs_temp"""

    # ========== Generate the solver ========== #
    output = []  # defining custom outputs

    # [_x0_|_x1_|_..._|_xN-1_|_xN_]
    output.append(
        (
            "state_horizon",
            [i for i in range(N_HORIZON)],
            [i for i in range(N_STATE)],
        )
    )

    # [_u0_|_u1_|_..._|_uN-1_]
    output.append(
        (
            "input_horizon",
            [i for i in range(N_HORIZON - 1)],
            [i for i in range(N_STATE, N_STATE + N_INPUT)],
        )
    )

    """
  If at some point we need to get the dual variables, we can use the following outputs:

  Returns the dual variables of:
      - Equalities
      - Lower bound on variables
      - Upper bound on variables
      - Lower bound on slacks
      - Upper bound on slacks
      - Inequalities
      - Slacks on inequalities


  output.append(("eq_dual", [i for i in range(N_HORIZON_PLUS_ONE)], [i for i in range(N_STATE)], "nl_eq_dual"))

  output.append(("lb_var_dual", [i for i in range(N_HORIZON_PLUS_ONE)], [i for i in range(N_VARIABLE)], "nl_lb_var_dual"))
  output.append(("ub_var_dual", [i for i in range(N_HORIZON_PLUS_ONE)], [i for i in range(N_VARIABLE)], "nl_ub_var_dual"))

  output.append(("lb_slack_dual", [i for i in range(N_HORIZON_PLUS_ONE)], [i for i in range(N_VARIABLE)], "nl_lb_slack_dual"))
  output.append(("ub_slack_dual", [i for i in range(N_HORIZON_PLUS_ONE)], [i for i in range(N_VARIABLE)], "nl_ub_slack_dual"))

  output.append(("ip_ineq_dual", [i for i in range(N_HORIZON_PLUS_ONE)], [i for i in range(model.nh)], "nl_ip_ineq_dual"))
  output.append(("ineq_slack", [i for i in range(N_HORIZON_PLUS_ONE)], [i for i in range(model.nh)], "nl_ineq_slack"))
  """

    solver = model.generate_solver(codeoptions, output)

    return (
        model,
        solver,
        model_params,
        constraint_params,
        embotech_solver_params,
        common_solver_params,
        cost_params,
    )


if __name__ == "__main__":
    generate_solver()
    print("Solver generated successfully.")
    sys.exit(0)
