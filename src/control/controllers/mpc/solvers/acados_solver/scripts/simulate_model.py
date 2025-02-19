"""
    Script to generate the solver C code for the NLP OCP problem
    written for the ACADOS solver
"""

import matplotlib.pyplot as plt
import numpy as np
from setup_model import setup_ocp
from load_mpc_parameters import load_mpc_yaml_params
from plot_sim_ocp import plot_nlp_dynamics


# constant curvature horizon for simulation
CONST_CURV = 0.1


def main(use_stepped_sim: bool = False):
    """
    Simulate the OCP problem with the acados solver, NOT to be used for code generation.

    Args:
        use_stepped_sim (bool, optional): If True, the simulation will be stepped and the NLP dynamics will be plotted. Defaults to False.
    """

    constraint_params, cost_params, model_params, _, common_solver_params = (
        load_mpc_yaml_params()
    )

    """ =========== INITIAL STATE FOR SIMULATION ============ """
    # x0 =        [n,   mu,  vx,  vy,  dpsi, ax,  del_s]
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    """ =========== GET SOLVER AND INTEGRATOR ============ """
    ocp_solver, integrator = setup_ocp(x0, simulate_ocp=True)

    """ =========== SET MODEL PARAMS ============ """
    kappa_ref = CONST_CURV  # reference curvature

    m = model_params["m"]  # mass
    l_f = model_params["l_f"]
    l_r = model_params["l_r"]
    C_d = model_params["C_d"]  # effective drag coefficient
    C_r = model_params["C_r"]  # const. rolling resistance
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

    for j in range(common_solver_params["N_horizon"]):
        ocp_solver.set(j, "p", paramvec)
    integrator.set("p", paramvec)

    """ =========== GET DIMS ============ """
    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu

    """ =========== GET CONSTRAINTS ========= """
    idx_b_x = ocp_solver.acados_ocp.constraints.idxbx
    # idx_b_x = np.array([])
    lb_x = ocp_solver.acados_ocp.constraints.lbx
    ub_x = ocp_solver.acados_ocp.constraints.ubx
    idx_b_u = ocp_solver.acados_ocp.constraints.idxbu
    # idx_b_u = np.array([])
    lb_u = ocp_solver.acados_ocp.constraints.lbu
    ub_u = ocp_solver.acados_ocp.constraints.ubu

    """ =========== SET SIMULATION PARAMS ============ """
    Nsim = 500
    stepping_start_idx = 0
    simX = np.ndarray((Nsim + 1, nx))
    simU = np.ndarray((Nsim, nu))

    # get s-state sim vector because it is reset
    # to 0 at the beginning of each prediction horizon
    s_values = np.zeros((Nsim + 1, 1))
    s_values[0, 0] = x0[0]

    # set initial simulation state
    simX[0, :] = x0

    # init prediction trajectories
    X_predict = np.zeros((common_solver_params["N_horizon"], nx))
    U_predict = np.zeros((common_solver_params["N_horizon"] - 1, nu))

    # Plot labels
    inputs_lables = (r"$\dot{a}_{x}$", r"$\dot{\delta}_s$")
    states_lables = (
        "$n$",
        r"$\mu$",
        "$v_x$",
        "$v_y$",
        r"$\dot{\psi}$",
        r"$a_x$",
        r"$\delta_s$",
    )

    # Vectors that contain the solve times at each simulation step
    t_ocp = np.zeros((Nsim))

    # closed loop sim (may be stepped)
    for i in range(Nsim):

        """Solve OCP"""
        init_state = np.copy(simX[i, :])
        # reset initial state s0 to 0 for MPC
        # init_state[0] = 0.0
        simU[i, :] = ocp_solver.solve_for_x0(
            x0_bar=init_state, fail_on_nonzero_status=False
        )

        # get elapsed solve time
        t_ocp[i] = ocp_solver.get_stats("time_tot")

        # simulate system one step
        simX[i + 1, :] = integrator.simulate(x=simX[i, :], u=simU[i, :])
        s_values[i + 1] = simX[i + 1, 0]  # + s_values[i]
        # simX[i + 1, 0] = 0.0

        print(
            f"{100 * (i+1)/Nsim:.2f} % of sim solved, solve time: {1000*t_ocp[i]:.3f} [ms]"
        )

        if use_stepped_sim and i > stepping_start_idx:

            for j in range(common_solver_params["N_horizon"] - 1):
                X_predict[j, :] = ocp_solver.get(j, "x")
                U_predict[j, :] = ocp_solver.get(j, "u")

            X_predict[common_solver_params["N_horizon"] - 1, :] = ocp_solver.get(
                common_solver_params["N_horizon"] - 1, "x"
            )

            t_predict = (
                common_solver_params["T_final"] / common_solver_params["N_horizon"]
            ) * i + np.linspace(
                0, common_solver_params["T_final"], common_solver_params["N_horizon"]
            )

            plot_nlp_dynamics(
                np.linspace(
                    0,
                    (
                        common_solver_params["T_final"]
                        / common_solver_params["N_horizon"]
                    )
                    * i,
                    i + 1,
                ),
                idx_b_x,
                lb_x,
                ub_x,
                idx_b_u,
                lb_u,
                ub_u,
                simU[:i, :],
                simX[: i + 1, :],
                inputs_lables,
                states_lables,
                U_predict,
                X_predict,
                t_predict,
            )

            input("Press Enter to continue...")

    # scale to milliseconds
    t_ocp *= 1000
    print(
        f"Computation time (nlp-ocp) in ms: min {np.min(t_ocp):.3f} median {np.median(t_ocp):.3f} max {np.max(t_ocp):.3f}"
    )

    # plot s values
    simX[:, 0] = np.squeeze(s_values)

    tvec = np.linspace(
        0,
        (common_solver_params["T_final"] / common_solver_params["N_horizon"]) * Nsim,
        Nsim + 1,
    )

    plot_nlp_dynamics(
        tvec,
        idx_b_x,
        lb_x,
        ub_x,
        idx_b_u,
        lb_u,
        ub_u,
        simU,
        simX,
        inputs_lables,
        states_lables,
    )

    ocp_solver = None

    # Plot the OCP solve time
    plt.figure()
    plt.title("acados OCP solve time")
    plt.plot(tvec[:-1], t_ocp, "kx", markersize=1.0, label="OCP solve time")
    plt.vlines(tvec[:-1], 0, t_ocp, colors="k", linewidth=0.1)
    plt.xlabel("Time [s]")
    plt.ylabel("Time [ms]")
    plt.ylim([0, 25])
    plt.legend()

    # Plot a violin plot of the OCP solve time
    plt.figure()
    plt.title("acados OCP solve time")
    violin_parts = plt.violinplot(
        t_ocp, showmeans=False, showmedians=True, showextrema=True
    )

    for pc in violin_parts["bodies"]:
        pc.set_facecolor("grey")
        pc.set_edgecolor("black")

    plt.ylabel("Time [ms]")
    plt.xlabel("Frequency")
    plt.ylim([0, 25])
    plt.grid("both")

    # Show the plots
    plt.show()


"""
SQP Solver Status:
    0 --> ACADOS_SUCCESS,
    1 --> ACADOS_FAILURE,
    2 --> ACADOS_MAXITER,
    3 --> ACADOS_MINSTEP,
    4 --> ACADOS_QP_FAILURE,
    5 --> ACADOS_READY

HPIPM Solver Status:
    0 --> SUCCESS, // found solution satisfying accuracy tolerance
    1 --> MAX_ITER, // maximum iteration number reached
    2 --> MIN_STEP, // minimum step length reached
    3 --> NAN_SOL, // NaN in solution detected
    4 --> INCONS_EQ, // unconsistent equality constraints

"""

if __name__ == "__main__":
    main(use_stepped_sim=False)
