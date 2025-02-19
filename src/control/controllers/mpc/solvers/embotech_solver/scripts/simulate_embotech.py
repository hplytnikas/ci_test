import forcespro.nlp
import matplotlib.pyplot as plt
import numpy as np
from codegen_embotech_functions import getContinuousDynamics
from codegen_embotech import generate_solver


def simulate_and_plot_embotech():
    # ========== Simulation ========== #
    print(" --- Generating the solver --- ")

    (
        model,
        solver,
        model_params,
        constraint_params,
        embotech_solver_params,
        common_solver_params,
        cost_params,
    ) = generate_solver()

    print(" --- Starting the simulation --- ")

    m = model_params["m"]
    l_f = model_params["l_f"]
    l_r = model_params["l_r"]

    C_d = model_params["C_d"]
    C_r = model_params["C_r"]
    C_l = model_params["C_l"]

    B_tire_lat = model_params["B_tire"]
    C_tire_lat = model_params["C_tire"]
    D_tire_lat = model_params["D_tire"]

    h_cg = model_params["h_cg"]
    Iz = model_params["Iz"]
    l_to_front = model_params["l_to_front"]
    l_to_rear = model_params["l_to_rear"]
    car_width = model_params["car_width"]

    q_ds = cost_params["q_ds"]
    q_n = cost_params["q_n"]
    q_mu = cost_params["q_mu"]
    r_dax = cost_params["r_dax"]
    r_ddels = cost_params["r_ddels"]

    q_ds_terminal = cost_params["q_ds_terminal"]
    q_n_terminal = cost_params["q_n_terminal"]
    q_mu_terminal = cost_params["q_mu_terminal"]

    trackbound_min = constraint_params["trackbound_min"]
    trackbound_max = constraint_params["trackbound_max"]
    vx_max = constraint_params["vx_max"]
    vx_max_terminal = constraint_params["vx_max_terminal"]

    axt_abs_max = constraint_params["axt_abs_max"]
    ayt_abs_max = constraint_params["ayt_abs_max"]

    dels_abs_max = constraint_params["dels_abs_max"]
    dax_min = constraint_params["dax_min"]
    dax_max = constraint_params["dax_max"]
    ddels_abs_max = constraint_params["ddels_abs_max"]

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

    curvature = 0.1

    paramvec = np.array(
        [
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
            curvature,
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
        ]
    )

    x1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    kmax = 500

    x = np.zeros((embotech_solver_params["n_states"], kmax + 1))
    x[:, 0] = x1
    u = np.zeros((embotech_solver_params["n_inputs"], kmax))
    problem = {}

    solvetime = []
    iters = []
    feval = []

    problem["x0"] = np.zeros(model.nvar * model.N)

    for k in range(kmax):
        # initial state
        init_state = x[:, k]
        if x[2, k] < 0.0:
            init_state[2] = 0.0

        problem["xinit"] = init_state

        # curvature
        problem["all_parameters"] = np.tile(paramvec, (model.N,))

        # call the solver
        solverout, exitflag, info = solver.solve(problem)
        if exitflag != 1:
            print("Some problem in solver")
            print("Exitflag : " + str(exitflag))
            # print state
            print("State : " + str(x[:, k]))
            break

        # get the output from the solver
        u[0, k] = solverout["input_horizon"][0]
        u[1, k] = solverout["input_horizon"][1]

        solvetime.append(info.solvetime)
        iters.append(info.it)
        feval.append(info.fevalstime)

        # simulate the dynamics
        x[:, k + 1] = (
            forcespro.nlp.integrate(
                getContinuousDynamics,
                x[:, k],
                u[:, k],
                paramvec,
                idx_params,
                model_params,
                integrator=forcespro.nlp.integrators.RK4,
                stepsize=common_solver_params["T_final"]
                / common_solver_params["N_horizon"],
            )
            .full()
            .reshape(
                embotech_solver_params["n_states"],
            )
        )

    # average solve time
    print("average solve time : " + str(sum(solvetime) / len(solvetime) * 1000) + " ms")
    print("max solve time : " + str(max(solvetime) * 1000) + " ms")
    print("min solve time : " + str(min(solvetime) * 1000) + " ms")

    print(" ")

    # average solve time
    print("average #iterations : " + str(sum(iters) / len(iters)))
    print("max #iterations : " + str(max(iters)))
    print("min #iterations : " + str(min(iters)))

    print(" ")

    # average solve time
    print("average feval time : " + str(sum(feval) / len(feval) * 1000) + " ms")
    print("max feval time : " + str(max(feval) * 1000) + " ms")
    print("min feval time : " + str(min(feval) * 1000) + " ms")

    # Plot results

    n = x[0, :]
    mu = x[1, :] * 180 / np.pi
    vx = x[2, :]
    vy = x[3, :]
    dpsi = x[4, :]
    Fx = x[5, :] * model_params["m"]
    dels = x[6, :] * 180 / np.pi

    dax = u[0, :]
    ddels = u[1, :]

    plt.figure()
    plt.subplot(5, 2, 1)
    plt.plot(n)
    plt.title("Deviation [m]")
    plt.grid()

    plt.subplot(5, 2, 2)
    plt.plot(mu)
    plt.title("Heading [°]")
    plt.grid()

    plt.subplot(5, 2, 3)
    plt.plot(vx)
    plt.title("Longitudinal speed [m/s]")
    plt.grid()

    plt.subplot(5, 2, 4)
    plt.plot(vy)
    plt.title("Lateral speed [m/s]")
    plt.grid()

    plt.subplot(5, 2, 5)
    plt.plot(dpsi)
    plt.title("Yaw [rad/s]")
    plt.grid()

    plt.subplot(5, 2, 6)
    plt.plot(Fx)
    plt.title("Longitudinal force [N]")
    plt.grid()

    plt.subplot(5, 2, 7)
    plt.plot(dels)
    plt.title("Steering angle [°]")
    plt.grid()

    plt.subplot(5, 2, 8)
    plt.plot(dax)
    plt.title("Acceleration rate [m/s²/s]")
    plt.grid()

    plt.subplot(5, 2, 9)
    plt.plot(ddels)
    plt.title("Steering angle rate [°/s]")
    plt.grid()

    plt.subplot(5, 2, 10)
    plt.plot(np.array(solvetime) * 1000)
    plt.title("Solve time [ms]")
    plt.grid()

    plt.show()


if __name__ == "__main__":
    simulate_and_plot_embotech()
