from load_mcap_to_dataframe import load_mcap_to_dataframe
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse


def main(file_path):
    # Define the topic names
    MpcLoggingTopic = "/control/mpc_logging"

    # List of topics to process
    topics_to_process = [MpcLoggingTopic]

    # Load the MCAP file into a dictionary of DataFrames
    dataframes = load_mcap_to_dataframe(file_path, topics_to_process)

    # Display the first few rows of the DataFrames
    for topic, df in dataframes.items():
        print(f"Topic: {topic}")
        print(df.head())
        print()

    def compute_costs(mpc_df):
        costs = {
            "cost_sd": [],
            "cost_sd_terminal": [],
            "cost_n": [],
            "cost_n_terminal": [],
            "cost_mu": [],
            "cost_mu_terminal": [],
            "cost_dax": [],
            "cost_ddel_s": [],
            "cost_slack_tb_rr": [],
            "cost_slack_tb_rl": [],
            "cost_slack_tb_fr": [],
            "cost_slack_tb_fl": [],
            "cost_slack_te_r": [],
            "cost_slack_te_f": [],
            "cost_slack_vx": [],
            "cost_slack_vx_terminal": [],
            "total_cost": [],
        }

        for idx, row in mpc_df.iterrows():
            msg = row["message"]
            q_ds = msg.cost_q_ds
            q_ds_terminal = msg.cost_q_ds_terminal
            q_n = msg.cost_q_n
            q_n_terminal = msg.cost_q_n_terminal
            q_mu = msg.cost_q_mu
            q_mu_terminal = msg.cost_q_mu_terminal
            r_dax = msg.cost_r_dax
            r_ddels = msg.cost_r_ddels

            trackbound_min_hor = np.array(msg.trackbound_min_horizon)
            trackbound_max_hor = np.array(msg.trackbound_max_horizon)
            if len(trackbound_min_hor) != len(trackbound_max_hor):
                print("Trackbound horizon mismatch")
                return
            tb_shrinkage = msg.constr_trackbound_shrinkage

            axt_abs_max = msg.constr_axt_abs_max
            ayt_abs_max = msg.constr_ayt_abs_max
            vx_max = msg.constr_vx_max
            vx_max_terminal = msg.constr_vx_max_terminal

            l_to_front = msg.model_l_to_front
            l_to_rear = msg.model_l_to_rear
            car_width = msg.model_car_width
            l_f = msg.model_lf
            l_r = msg.model_lr
            C_l = msg.model_cl
            m = msg.model_mass
            B_tire_lat = msg.model_b_tire
            C_tire_lat = msg.model_c_tire
            D_tire_lat = msg.model_d_tire
            g = 9.81
            a_front = 1.2
            rho = 1.225

            z_slack_tb = 0.5
            z_slack_te = 5.0
            z_slack_vx = 2.0

            q_slack_tb = 1.0
            q_slack_te = 10.0
            q_slack_vx = 10.0

            vx = np.array(msg.vx_prediction)
            vy = np.array(msg.vy_prediction)
            dpsi = np.array(msg.dpsi_prediction)
            mu = np.array(msg.mu_prediction)
            n = np.array(msg.n_prediction)
            del_s = np.array(msg.dels_prediction)
            axm = np.array(msg.ax_prediction)
            dax = np.array(msg.dax_prediction)
            ddel_s = np.array(msg.ddels_prediction)
            kappa_ref = np.array(msg.curvature_horizon)

            if len(kappa_ref) != len(vx):
                if len(kappa_ref) == len(vx) - 1:
                    # print("Appending 0.0 to curvature horizon")
                    kappa_ref = np.append(kappa_ref, 0.0)
                else:
                    return

            if len(trackbound_max_hor) != len(vx):
                if len(trackbound_max_hor) == len(vx) - 1:
                    # print("Appending +-1.5m to trackbound horizons")
                    trackbound_max_hor = np.append(trackbound_max_hor, 1.5)
                    trackbound_min_hor = np.append(trackbound_min_hor, -1.5)

            cost_sd = (
                -q_ds
                * (vx[:-1] * np.cos(mu[:-1]) - vy[:-1] * np.sin(mu[:-1]))
                / (1 - n[:-1] * kappa_ref[:-1])
            )
            cost_sd_terminal = (
                -q_ds_terminal
                * (vx[-1] * np.cos(mu[-1]) - vy[-1] * np.sin(mu[-1]))
                / (1 - n[-1] * kappa_ref[-1])
            )
            cost_n = q_n * n[:-1] ** 2
            cost_n_terminal = q_n_terminal * n[-1] ** 2
            cost_mu = q_mu * mu[:-1] ** 2
            cost_mu_terminal = q_mu_terminal * mu[-1] ** 2
            cost_dax = r_dax * dax**2
            cost_ddel_s = r_ddels * ddel_s**2

            slacks_tb_rr = np.maximum(
                0.0,
                tb_shrinkage * trackbound_min_hor[1:]
                - n[1:]
                + l_to_rear * np.sin(mu[1:])
                + car_width / 2.0 * np.cos(mu[1:]),
            )
            slacks_tb_rl = np.maximum(
                0.0,
                -tb_shrinkage * trackbound_max_hor[1:]
                + n[1:]
                - l_to_rear * np.sin(mu[1:])
                + car_width / 2.0 * np.cos(mu[1:]),
            )
            slacks_tb_fr = np.maximum(
                0.0,
                tb_shrinkage * trackbound_min_hor[1:]
                - n[1:]
                - l_to_front * np.sin(mu[1:])
                + car_width / 2.0 * np.cos(mu[1:]),
            )
            slacks_tb_fl = np.maximum(
                0.0,
                -tb_shrinkage * trackbound_max_hor[1:]
                + n[1:]
                + l_to_front * np.sin(mu[1:])
                + car_width / 2.0 * np.cos(mu[1:]),
            )

            vx_n = np.maximum(1.0, vx)

            alpha_f = np.arctan2(vy + l_f * dpsi, vx_n) - del_s
            alpha_r = np.arctan2(vy - l_r * dpsi, vx_n)

            # Tire Forces
            F_downforce_per_axle = 0.5 * rho * C_l * a_front * (vx**2) / 2.0
            Fz_f = m * g * l_r / (l_f + l_r) + F_downforce_per_axle
            Fz_r = m * g * l_f / (l_f + l_r) + F_downforce_per_axle

            Fy_f = (
                -Fz_f
                * D_tire_lat
                * np.sin(C_tire_lat * np.arctan(B_tire_lat * alpha_f))
                / m
            )
            Fy_r = (
                -Fz_r
                * D_tire_lat
                * np.sin(C_tire_lat * np.arctan(B_tire_lat * alpha_r))
                / m
            )

            Fx_f = axm / 2.0
            Fx_r = axm / 2.0

            slacks_te_f = np.maximum(
                0.0, (Fx_f[1:] / axt_abs_max) ** 2 + (Fy_f[1:] / ayt_abs_max) ** 2 - 1
            )
            slacks_te_r = np.maximum(
                0.0, (Fx_r[1:] / axt_abs_max) ** 2 + (Fy_r[1:] / ayt_abs_max) ** 2 - 1
            )

            slack_vx = np.maximum(0.0, vx[1:-1] - vx_max)
            slack_vx_terminal = np.maximum(0.0, vx[-1] - vx_max_terminal)

            cost_slack_tb_rr = q_slack_tb * slacks_tb_rr**2 + z_slack_tb * slacks_tb_rr
            cost_slack_tb_rl = q_slack_tb * slacks_tb_rl**2 + z_slack_tb * slacks_tb_rl
            cost_slack_tb_fr = q_slack_tb * slacks_tb_fr**2 + z_slack_tb * slacks_tb_fr
            cost_slack_tb_fl = q_slack_tb * slacks_tb_fl**2 + z_slack_tb * slacks_tb_fl

            cost_slack_te_r = q_slack_te * slacks_te_r**2 + z_slack_te * slacks_te_r
            cost_slack_te_f = q_slack_te * slacks_te_f**2 + z_slack_te * slacks_te_f

            cost_slack_vx = q_slack_vx * slack_vx**2 + z_slack_vx * slack_vx
            cost_slack_vx_terminal = (
                q_slack_vx * slack_vx_terminal**2 + z_slack_vx * slack_vx_terminal
            )

            total_cost = (
                np.sum(cost_sd)
                + np.sum(cost_sd_terminal)
                + np.sum(cost_n)
                + np.sum(cost_n_terminal)
                + np.sum(cost_mu)
                + np.sum(cost_mu_terminal)
                + np.sum(cost_dax)
                + np.sum(cost_ddel_s)
                + np.sum(cost_slack_tb_rr)
                + np.sum(cost_slack_tb_rl)
                + np.sum(cost_slack_tb_fr)
                + np.sum(cost_slack_tb_fl)
                + np.sum(cost_slack_te_r)
                + np.sum(cost_slack_te_f)
                + np.sum(cost_slack_vx)
                + np.sum(cost_slack_vx_terminal)
            )

            costs["cost_sd"].append(np.sum(cost_sd))
            costs["cost_sd_terminal"].append(np.sum(cost_sd_terminal))
            costs["cost_n"].append(np.sum(cost_n))
            costs["cost_n_terminal"].append(np.sum(cost_n_terminal))
            costs["cost_mu"].append(np.sum(cost_mu))
            costs["cost_mu_terminal"].append(np.sum(cost_mu_terminal))
            costs["cost_dax"].append(np.sum(cost_dax))
            costs["cost_ddel_s"].append(np.sum(cost_ddel_s))

            costs["total_cost"].append(total_cost)

            costs["cost_slack_tb_rr"].append(np.sum(cost_slack_tb_rr))
            costs["cost_slack_tb_rl"].append(np.sum(cost_slack_tb_rl))
            costs["cost_slack_tb_fr"].append(np.sum(cost_slack_tb_fr))
            costs["cost_slack_tb_fl"].append(np.sum(cost_slack_tb_fl))
            costs["cost_slack_te_r"].append(np.sum(cost_slack_te_r))
            costs["cost_slack_te_f"].append(np.sum(cost_slack_te_f))
            costs["cost_slack_vx"].append(np.sum(cost_slack_vx))
            costs["cost_slack_vx_terminal"].append(np.sum(cost_slack_vx_terminal))

        return costs

    def plot_cost_proportions(mpc_df, costs):
        fig, (ax1, ax2) = plt.subplots(
            2, 1, figsize=(14, 12), gridspec_kw={"height_ratios": [2, 1]}
        )

        indices = np.arange(len(mpc_df))

        cost_sd = np.array(costs["cost_sd"])
        cost_sd_terminal = np.array(costs["cost_sd_terminal"])
        cost_n = np.array(costs["cost_n"])
        cost_n_terminal = np.array(costs["cost_n_terminal"])
        cost_mu = np.array(costs["cost_mu"])
        cost_mu_terminal = np.array(costs["cost_mu_terminal"])
        cost_dax = np.array(costs["cost_dax"])
        cost_ddel_s = np.array(costs["cost_ddel_s"])
        cost_slack_tb_rr = np.array(costs["cost_slack_tb_rr"])
        cost_slack_tb_rl = np.array(costs["cost_slack_tb_rl"])
        cost_slack_tb_fr = np.array(costs["cost_slack_tb_fr"])
        cost_slack_tb_fl = np.array(costs["cost_slack_tb_fl"])
        cost_slack_te_r = np.array(costs["cost_slack_te_r"])
        cost_slack_te_f = np.array(costs["cost_slack_te_f"])
        cost_slack_vx = np.array(costs["cost_slack_vx"])
        cost_slack_vx_terminal = np.array(costs["cost_slack_vx_terminal"])

        total_cost = np.array(costs["total_cost"])

        # Create a DataFrame for easier plotting
        cost_df = pd.DataFrame(
            {
                "index": indices,
                "Progress Rate": cost_sd,
                "Progress Rate Terminal": cost_sd_terminal,
                "Lateral Deviation": cost_n,
                "Lateral Deviation Terminal": cost_n_terminal,
                "Heading Deviation": cost_mu,
                "Heading Deviation Terminal": cost_mu_terminal,
                "Ax Change Rate": cost_dax,
                "Dels Change Rate": cost_ddel_s,
                "Trackbound RR": cost_slack_tb_rr,
                "Trackbound RL": cost_slack_tb_rl,
                "Trackbound FR": cost_slack_tb_fr,
                "Trackbound FL": cost_slack_tb_fl,
                "Tire Forces R": cost_slack_te_r,
                "Tire Forces F": cost_slack_te_f,
                "Vx Slack": cost_slack_vx,
                "Vx Slack Terminal": cost_slack_vx_terminal,
            }
        ).set_index("index")

        # Plot the grouped bar plot
        cost_df.plot(
            kind="bar",
            stacked=True,
            ax=ax1,
            width=0.8,
            color=[
                "blue",
                "cyan",
                "green",
                "lime",
                "red",
                "orange",
                "purple",
                "pink",
                "brown",
                "gray",
                "black",
                "yellow",
                "magenta",
                "olive",
                "teal",
                "navy",
            ],
        )

        ax1.set_xticks(indices)
        ax1.set_xticklabels(indices)
        ax1.set_ylabel("Cost")
        ax1.set_title("Cost Contributions Over Time")
        ax1.legend(loc="upper left")
        ax1.grid(True)

        # Plot vx and dels in the second subplot
        vx = np.array([msg.vx_prediction[0] for msg in mpc_df["message"]])
        dels = np.array([msg.dels_prediction[0] for msg in mpc_df["message"]])

        ax2.plot(indices, vx, "b-", label="vx")
        ax2.set_ylabel("vx", color="b")
        ax2.tick_params(axis="y", labelcolor="b")

        ax3 = ax2.twinx()
        ax3.plot(indices, dels, "r-", label="dels")
        ax3.set_ylabel("dels", color="r")
        ax3.tick_params(axis="y", labelcolor="r")

        fig.tight_layout()
        plt.show()

    mpc_df = dataframes[MpcLoggingTopic]
    costs = compute_costs(mpc_df)
    plot_cost_proportions(mpc_df, costs)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize MPC predictions from an MCAP file."
    )
    parser.add_argument("file_path", type=str, help="Path to the MCAP file.")
    args = parser.parse_args()
    main(args.file_path)
