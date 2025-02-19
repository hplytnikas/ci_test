from load_mcap_to_dataframe import load_mcap_to_dataframe
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.widgets import Slider
import numpy as np
import matplotlib.colors as mcolors
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

    def extract_mpc_tire_predictions(mpc_df):
        tire_predictions = {
            "vx": [],
            "vy": [],
            "dpsi": [],
            "del_s": [],
            "ax": [],
            "l_f": [],
            "l_r": [],
            "C_l": [],
            "m": [],
            "B_tire_lat": [],
            "C_tire_lat": [],
            "D_tire_lat": [],
            "axt_abs_max": [],
            "ayt_abs_max": [],
            "Fx_f_actual": [],
            "Fy_f_actual": [],
            "Fx_r_actual": [],
            "Fy_r_actual": [],
        }

        for idx, row in mpc_df.iterrows():
            msg = row["message"]
            vx = np.array(msg.vx_prediction)
            vy = np.array(msg.vy_prediction)
            dpsi = np.array(msg.dpsi_prediction)
            del_s = np.array(msg.dels_prediction)
            ax = np.array(msg.ax_prediction)

            tire_predictions["vx"].append(vx)
            tire_predictions["vy"].append(vy)
            tire_predictions["dpsi"].append(dpsi)
            tire_predictions["del_s"].append(del_s)
            tire_predictions["ax"].append(ax)
            tire_predictions["l_f"].append(msg.model_lf)
            tire_predictions["l_r"].append(msg.model_lr)
            tire_predictions["C_l"].append(msg.model_cl)
            tire_predictions["m"].append(msg.model_mass)
            tire_predictions["B_tire_lat"].append(msg.model_b_tire)
            tire_predictions["C_tire_lat"].append(msg.model_c_tire)
            tire_predictions["D_tire_lat"].append(msg.model_d_tire)
            tire_predictions["axt_abs_max"].append(msg.constr_axt_abs_max)
            tire_predictions["ayt_abs_max"].append(msg.constr_ayt_abs_max)

            vx_n = np.maximum(1.0, vx[0])
            alpha_f = np.arctan2(vy[0] + msg.model_lf * dpsi[0], vx_n) - del_s[0]
            alpha_r = np.arctan2(vy[0] - msg.model_lr * dpsi[0], vx_n)

            F_downforce_per_axle = 0.5 * 1.225 * msg.model_cl * 1.2 * (vx[0] ** 2) / 2.0
            Fz_f = (
                msg.model_mass * 9.81 * msg.model_lr / (msg.model_lf + msg.model_lr)
                + F_downforce_per_axle
            )
            Fz_r = (
                msg.model_mass * 9.81 * msg.model_lf / (msg.model_lf + msg.model_lr)
                + F_downforce_per_axle
            )

            Fy_f = (
                -Fz_f
                * msg.model_d_tire
                * np.sin(msg.model_c_tire * np.arctan(msg.model_b_tire * alpha_f))
                / msg.model_mass
            )
            Fy_r = (
                -Fz_r
                * msg.model_d_tire
                * np.sin(msg.model_c_tire * np.arctan(msg.model_b_tire * alpha_r))
                / msg.model_mass
            )

            Fx_f = ax[0] / 2.0
            Fx_r = ax[0] / 2.0

            tire_predictions["Fx_f_actual"].append(Fx_f)
            tire_predictions["Fy_f_actual"].append(Fy_f)
            tire_predictions["Fx_r_actual"].append(Fx_r)
            tire_predictions["Fy_r_actual"].append(Fy_r)

        return tire_predictions

    def plot_tire_ellipse_constraints(mpc_df, tire_predictions):
        fig, (ax_front, ax_rear) = plt.subplots(1, 2, figsize=(14, 7))
        plt.subplots_adjust(left=0.1, bottom=0.25)

        timestamps = mpc_df.index
        vx_predictions = tire_predictions["vx"]
        vy_predictions = tire_predictions["vy"]
        dpsi_predictions = tire_predictions["dpsi"]
        del_s_predictions = tire_predictions["del_s"]
        ax_predictions = tire_predictions["ax"]
        l_f = tire_predictions["l_f"][0]
        l_r = tire_predictions["l_r"][0]
        C_l = tire_predictions["C_l"][0]
        m = tire_predictions["m"][0]
        B_tire_lat = tire_predictions["B_tire_lat"][0]
        C_tire_lat = tire_predictions["C_tire_lat"][0]
        D_tire_lat = tire_predictions["D_tire_lat"][0]
        axt_abs_max = tire_predictions["axt_abs_max"][0]
        ayt_abs_max = tire_predictions["ayt_abs_max"][0]
        rho = 1.225  # Air density (kg/m^3)
        g = 9.81  # Gravity (m/s^2)
        a_front = 1.2  # Assumed value for frontal area

        # Real ellipses for constraints
        front_ellipse = Ellipse(
            (0, 0),
            2 * ayt_abs_max,
            2 * axt_abs_max,
            edgecolor="#ffcc00",
            facecolor="none",
        )
        rear_ellipse = Ellipse(
            (0, 0),
            2 * ayt_abs_max,
            2 * axt_abs_max,
            edgecolor="#ffcc00",
            facecolor="none",
        )

        ax_front.add_patch(front_ellipse)
        ax_rear.add_patch(rear_ellipse)

        ax_front.set_xlim(-1.5 * ayt_abs_max, 1.5 * ayt_abs_max)
        ax_front.set_ylim(-1.5 * axt_abs_max, 1.5 * axt_abs_max)
        ax_front.set_aspect("equal", adjustable="datalim")
        ax_front.set_ylabel("Normalized Longitudinal Force (Fx/m)", color="#000000")
        ax_front.set_xlabel("Normalized Lateral Force (Fy/m)", color="#000000")
        ax_front.set_title(
            "Front Tire Forces",
            fontdict={"fontsize": 14, "fontweight": "bold", "color": "#d20a11"},
        )
        ax_front.grid(True, color="#e3e3e3")

        ax_rear.set_xlim(-1.5 * ayt_abs_max, 1.5 * ayt_abs_max)
        ax_rear.set_ylim(-1.5 * axt_abs_max, 1.5 * axt_abs_max)
        ax_rear.set_aspect("equal", adjustable="datalim")
        ax_rear.set_ylabel("Normalized Longitudinal Force (Fx/m)", color="#000000")
        ax_rear.set_xlabel("Normalized Lateral Force (Fy/m)", color="#000000")
        ax_rear.set_title(
            "Rear Tire Forces",
            fontdict={"fontsize": 14, "fontweight": "bold", "color": "#d20a11"},
        )
        ax_rear.grid(True, color="#e3e3e3")

        # Background scatter plots
        ax_front.scatter(
            [-f for f in tire_predictions["Fy_f_actual"]],
            tire_predictions["Fx_f_actual"],
            color="grey",
            alpha=0.5,
        )
        ax_rear.scatter(
            [-f for f in tire_predictions["Fy_r_actual"]],
            tire_predictions["Fx_r_actual"],
            color="grey",
            alpha=0.5,
        )

        # Create a colormap from red to green
        cmap = mcolors.LinearSegmentedColormap.from_list(
            "ci_colormap", ["#d20a11", "#008237"]
        )
        norm = plt.Normalize(vmin=0, vmax=1)

        front_scatters = ax_front.scatter([], [], c=[], cmap=cmap, norm=norm)
        rear_scatters = ax_rear.scatter([], [], c=[], cmap=cmap, norm=norm)

        front_texts = []
        rear_texts = []

        # Create a slider for time selection
        axcolor = "#ffcc00"
        ax_slider = plt.axes([0.1, 0.1, 0.8, 0.03], facecolor=axcolor)
        slider = Slider(ax_slider, "Time", 0, len(timestamps) - 1, valinit=0, valstep=1)

        def update(val):
            idx = int(slider.val)
            vx = vx_predictions[idx]
            vy = vy_predictions[idx]
            dpsi = dpsi_predictions[idx]
            del_s = del_s_predictions[idx]
            axm = ax_predictions[idx]

            vx_n = np.maximum(1.0, vx)
            alpha_f = np.arctan2(vy + l_f * dpsi, vx_n) - del_s
            alpha_r = np.arctan2(vy - l_r * dpsi, vx_n)

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

            norm_Fx_f = -Fy_f
            norm_Fy_f = Fx_f
            norm_Fx_r = -Fy_r
            norm_Fy_r = Fx_r

            # Determine color based on the amount of force
            front_colors = np.sqrt(norm_Fx_f**2 + norm_Fy_f**2) / np.max(
                np.sqrt(norm_Fx_f**2 + norm_Fy_f**2)
            )
            rear_colors = np.sqrt(norm_Fx_r**2 + norm_Fy_r**2) / np.max(
                np.sqrt(norm_Fx_r**2 + norm_Fy_r**2)
            )

            front_scatters.set_offsets(np.c_[norm_Fx_f, norm_Fy_f])
            front_scatters.set_array(front_colors)
            rear_scatters.set_offsets(np.c_[norm_Fx_r, norm_Fy_r])
            rear_scatters.set_array(rear_colors)

            # Clear previous annotations
            for txt in front_texts:
                txt.remove()
            front_texts.clear()

            for txt in rear_texts:
                txt.remove()
            rear_texts.clear()

            for i, (fx, fy) in enumerate(zip(norm_Fx_f, norm_Fy_f)):
                txt = ax_front.annotate(
                    f"{i}",
                    (fx, fy),
                    textcoords="offset points",
                    xytext=(0, 5),
                    ha="center",
                )
                front_texts.append(txt)

            for i, (fx, fy) in enumerate(zip(norm_Fx_r, norm_Fy_r)):
                txt = ax_rear.annotate(
                    f"{i}",
                    (fx, fy),
                    textcoords="offset points",
                    xytext=(0, 5),
                    ha="center",
                )
                rear_texts.append(txt)

            fig.canvas.draw_idle()

        slider.on_changed(update)
        plt.show()

    mpc_df = dataframes[MpcLoggingTopic]
    tire_predictions = extract_mpc_tire_predictions(mpc_df)
    plot_tire_ellipse_constraints(mpc_df, tire_predictions)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize MPC predictions from an MCAP file."
    )
    parser.add_argument("file_path", type=str, help="Path to the MCAP file.")
    args = parser.parse_args()
    main(args.file_path)
