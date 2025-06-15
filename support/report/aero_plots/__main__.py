from pathlib import Path

from .plot import PlotConfig, create_aero_plot


# ==============================================================================
# --- USER CONFIGURATION ---
# ==============================================================================

# ROOT = Path(__file__).parent

# CONFIG_1 = {
#     "filepath": Path(
#         ROOT
#         / "../../pipeline/data/output/decoupled/0250_coupled-axis_r0.1-0.0-0.0_w0.5_s0.0_t0.0.parquet"
#     ),
#     "label": "Pitch Rate: 0.1 rad/s",
# }

# CONFIG_2 = {
#     "filepath": Path(
#         ROOT
#         / "../../pipeline/data/output/decoupled/0251_coupled-axis_r2.0-0.0-0.0_w0.5_s0.0_t0.0.parquet"
#     ),
#     "label": "Pitch Rate: 2.0 rad/s",
# }

# OUTPUT_FILE = ROOT / "lift_drag_comparison_pitch.png"
# PLOT_TITLE = "Aerodynamic Analysis: Effect of Pitch Rate on Lift & Drag"


# ROOT = Path(__file__).parent

# CONFIG_1 = {
#     "filepath": Path(
#         ROOT
#         / "../../pipeline/data/output/decoupled/0246_coupled-axis_r0.0-0.0-0.1_w0.5_s0.0_t0.0.parquet"
#     ),
#     "label": "Yaw Rate: 0.1 rad/s",
# }

# CONFIG_2 = {
#     "filepath": Path(
#         ROOT
#         / "../../pipeline/data/output/decoupled/0247_coupled-axis_r0.0-0.0-2.0_w0.5_s0.0_t0.0.parquet"
#     ),
#     "label": "Yaw Rate: 2.0 rad/s",
# }

# OUTPUT_FILE = ROOT / "lift_drag_comparison_yaw_moment.png"
# PLOT_TITLE = "Aerodynamic Analysis: Effect of Yaw Rate on Lift & Drag"


ROOT = Path(__file__).parent

CONFIG_1 = {
    "filepath": Path(ROOT / "../data/250614_free-flight-extended/no-model.parquet"),
    "label": "Yaw Rate: 0.1 rad/s",
}

CONFIG_2 = {
    "filepath": Path(ROOT / "../data/250614_free-flight-extended/no-model.parquet"),
    "label": "Yaw Rate: 2.0 rad/s",
}

OUTPUT_FILE = ROOT / "lift_drag_comparison_free_flight.png"
PLOT_TITLE = "Aerodynamic Analysis: Effect of Yaw Rate on Lift & Drag"


def main():
    config1 = PlotConfig(filepath=CONFIG_1["filepath"], label=CONFIG_1["label"])
    config2 = PlotConfig(filepath=CONFIG_2["filepath"], label=CONFIG_2["label"])

    create_aero_plot(config1, config2, OUTPUT_FILE, PLOT_TITLE)


if __name__ == "__main__":
    main()
