from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from numpy import rad2deg


# ==============================================================================
# --- SCRIPT LOGIC (Generally no need to edit below this line) ---
# ==============================================================================

# --- Constants ---
POINT_SIZE = 20
POINT_ALPHA = 0.6
MODEL_LINEWIDTH = 2.0

# Define required columns, including the analytical model outputs
REQUIRED_COLS = ["alpha", "lift", "drag"]


@dataclass
class PlotConfig:
    """A simple structure to hold configuration for one experimental run."""

    filepath: Path
    label: str


def load_and_prepare_df(config: PlotConfig) -> pd.DataFrame:
    """
    Loads a single DataFrame, validates it, and prepares it for plotting.
    """
    if not config.filepath.exists():
        raise FileNotFoundError(f"Input file not found: {config.filepath}")

    # Load data from .parquet (or .csv)
    if config.filepath.suffix.lower() == ".parquet":
        df = pd.read_parquet(config.filepath)
    elif config.filepath.suffix.lower() == ".csv":
        df = pd.read_csv(config.filepath)
    else:
        raise ValueError(f"Unsupported file format: {config.filepath.suffix}")

    # Validate required columns
    missing_cols = [col for col in REQUIRED_COLS if col not in df.columns]
    if missing_cols:
        raise KeyError(
            f"File '{config.filepath.name}' is missing required columns: {missing_cols}"
        )

    # --- Prepare data for plotting ---
    df["source"] = config.label
    # mid_index = len(df) // 2
    mid_index = len(df)
    df["motion_phase"] = "Forward Motion"
    df.loc[mid_index:, "motion_phase"] = "Backward Motion"
    df["plot_group"] = df["source"] + " - " + df["motion_phase"]

    df["alpha_deg"] = rad2deg(df["alpha"])
    df["beta_deg"] = rad2deg(df["beta"])

    return df


def create_aero_plot(
    config1: PlotConfig, config2: PlotConfig, output_file: Path, plot_title: str
):
    """
    Generates and saves a comparative plot of aerodynamic forces for two experiments.
    """
    print("--- Starting Aerodynamic Plot Generation ---")

    # Load and prepare both dataframes
    try:
        df1 = load_and_prepare_df(config1)
        print(f"✔ Loaded and prepared '{config1.filepath.name}' ({len(df1)} rows)")
        df2 = load_and_prepare_df(config2)
        print(f"✔ Loaded and prepared '{config2.filepath.name}' ({len(df2)} rows)")
    except (FileNotFoundError, KeyError, ValueError) as e:
        print(f"Error: {e}")
        return

    # combined_df = pd.concat([df1, df2], ignore_index=True)
    combined_df = df1
    print("\nGenerating plots...")

    # --- Plotting Setup ---
    sns.set_theme(style="whitegrid", context="talk")
    fig, axes = plt.subplots(2, 2, figsize=(22, 9), sharey=False)
    fig.suptitle(plot_title, fontsize=24, y=0.98)

    # --- Smart Palette Definition ---
    # Assigns related colors to forward/backward motion and the model line
    palette = {
        f"{config1.label} - Forward Motion": "C0",  # Dark Blue
        f"{config1.label} - Backward Motion": "C9",  # Light Blue
        f"{config2.label} - Forward Motion": "C1",  # Orange
        f"{config2.label} - Backward Motion": "C8",  # Light Orange/Brown
    }
    model_colors = {config1.label: "C0", config2.label: "C1"}

    # --- Plot 1: Lift vs. Angle of Attack ---
    ax1 = axes[0, 0]
    sns.scatterplot(
        data=combined_df,
        x="alpha_deg",
        y="lift",
        hue="plot_group",
        palette=palette,
        s=POINT_SIZE,
        alpha=POINT_ALPHA,
        ax=ax1,
        edgecolor=None,
    )
    # # Overlay the analytical model for both experiments
    # for cfg in [config1, config2]:
    #     model_fwd = combined_df[
    #         (combined_df["source"] == cfg.label)
    #         & (combined_df["motion_phase"] == "Forward Motion")
    #     ].sort_values("beta_deg")
    #     ax1.plot(
    #         model_fwd["beta_deg"],
    #         model_fwd["mz_model"],
    #         color=model_colors[cfg.label],
    #         linewidth=MODEL_LINEWIDTH,
    #         label=f"{cfg.label} - Model",
    #     )
    # ax1.set_title("Lift vs. Angle of Sideslip")
    ax1.set_title("Lift vs Angle of Attack")
    ax1.set_xlabel("Angle of Attack [°]")
    ax1.set_ylabel("Lift [N]")
    ax1.legend(title="Data Source", fontsize="small")

    # --- Plot 2: Drag vs. Angle of Attack ---
    ax2 = axes[0, 1]
    sns.scatterplot(
        data=combined_df,
        x="alpha_deg",
        y="drag",
        hue="plot_group",
        palette=palette,
        s=POINT_SIZE,
        alpha=POINT_ALPHA,
        ax=ax2,
        edgecolor=None,
    )
    # Overlay the analytical model for both experiments
    # for cfg in [config1, config2]:
    #     model_fwd = combined_df[
    #         (combined_df["source"] == cfg.label)
    #         & (combined_df["motion_phase"] == "Forward Motion")
    #     ].sort_values("beta_deg")
    #     ax2.plot(
    #         model_fwd["beta_deg"],
    #         model_fwd["mz_model"],
    #         color=model_colors[cfg.label],
    #         linewidth=MODEL_LINEWIDTH,
    #         label=f"{cfg.label} - Model",
    # )
    ax2.set_title("Drag vs Angle of Attack")
    ax2.set_xlabel("Angle of Attack [°]")
    ax2.set_ylabel("Drag [N]")
    ax2.legend(title="Data Source", fontsize="small")

    ax3 = axes[1, 0]
    sns.scatterplot(
        data=combined_df,
        x="beta_deg",
        y="lift",
        hue="plot_group",
        palette=palette,
        s=POINT_SIZE,
        alpha=POINT_ALPHA,
        ax=ax3,
        edgecolor=None,
    )

    ax3.set_title("Lift vs Angle of Sideslip")
    ax3.set_xlabel("Angle of Sideslip [°]")
    ax3.set_ylabel("Lift [N]")
    ax3.legend(title="Data Source", fontsize="small")

    ax4 = axes[1, 1]
    sns.scatterplot(
        data=combined_df,
        x="beta_deg",
        y="drag",
        hue="plot_group",
        palette=palette,
        s=POINT_SIZE,
        alpha=POINT_ALPHA,
        ax=ax4,
        edgecolor=None,
    )

    ax4.set_title("Drag vs Angle of Sideslip")
    ax4.set_xlabel("Angle of Sideslip [°]")
    ax4.set_ylabel("Drag [N]")
    ax4.legend(title="Data Source", fontsize="small")

    # --- Finalize and Save ---
    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    plt.savefig(output_file, dpi=200)
    print(f"\n✔ Plot saved successfully to '{output_file}'")
