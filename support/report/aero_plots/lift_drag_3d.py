from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import torch

from .network import SimpleFeedForwardNN

ATTITUDE_QUAT_COLUMNS = ["qx", "qy", "qz", "qw"]
ANGULAR_VELOCITY_COLUMNS = ["p", "q", "r"]
RELATIVE_VELOCITY_COLUMNS = ["u", "v", "w"]

DRONE_ACTUATOR_COLUMNS = [
    "drone/motor",
    "drone/left_wing",
    "drone/right_wing",
    "drone/elevator",
    "drone/rudder",
]

INPUT_COLUMNS = [
    *ATTITUDE_QUAT_COLUMNS,
    *ANGULAR_VELOCITY_COLUMNS,
    *RELATIVE_VELOCITY_COLUMNS,
    *DRONE_ACTUATOR_COLUMNS,
    "wind/speed",
]


# This function loads the trained model and generates predictions.
def evaluate_model(df, model_path):
    """Loads the model and adds its predictions to the DataFrame."""
    print("\nEvaluating model...")
    model = SimpleFeedForwardNN(16, 6, [128, 128, 64])
    model.load_state_dict(torch.load(model_path))
    model.eval()  # Set model to evaluation mode (important!)

    # Prepare input data
    inputs = torch.tensor(df[INPUT_COLUMNS].values, dtype=torch.float32)

    # Get predictions without tracking gradients
    with torch.no_grad():
        predictions = model(inputs)

    # Add predictions as new columns to the DataFrame
    df_eval = df.copy()
    df_eval["lift_pred"] = predictions[:, 0].numpy()
    df_eval["drag_pred"] = predictions[:, 1].numpy()

    print("Model evaluation complete. DataFrame augmented with predictions.")
    return df_eval


# --- Main Script Execution ---
if __name__ == "__main__":
    # --- 4. Load and Prepare Data ---
    # Define paths relative to the script location
    script_dir = Path(__file__).parent
    data_path = script_dir / "../data/250614_free-flight-extended/no-model.parquet"
    model_path = script_dir / "trained_model.pth"  # Path to save/load model weights

    df = pd.read_parquet(data_path)

    # Augment the DataFrame with degrees for plotting
    df["alpha_deg"] = np.rad2deg(df["alpha"])
    df["beta_deg"] = np.rad2deg(df["beta"])

    print("\nDataFrame head with new degree columns:")
    print(df.head())

    # --- 5. Train and Evaluate the Model ---
    # This will train the model only on the first run, then load from file.
    df_with_preds = evaluate_model(df, model_path)
    print("\nDataFrame head with model predictions:")
    print(df_with_preds.head())

    # --- 6. Create the 2x2 Subplots ---
    print("\nGenerating 2x2 grid of 3D plots...")
    fig, axes = plt.subplots(
        2,
        2,  # 2 rows, 2 columns
        figsize=(20, 18),
        subplot_kw={"projection": "3d"},  # Make all subplots 3D
    )
    fig.suptitle(
        "Aerodynamic Coefficients: Actual Data vs. PyTorch Model Predictions",
        fontsize=20,
    )

    # Common plotting arguments
    plot_kwargs = {
        "s": 1,  # Small point size
        "marker": ".",  # Thin marker
    }

    # --- Top Row: Original Data ---

    # Plot 1: Original Lift Coefficient (CL)
    ax1 = axes[0, 0]
    scatter_lift = ax1.scatter(
        df_with_preds["alpha_deg"],
        df_with_preds["beta_deg"],
        df_with_preds["lift"],
        c=df_with_preds["lift"],
        cmap="viridis",
        **plot_kwargs
    )
    ax1.set_title("Original Data: Lift Coefficient")
    ax1.set_xlabel("Alpha (degrees)")
    ax1.set_ylabel("Beta (degrees)")
    ax1.set_zlabel("Lift Coefficient (CL)")
    cbar_lift = fig.colorbar(scatter_lift, ax=ax1, shrink=0.6, aspect=20)
    cbar_lift.set_label("CL Value")

    # Plot 2: Original Drag Coefficient (CD)
    ax2 = axes[0, 1]
    scatter_drag = ax2.scatter(
        df_with_preds["alpha_deg"],
        df_with_preds["beta_deg"],
        df_with_preds["drag"],
        c=df_with_preds["drag"],
        cmap="plasma",
        **plot_kwargs
    )
    ax2.set_title("Original Data: Drag Coefficient")
    ax2.set_xlabel("Alpha (degrees)")
    ax2.set_ylabel("Beta (degrees)")
    ax2.set_zlabel("Drag Coefficient (CD)")
    cbar_drag = fig.colorbar(scatter_drag, ax=ax2, shrink=0.6, aspect=20)
    cbar_drag.set_label("CD Value")

    # --- Bottom Row: Model Predictions ---

    # Plot 3: Predicted Lift Coefficient (CL)
    ax3 = axes[1, 0]
    scatter_lift_pred = ax3.scatter(
        df_with_preds["alpha_deg"],
        df_with_preds["beta_deg"],
        df_with_preds["lift_pred"],
        c=df_with_preds["lift_pred"],
        cmap="viridis",
        **plot_kwargs
    )
    ax3.set_title("Model Prediction: Lift Coefficient")
    ax3.set_xlabel("Alpha (degrees)")
    ax3.set_ylabel("Beta (degrees)")
    ax3.set_zlabel("Predicted Lift (CL)")
    cbar_lift_pred = fig.colorbar(scatter_lift_pred, ax=ax3, shrink=0.6, aspect=20)
    cbar_lift_pred.set_label("Predicted CL Value")

    # Plot 4: Predicted Drag Coefficient (CD)
    ax4 = axes[1, 1]
    scatter_drag_pred = ax4.scatter(
        df_with_preds["alpha_deg"],
        df_with_preds["beta_deg"],
        df_with_preds["drag_pred"],
        c=df_with_preds["drag_pred"],
        cmap="plasma",
        **plot_kwargs
    )
    ax4.set_title("Model Prediction: Drag Coefficient")
    ax4.set_xlabel("Alpha (degrees)")
    ax4.set_ylabel("Beta (degrees)")
    ax4.set_zlabel("Predicted Drag (CD)")
    cbar_drag_pred = fig.colorbar(scatter_drag_pred, ax=ax4, shrink=0.6, aspect=20)
    cbar_drag_pred.set_label("Predicted CD Value")

    # Adjust layout and display the plots
    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    plt.show()
