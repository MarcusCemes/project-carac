from dataclasses import dataclass

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from pipeline.dataframe import Columns

from ..common import find_experiment
from ..defs import *


@dataclass
class Opts:
    x: str
    xlabel: str


# FILE = find_experiment("axis-uncoupled", 13)
# FILE = find_experiment("plunge", 14)
# FILE = find_experiment("attack-rotations", 28)  # roll
FILE = find_experiment("attack-rotations", 31)  # yaw
df = pd.read_parquet(FILE)


import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from matplotlib.widgets import Slider
import pandas as pd


COLS = Columns.Attitude

# Replace attitude with motion capture data
# MOCAP_COLS = ["track/indUAV/roll", "track/indUAV/pitch", "track/indUAV/yaw"]
# mocap_rot = Rotation.from_euler("xyz", df[MOCAP_COLS].values)
# df[Columns.Attitude] = mocap_rot.as_quat()


class QuaternionVisualizer:
    def __init__(self, df, time_col="time"):
        """
        Interactive quaternion attitude visualizer with slider

        Parameters:
        df: pandas DataFrame with quaternion data
        quat_cols: list of column names for [w, x, y, z] quaternion components
        time_col: column name for time data
        """
        self.df = df
        self.time_col = time_col

        # Convert quaternions to rotation objects
        self.rotations = Rotation.from_quat(df[COLS].values)

        # Setup the figure
        self.fig = plt.figure(figsize=(15, 10))

        # Main 3D plot
        self.ax3d: Axes3D = self.fig.add_subplot(221, projection="3d")  # type: ignore

        # Quaternion components plot
        self.ax_quat = self.fig.add_subplot(222)

        # Euler angles plot
        self.ax_euler = self.fig.add_subplot(223)

        # Angular velocity plot (if available)
        self.ax_info = self.fig.add_subplot(224)

        # Create slider
        plt.subplots_adjust(bottom=0.15)
        ax_slider = plt.axes((0.1, 0.05, 0.8, 0.03))
        self.slider = Slider(
            ax_slider, "Time Index", 0, len(df) - 1, valinit=0, valfmt="%d"
        )

        # Initialize plots
        self.setup_plots()
        self.slider.on_changed(self.update_plots)

        # Initial plot
        self.update_plots(0)

    def setup_plots(self):
        """Setup static elements of all plots"""

        # 3D plot setup
        self.ax3d.set_xlim((-2, 2))
        self.ax3d.set_ylim((-2, 2))
        self.ax3d.set_zlim((-2, 2))
        self.ax3d.set_xlabel("X")
        self.ax3d.set_ylabel("Y")
        self.ax3d.set_zlabel("Z")
        self.ax3d.set_title("3D Orientation Frame")

        # Create axis vectors (will be updated)
        self.x_axis = self.ax3d.quiver(
            0,
            0,
            0,
            1,
            0,
            0,
            color="red",
            arrow_length_ratio=0.1,
            linewidth=3,
            label="X",
        )
        self.y_axis = self.ax3d.quiver(
            0,
            0,
            0,
            0,
            1,
            0,
            color="green",
            arrow_length_ratio=0.1,
            linewidth=3,
            label="Y",
        )
        self.z_axis = self.ax3d.quiver(
            0,
            0,
            0,
            0,
            0,
            1,
            color="blue",
            arrow_length_ratio=0.1,
            linewidth=3,
            label="Z",
        )

        # Add reference frame (world coordinates)
        self.ax3d.quiver(
            0, 0, 0, 0.5, 0, 0, color="red", alpha=0.3, arrow_length_ratio=0.1
        )
        self.ax3d.quiver(
            0, 0, 0, 0, 0.5, 0, color="green", alpha=0.3, arrow_length_ratio=0.1
        )
        self.ax3d.quiver(
            0, 0, 0, 0, 0, 0.5, color="blue", alpha=0.3, arrow_length_ratio=0.1
        )

        self.ax3d.legend()

        # Quaternion components plot
        time_data = (
            self.df[self.time_col]
            if self.time_col in self.df.columns
            else range(len(self.df))
        )
        colors = ["black", "red", "green", "blue"]

        for i, (col, color) in enumerate(zip(COLS, colors)):
            self.ax_quat.plot(
                time_data,
                self.df[col],
                color=color,
                alpha=0.7,
                linewidth=1,
                label=col,
            )

        self.ax_quat.set_title("Quaternion Components")
        self.ax_quat.set_ylabel("Value")
        self.ax_quat.legend()
        self.ax_quat.grid(True, alpha=0.3)
        self.ax_quat.set_ylim(-1.1, 1.1)

        # Current time indicator line (will be updated)
        self.quat_vline = self.ax_quat.axvline(
            x=0, color="orange", linewidth=2, alpha=0.8
        )

        # Euler angles plot
        euler_angles = self.rotations.as_euler("xyz", degrees=True)
        euler_labels = ["Roll (X)", "Pitch (Y)", "Yaw (Z)"]
        euler_colors = ["red", "green", "blue"]

        for i, (angles, color, label) in enumerate(
            zip(euler_angles.T, euler_colors, euler_labels)
        ):
            self.ax_euler.plot(
                time_data, angles, color=color, alpha=0.7, label=label, linewidth=1
            )

        self.ax_euler.set_title("Euler Angles (XYZ)")
        self.ax_euler.set_ylabel("Degrees")
        self.ax_euler.legend()
        self.ax_euler.grid(True, alpha=0.3)

        self.euler_vline = self.ax_euler.axvline(
            x=0, color="orange", linewidth=2, alpha=0.8
        )

        # Info panel
        self.ax_info.axis("off")
        self.info_text = self.ax_info.text(
            0.1,
            0.9,
            "",
            transform=self.ax_info.transAxes,
            fontsize=10,
            verticalalignment="top",
            fontfamily="monospace",
        )

    def update_plots(self, val):
        """Update all plots based on slider value"""
        idx = int(self.slider.val)

        # Get current rotation matrix
        R = self.rotations[idx].as_matrix()

        # Update 3D orientation vectors
        # Remove old quivers and create new ones (matplotlib limitation)
        self.x_axis.remove()
        self.y_axis.remove()
        self.z_axis.remove()

        # Create new quiver plots with updated orientations
        self.x_axis = self.ax3d.quiver(
            0,
            0,
            0,
            R[0, 0],
            R[1, 0],
            R[2, 0],
            color="red",
            arrow_length_ratio=0.1,
            linewidth=3,
            label="X",
        )
        self.y_axis = self.ax3d.quiver(
            0,
            0,
            0,
            R[0, 1],
            R[1, 1],
            R[2, 1],
            color="green",
            arrow_length_ratio=0.1,
            linewidth=3,
            label="Y",
        )
        self.z_axis = self.ax3d.quiver(
            0,
            0,
            0,
            R[0, 2],
            R[1, 2],
            R[2, 2],
            color="blue",
            arrow_length_ratio=0.1,
            linewidth=3,
            label="Z",
        )

        # Update time indicators
        if self.time_col in self.df.columns:
            current_time = self.df[self.time_col].iloc[idx]
            self.quat_vline.set_xdata([current_time, current_time])
            self.euler_vline.set_xdata([current_time, current_time])
        else:
            self.quat_vline.set_xdata([idx, idx])
            self.euler_vline.set_xdata([idx, idx])

        # Update info panel
        quat = self.df[COLS].iloc[idx]
        euler = self.rotations[idx].as_euler("xyz", degrees=True)
        quat_norm = np.linalg.norm(quat.values)

        info_str = f"""Current State (Index: {idx})

Quaternion:
  w: {quat.iloc[0]:7.4f}
  x: {quat.iloc[1]:7.4f}
  y: {quat.iloc[2]:7.4f}
  z: {quat.iloc[3]:7.4f}
  |q|: {quat_norm:.6f}

Euler Angles (deg):
  Roll:  {euler[0]:7.2f}°
  Pitch: {euler[1]:7.2f}°
  Yaw:   {euler[2]:7.2f}°"""

        if self.time_col in self.df.columns:
            info_str += f"\n\nTime: {self.df[self.time_col].iloc[idx]:.3f}"

        self.info_text.set_text(info_str)

        # Update plot
        self.fig.canvas.draw()


def visualize_quaternion_attitude(df, time_col="time"):
    """
    Create interactive quaternion visualizer

    Usage:
    # Assuming your DataFrame has columns: 'time', 'qw', 'qx', 'qy', 'qz'
    visualizer = visualize_quaternion_attitude(df)
    plt.show()

    # Or with custom column names:
    visualizer = visualize_quaternion_attitude(df,
                                             quat_cols=['q0', 'q1', 'q2', 'q3'],
                                             time_col='timestamp')
    """
    visualizer = QuaternionVisualizer(df, time_col)
    return visualizer


# Example usage with sample data:
if __name__ == "__main__":

    # Create visualizer
    viz = visualize_quaternion_attitude(df)
    plt.show()
