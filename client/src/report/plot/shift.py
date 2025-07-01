from sys import argv

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from rich.status import Status
from scipy import signal

from .defs import *
from .lib.export import save_figure_tikz


# INPUT_1 = INPUT_PATH / "0001_high_samples.parquet"
# INPUT_2 = INPUT_PATH / "0002_high_samples.parquet"

INPUT_1 = INPUT_PATH / "0001_high_samples_10.parquet"
INPUT_2 = INPUT_PATH / "0002_high_samples_10.parquet"

COL_X = "time"
COL_Y = "fx"

OUTPUT_PATH = PLOT_PATH / "shift"

LINE_WIDTH = 1.0


def main():
    df1 = pd.read_parquet(INPUT_1)
    df2 = pd.read_parquet(INPUT_2)

    with Status("Computing cross-correlation"):
        calculated_shift, _ = find_time_shift_fast(df1, df2)

    if calculated_shift is not None:
        print(f"Calculated Time Shift: {1e9 * calculated_shift} ns")
        print(f"(A positive value means FILE_2 is delayed relative to FILE_1")

        # plot_comparison(df1, df2, calculated_shift)
        plot_results(df1, df2)


def find_time_shift(df1: pd.DataFrame, df2: pd.DataFrame):
    df1.sort_values(by=COL_X, inplace=True)
    df2.sort_values(by=COL_X, inplace=True)

    # 2. Resample data onto a common, uniform time grid
    # Determine the resampling frequency (use the finer of the two)
    dt1 = np.median(np.diff(df1[COL_X])).item()
    dt2 = np.median(np.diff(df2[COL_X])).item()
    resample_dt = min(dt1, dt2)

    # Create the common time grid
    t_start = min(df1[COL_X].min(), df2[COL_X].min())
    t_end = max(df1[COL_X].max(), df2[COL_X].max())
    common_time_grid = np.arange(t_start, t_end, resample_dt)

    # Interpolate both signals onto the common grid
    fx1_resampled = np.interp(common_time_grid, df1[COL_X], df1[COL_Y])
    fx2_resampled = np.interp(common_time_grid, df2[COL_X], df2[COL_Y])

    # 3. Compute cross-correlation
    # De-mean the signals to focus on shape, not absolute value
    s1 = fx1_resampled - fx1_resampled.mean()
    s2 = fx2_resampled - fx2_resampled.mean()

    # Use 'full' mode to check all possible overlaps
    correlation = np.correlate(s2, s1, mode="full")

    # 4. Find the lag (shift)
    # The index of the peak correlation gives the shift in number of samples
    lag_in_samples = np.argmax(correlation)

    # The center of the correlation array corresponds to zero lag.
    # We need to subtract this center index to find the actual shift.
    zero_lag_index = len(s1) - 1
    shift_in_samples = lag_in_samples - zero_lag_index

    # Convert shift from samples to time units
    time_shift = shift_in_samples * resample_dt

    return time_shift, correlation


def find_time_shift_fast(
    df1: pd.DataFrame, df2: pd.DataFrame, max_shift_s: float = 0.1
):

    # 1. Sort and Resample data onto a common, uniform time grid (same as before)
    df1 = df1.sort_values(by=COL_X)
    df2 = df2.sort_values(by=COL_X)

    dt1 = np.median(np.diff(df1[COL_X])).item()
    dt2 = np.median(np.diff(df2[COL_X])).item()
    resample_dt = min(dt1, dt2)

    # Use a more efficient way to create the common grid based on the reference signal (df1)
    # This prevents creating unnecessarily large arrays if signals have gaps
    common_time_grid = np.arange(df1[COL_X].min(), df1[COL_X].max(), resample_dt)

    fx1_resampled = np.interp(common_time_grid, df1[COL_X], df1[COL_Y])
    fx2_resampled = np.interp(common_time_grid, df2[COL_X], df2[COL_Y])

    # 2. De-mean the signals
    s1 = fx1_resampled - fx1_resampled.mean()
    s2 = fx2_resampled - fx2_resampled.mean()

    # 3. Compute cross-correlation using the much faster FFT method
    # This is the first key optimization.
    correlation = signal.correlate(s2, s1, mode="full", method="fft")

    # 4. Find the lag within the specified time window
    # This is the second key optimization.
    zero_lag_index = len(s1) - 1

    # Calculate the search window in number of samples
    max_shift_samples = int(max_shift_s / resample_dt)

    # Define the slice for our limited search
    search_start = max(0, zero_lag_index - max_shift_samples)
    search_end = min(len(correlation), zero_lag_index + max_shift_samples + 1)

    # Search for the peak *only* within this small window
    limited_correlation = correlation[search_start:search_end]
    lag_in_limited_window = np.argmax(limited_correlation)

    # The final lag index in the full correlation array
    lag_in_samples = search_start + lag_in_limited_window

    shift_in_samples = lag_in_samples - zero_lag_index
    time_shift = shift_in_samples * resample_dt

    return time_shift, correlation


def plot_comparison(df1: pd.DataFrame, df2: pd.DataFrame, calculated_shift: float):
    fig, axes = plt.subplots(2, 1, sharex=True)

    # Plot 1: Original Data
    axes[0].plot(
        df1[COL_X],
        df1[COL_Y],
        "o-",
        label="Dataset 1",
        alpha=0.7,
        linewidth=LINE_WIDTH,
    )

    axes[0].plot(
        df2[COL_X],
        df2[COL_Y],
        "o-",
        label="Dataset 2",
        alpha=0.7,
        linewidth=LINE_WIDTH,
    )

    axes[0].set_title("Original Data")
    axes[0].set_ylabel(COL_Y)
    axes[0].legend()
    axes[0].grid(True, linestyle="--", alpha=0.6)

    # Plot 2: Aligned Data
    # We shift Dataset 2 to align with Dataset 1
    axes[1].plot(df1[COL_X], df1[COL_Y], "o-", label="Dataset 1", alpha=0.7)
    axes[1].plot(
        df2[COL_X] - calculated_shift,
        df2[COL_Y],
        "o-",
        label=f"Dataset 2 (Shifted by {-calculated_shift:.2f} s)",
        alpha=0.7,
        color="green",
    )
    axes[1].set_title("Data Aligned using Calculated Shift")
    axes[1].set_xlabel(COL_X)
    axes[1].set_ylabel(COL_Y)
    axes[1].legend()
    axes[1].grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()


def plot_results(df1: pd.DataFrame, df2: pd.DataFrame):
    fig = plt.figure(figsize=TIKZ_SIZE)
    ax = fig.gca()

    idx1 = (df1[COL_X] >= 330) & (df1[COL_X] <= 331)
    idx2 = (df2[COL_X] >= 330) & (df2[COL_X] <= 331)

    df1[idx1].plot.line(x=COL_X, y=COL_Y, ax=ax, c="#ee6677", linewidth=LINE_WIDTH)
    df2[idx2].plot.line(x=COL_X, y=COL_Y, ax=ax, c="#4477aa", linewidth=LINE_WIDTH)

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Force X [N]")
    ax.legend().remove()

    if "--tikz" in argv:
        save_figure_tikz(OUTPUT_PATH)
        plt.close(fig)

    else:
        plt.show()


if __name__ == "__main__":
    main()
