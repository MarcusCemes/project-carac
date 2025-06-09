import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# --- Configuration ---
FILE_1 = Path("data/250609_repeatability/0005.parquet")
FILE_2 = Path("data/250609_repeatability/0041.parquet")
TIME_COLUMN = "time"
DATA_COLUMN = "fx"


def create_dummy_data(file1: Path, file2: Path, true_shift: float = 12.5):
    """
    Generates two noisy, time-shifted datasets for demonstration.
    """
    print("--- Creating dummy data ---")

    # 1. Create a base signal
    t_base = np.linspace(0, 100, 500)  # 500 points from 0 to 100 seconds
    # A signal with some interesting shape
    signal_base = (
        np.sin(t_base / 5) + np.cos(t_base / 2) * 0.5 + np.sin(t_base / 1.5) * 0.2
    )

    # 2. Create Dataset 1
    t1 = t_base + np.random.uniform(-0.05, 0.05, t_base.size)  # Add jitter to time
    t1.sort()
    fx1 = signal_base + np.random.normal(0, 0.2, t_base.size)  # Add noise to signal
    df1 = pd.DataFrame({TIME_COLUMN: t1, DATA_COLUMN: fx1})
    df1.to_parquet(file1)
    print(f"Created {file1} with {len(df1)} data points.")

    # 3. Create Dataset 2 (shifted and with different noise)
    t2 = t_base + true_shift + np.random.uniform(-0.05, 0.05, t_base.size)  # Add jitter
    t2.sort()
    fx2 = signal_base + np.random.normal(0, 0.2, t_base.size)  # Add different noise
    df2 = pd.DataFrame({TIME_COLUMN: t2, DATA_COLUMN: fx2})
    df2.to_parquet(file2)
    print(f"Created {file2} with a true shift of {true_shift} seconds.")
    print("-" * 27)
    return true_shift


def find_time_shift(file1: Path, file2: Path):
    """
    Reads two parquet files and finds the time shift between their signals.
    """
    # 1. Read and sort the data
    try:
        df1 = pd.read_parquet(file1).sort_values(by=TIME_COLUMN).reset_index(drop=True)
        df2 = pd.read_parquet(file2).sort_values(by=TIME_COLUMN).reset_index(drop=True)
    except FileNotFoundError as e:
        print(f"Error: {e}. One of the files was not found.")
        print(
            "Please ensure the Parquet files exist or run the script to generate dummy data."
        )
        return None, None, None, None

    # 2. Resample data onto a common, uniform time grid
    # This is essential because cross-correlation assumes evenly spaced data.

    # Determine the resampling frequency (use the finer of the two)
    dt1 = np.median(np.diff(df1[TIME_COLUMN]))
    dt2 = np.median(np.diff(df2[TIME_COLUMN]))
    resample_dt = min(dt1, dt2)

    # Create the common time grid
    t_start = min(df1[TIME_COLUMN].min(), df2[TIME_COLUMN].min())
    t_end = max(df1[TIME_COLUMN].max(), df2[TIME_COLUMN].max())
    common_time_grid = np.arange(t_start, t_end, resample_dt)

    # Interpolate both signals onto the common grid
    fx1_resampled = np.interp(common_time_grid, df1[TIME_COLUMN], df1[DATA_COLUMN])
    fx2_resampled = np.interp(common_time_grid, df2[TIME_COLUMN], df2[DATA_COLUMN])

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

    return time_shift, df1, df2, correlation


def plot_results(df1, df2, calculated_shift):
    """
    Visualizes the original and aligned data.
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Plot 1: Original Data
    axes[0].plot(df1[TIME_COLUMN], df1[DATA_COLUMN], "o-", label="Dataset 1", alpha=0.7)
    axes[0].plot(df2[TIME_COLUMN], df2[DATA_COLUMN], "o-", label="Dataset 2", alpha=0.7)
    axes[0].set_title("Original Data")
    axes[0].set_ylabel(DATA_COLUMN)
    axes[0].legend()
    axes[0].grid(True, linestyle="--", alpha=0.6)

    # Plot 2: Aligned Data
    # We shift Dataset 2 to align with Dataset 1
    axes[1].plot(df1[TIME_COLUMN], df1[DATA_COLUMN], "o-", label="Dataset 1", alpha=0.7)
    axes[1].plot(
        df2[TIME_COLUMN] - calculated_shift,
        df2[DATA_COLUMN],
        "o-",
        label=f"Dataset 2 (Shifted by {-calculated_shift:.2f} s)",
        alpha=0.7,
        color="green",
    )
    axes[1].set_title("Data Aligned using Calculated Shift")
    axes[1].set_xlabel(TIME_COLUMN)
    axes[1].set_ylabel(DATA_COLUMN)
    axes[1].legend()
    axes[1].grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # Step 1: Create dummy data for demonstration (optional)
    # If you have your own files, you can comment this line out.
    # true_shift = create_dummy_data(FILE_1, FILE_2)

    # Step 2: Find the time shift from the files
    calculated_shift, df1, df2, correlation = find_time_shift(FILE_1, FILE_2)

    if calculated_shift is not None:
        print("\n--- Analysis Results ---")
        # Note: The sign indicates the direction.
        # A positive shift means signal 2 occurs AFTER signal 1.
        print(f"Calculated Time Shift: {calculated_shift:.9f} seconds")
        print(
            f"(A positive value means '{FILE_2.name}' is delayed relative to '{FILE_1.name}')"
        )

        # Step 3: Plot the results for visual confirmation
        plot_results(df1, df2, calculated_shift)
