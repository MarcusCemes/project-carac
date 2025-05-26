import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

sns.set_theme()


def main():
    drone_wind = pd.read_parquet("data/drone_wind.parquet")
    drone_no_wind = pd.read_parquet("data/drone_no_wind.parquet")
    mount_wind = pd.read_parquet("data/mount_wind.parquet")
    mount_no_wind = pd.read_parquet("data/mount_no_wind.parquet")

    df = pd.DataFrame(
        {
            "time": drone_wind["time"],
            "load_cell/fy": (drone_wind["load_cell/fy"] - drone_no_wind["load_cell/fy"])
            - (mount_wind["load_cell/fy"] - mount_no_wind["load_cell/fy"]),
        }
    )

    # Plotting the data
    fig, axes = plt.subplots(1, 4, figsize=(12, 4), sharex=True)
    dataframes = [drone_wind, drone_no_wind, mount_wind, mount_no_wind]
    titles = ["DW", "DNW", "MW", "MNW"]

    for ax, df, title in zip(axes, dataframes, titles):
        ax.plot(df["time"], df["load_cell/fy"], marker="o")
        ax.set_title(title)
        ax.set_xlabel("Time")
        ax.set_ylabel("Load Cell / fy")

    # Large plot for the fifth dataframe
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(df["time"], df["load_cell/fy"], marker="o", color="red")
    ax.set_title("E = (A-B) - (C-D)")
    ax.set_xlabel("Time")
    ax.set_ylabel("Load Cell / fy")

    plt.show()


if __name__ == "__main__":
    main()
