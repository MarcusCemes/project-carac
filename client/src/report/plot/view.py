import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from pipeline.dataframe import Columns

from .common import find_experiment
from .defs import *


MOCAP_COLS = ["track/indUAV/roll", "track/indUAV/pitch", "track/indUAV/yaw"]

# path = find_experiment("axis-uncoupled", 16)  # pitch,
path = find_experiment("attack-rotations", 24)  # roll
# path = find_experiment("attack-rotations", 26)  # yaw


columns = [
    ["lift", "lift_model"],
    ["drag", "drag_model"],
    Columns.WorldRotation + Columns.AeroAngles,
    Columns.BodyForce + Columns.BodyForceModel,
    Columns.AeroVelocity,
    Columns.AeroAngularVelocity,
]

df = pd.read_parquet(path)

df[Columns.WorldRotation] = np.rad2deg(df[Columns.WorldRotation])
df[Columns.AeroAngles] = np.rad2deg(df[Columns.AeroAngles])
# df[MOCAP_COLS] = np.rad2deg(df[MOCAP_COLS])

fig, axs = plt.subplots(3, 2)

for columns, ax in zip(columns, (ax for axs in axs for ax in axs)):
    df.plot(x="time", y=columns, ax=ax)

plt.show()
