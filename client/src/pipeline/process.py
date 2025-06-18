from pandas import DataFrame
import numpy as np
from scipy.linalg import norm
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation

from .config import *
from .dataframe import *
from .utils import *


def process_dataframe(df: DataFrame) -> None:
    """
    Applies a series of transformations to process/augment data.
    Modifies the DataFrame in-place.

    Order of operations:
    1. Corrects moments for the center of mass offset.
    2. Computes all kinematic quantities (pqr, uvw, alpha, beta).
    3. Computes aerodynamic forces (lift, drag, side_force).
    """

    correct_sweep(df)
    transform_wrench(df)
    add_body_frame_kinematics(df)
    add_aero_forces(df, Columns.BodyForce, Columns.AeroForces)

    if ANALYTICAL_MODELLING and has_drone_actuation(df):
        add_analytical_model(df)
        add_aero_forces(df, Columns.BodyForceModel, Columns.BodyMomentModel)


def correct_sweep(df: DataFrame) -> None:
    """
    Corrects the left wing sweep direction in the DataFrame, which is inverted for
    readability in the drone relay program.
    """

    if CORRECT_L_SWEEP and has_drone_actuation(df):
        drone_left_wing = Columns.DroneActuators[1]
        df[drone_left_wing] *= -1.0


def transform_wrench(df: DataFrame) -> None:
    """
    Transforms load cell moments to the center of mass. Modifies df in-place.
    Formula: M_new = M_old - r x F, where r is from old to new point.
    """

    forces = df[Columns.LoadForce].to_numpy()
    moment_correction = np.cross(np.array(COM_OFFSET_M), forces)

    # Rename columns from world frame to drone frame
    col_map = dict(
        zip(
            [*Columns.LoadForce, *Columns.LoadMoment],
            [*Columns.BodyForce, *Columns.BodyMoment],
        )
    )

    df.rename(columns=col_map, inplace=True)

    # Apply the moment transform correction
    df[Columns.BodyMoment] -= moment_correction


def add_body_frame_kinematics(df: DataFrame) -> None:

    df.sort_values(by=Columns.Time, inplace=True)
    df.reset_index(drop=True, inplace=True)

    times = df[Columns.Time].to_numpy()
    fs = 1.0 / np.mean(np.diff(times))

    # --- 1. Low-Pass Filter the Source Euler Angles ---

    # a) Get Euler angles and unwrap them to create a continuous signal
    euler_angles_rad = df[Columns.WorldRotation].to_numpy()
    unwrapped_euler = np.unwrap(euler_angles_rad, axis=0)

    # b) Design the Butterworth low-pass filter
    cutoff_hz = 10.0
    filter_order = 4

    # The butterworth function requires the cutoff frequency to be normalized by the Nyquist frequency (fs / 2)
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_hz / nyquist

    if 0 < normal_cutoff < 1:
        b, a = butter(filter_order, normal_cutoff, btype="low", analog=False)

        # c) Apply the filter using 'filtfilt' for zero phase shift (no time delay)
        unwrapped_euler = filtfilt(b, a, unwrapped_euler, axis=0)

    # --- 2. Calculate PQR using Quaternion Central Derivative ---

    # a) Create new, clean Rotation objects from the filtered Euler angles
    # Note: We can use the unwrapped angles directly; `from_euler` handles it correctly.
    rot = Rotation.from_euler(ROT_ANGLE_SEQ, unwrapped_euler)

    # b) Save the filtered quaternion representation
    df[Columns.Attitude] = rot.as_quat()

    # c) Apply the stable central difference method
    dt_central = times[2:] - times[:-2]
    delta_r = rot[2:] * rot[:-2].inv()  # World-frame difference
    rot_vecs_world = delta_r.as_rotvec()
    angular_velocity_world = rot_vecs_world / dt_central[:, np.newaxis]
    angular_velocity_body = rot[1:-1].inv().apply(angular_velocity_world)

    # d) Pad the result to match the original dataframe shape
    padded_pqr = np.pad(angular_velocity_body, ((1, 1), (0, 0)), "edge")
    df[Columns.AeroAngularVelocity] = padded_pqr

    # --- 3. Calculate UVW and Aero Angles using the SAME filtered data ---
    # This ensures all derived quantities are consistent.

    positions = df[Columns.WorldPosition].to_numpy()
    vel_drone_world = np.zeros_like(positions)
    vel_drone_world[1:-1] = (positions[2:] - positions[:-2]) / dt_central[:, np.newaxis]
    vel_drone_world[0] = (positions[1] - positions[0]) / (times[1] - times[0])
    vel_drone_world[-1] = (positions[-1] - positions[-2]) / (times[-1] - times[-2])

    wind_speeds = df[Columns.Wind].map(lookup_wind)
    wind_dir_norm = np.array([-1.0, 0.0, 0.0])
    vel_wind_world = wind_speeds.to_numpy()[:, np.newaxis] * wind_dir_norm

    vel_air_world = vel_drone_world - vel_wind_world
    vel_air_body = rot.inv().apply(vel_air_world)
    df[Columns.AeroVelocity] = vel_air_body

    u, v, w = vel_air_body.T
    total_airspeed = norm(vel_air_body, axis=1)

    epsilon = 1e-9
    alpha = np.arctan2(-w, u)
    beta = np.arcsin(v / (total_airspeed + epsilon))
    df[Columns.AeroAngles] = np.vstack([alpha, beta]).T


def add_aero_forces(
    df: DataFrame,
    force_cols: list[str],
    output_cols: list[str],
) -> None:
    """Computes lift, drag, and side force from body-frame forces and aero angles."""

    forces_body = df[force_cols].to_numpy()
    alpha, beta = df[Columns.AeroAngles].to_numpy().T

    rotation = Rotation.from_euler(
        "XYZ", -np.array([np.zeros_like(alpha), alpha, beta]).T
    )
    aero_forces = rotation.apply(forces_body)

    # Give a positive drag profile
    aero_forces[:, 0] = -aero_forces[:, 0]

    df[output_cols] = aero_forces


def add_analytical_model(df: DataFrame) -> None:
    """
    Computes the analytical model and applies a correctly designed low-pass
    Butterworth filter to the output forces and moments.
    """
    from .model import compute_analytical_forces

    positions = df[Columns.WorldPosition].to_numpy()
    attitudes = df[Columns.Attitude].to_numpy()
    angular_velocities = df[Columns.AeroAngularVelocity].to_numpy()
    relative_velocities = df[Columns.AeroVelocity].to_numpy()
    drone_actuators = df[Columns.DroneActuators].to_numpy()

    # Override with fake actuators for testing purposes
    # drone_actuators = np.repeat(np.array([-1, -1, -1, 0, 0]), df.shape[0]).reshape(-1, 5)

    num_rows = len(df)
    forces_list = []
    moments_list = []

    for i in range(num_rows):
        force, moment = compute_analytical_forces(
            positions[i],
            attitudes[i],
            angular_velocities[i],
            relative_velocities[i],
            drone_actuators[i],
        )
        forces_list.append(force)
        moments_list.append(moment)

    df[Columns.BodyForceModel] = forces_list
    df[Columns.BodyMomentModel] = moments_list

    add_aero_forces(df, Columns.BodyForceModel, Columns.AeroForcesModel)


def has_drone_actuation(df: DataFrame) -> bool:
    """Checks if the DataFrame contains drone actuator data."""

    return (
        all(col in df.columns for col in Columns.DroneActuators)
        and not df[Columns.DroneActuators].isnull().all().all()
    )
