from pandas import DataFrame
import numpy as np
from scipy.linalg import norm
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation

from carac.helpers import Vec3

from .config import *
from .data.datasets import Loader
from .dataframe import *
from .utils import *


def process_dataframe(df: DataFrame, loader: Loader) -> None:
    """
    Applies a series of transformations to process/augment data.
    Modifies the DataFrame in-place.

    Order of operations:
    1. Corrects moments for the center of mass offset.
    2. Computes all kinematic quantities (pqr, uvw, alpha, beta).
    3. Computes aerodynamic forces (lift, drag, side_force).
    """

    transform_wrench(df, loader.offset)
    add_body_frame_kinematics(df, loader.wind)
    add_aero_forces(df, Columns.BodyForce, Columns.AeroForces)

    if ANALYTICAL_MODELLING:
        add_analytical_model(df)
        add_aero_forces(df, Columns.BodyForceModel, Columns.BodyMomentModel)


def transform_wrench(df: DataFrame, offset: Vec3) -> None:
    """
    Transforms load cell moments to the center of mass. Modifies df in-place.
    Formula: M_new = M_old - r x F, where r is from old to new point.
    """

    forces = df[Columns.LoadForce].to_numpy()
    moment_correction = np.cross(np.array(offset), forces)

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


def add_body_frame_kinematics(df: DataFrame, wind: WindLut) -> None:
    """
    Computes body-frame kinematics using a Savitzky-Golay filter for smooth
    derivatives of position and orientation.

    This version differentiates the quaternions directly to avoid noise amplification
    issues associated with Euler angles.
    """
    df.sort_values(by=Columns.Time, inplace=True)
    df.reset_index(drop=True, inplace=True)

    times = df[Columns.Time].to_numpy()

    # --- Savitzky-Golay Filter Parameters (tune these as needed) ---
    SAVGOL_WINDOW = 99
    SAVGOL_POLYORDER = 3

    # --- 1. Calculate PQR by Differentiating Quaternions ---

    euler_angles_rad = df[Columns.WorldRotation].to_numpy()
    unwrapped_euler = np.unwrap(euler_angles_rad, axis=0)
    rot = Rotation.from_euler(ROT_ANGLE_SEQ, unwrapped_euler)
    quats = rot.as_quat()

    df[Columns.Attitude] = quats

    # a) Get quaternion array and ensure continuity
    for i in range(1, len(quats)):
        # If the dot product is negative, the quaternions are pointing in
        # "opposite" directions, so we flip the sign of the current one.
        if np.dot(quats[i - 1], quats[i]) < 0:
            quats[i] *= -1

    # b) Calculate a stable, average time step
    dt = np.mean(np.diff(times)).item()

    # c) Calculate the time derivative of each quaternion component using SavGol
    quat_derivatives = savgol_filter(
        quats,
        window_length=SAVGOL_WINDOW,
        polyorder=SAVGOL_POLYORDER,
        deriv=1,
        delta=dt,
        axis=0,
        mode="interp",
    )

    # d) Convert quaternion derivatives to body-frame angular velocity (p, q, r)
    # The formula is omega_body = 2 * H(q)^T * q_dot
    # where H(q) is a specific matrix and scipy stores q as [x, y, z, w]
    qx, qy, qz, qw = quats.T
    qx_dot, qy_dot, qz_dot, qw_dot = quat_derivatives.T

    # This is the explicit formula for the conversion
    p = 2 * (-qx * qw_dot + qw * qx_dot - qz * qy_dot + qy * qz_dot)
    q = 2 * (-qy * qw_dot + qz * qx_dot + qw * qy_dot - qx * qz_dot)
    r = 2 * (-qz * qw_dot - qy * qx_dot + qx * qy_dot + qw * qz_dot)

    angular_velocity_body = np.vstack([p, q, r]).T
    df[Columns.AeroAngularVelocity] = angular_velocity_body

    # --- 2. Calculate UVW and Aero Angles ---

    positions = df[Columns.WorldPosition].to_numpy()

    vel_drone_world = savgol_filter(
        positions,
        window_length=SAVGOL_WINDOW,
        polyorder=SAVGOL_POLYORDER,
        deriv=1,
        delta=dt,
        axis=0,
        mode="interp",
    )

    wind_speeds = df[Columns.Wind].map(wind.lookup)
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

    alpha, beta = df[Columns.AeroAngles].to_numpy().T
    body_to_wind_rotation = Rotation.from_euler("zy", np.vstack([beta, alpha]).T)

    forces_body = df[force_cols].to_numpy()
    forces_wind_frame = body_to_wind_rotation.inv().apply(forces_body)

    drag = -forces_wind_frame[:, 0]
    side_force = forces_wind_frame[:, 1]
    lift = forces_wind_frame[:, 2]

    df[output_cols] = np.vstack([drag, side_force, lift]).T


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
