from pandas import DataFrame
from numpy import (
    arctan2,
    array,
    cross,
    diff,
    newaxis,
    vstack,
    zeros_like,
)
from scipy.spatial.transform import Rotation

from .defs import *
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
    add_aero_forces(df, Columns.DroneForce, Columns.AeroForces)

    if ANALYTICAL_MODELLING and has_drone_actuation(df):
        add_analytical_model(df)
        add_aero_forces(df, Columns.DroneForceModel, Columns.DroneMomentModel)


def correct_sweep(df: DataFrame) -> None:
    """
    Corrects the left wing sweep direction in the DataFrame, which is inverted for
    readability in the drone relay program.
    """

    if CORRECT_L_SWEEP:
        drone_left_wing = Columns.DroneActuators[1]
        df[drone_left_wing] *= -1.0


def transform_wrench(df: DataFrame) -> None:
    """
    Transforms load cell moments to the center of mass. Modifies df in-place.
    Formula: M_new = M_old - r x F, where r is from old to new point.
    """

    forces = df[Columns.LoadForce].to_numpy()
    moment_correction = cross(array(COM_OFFSET_M), forces)

    # Rename columns from world frame to drone frame
    col_map = dict(
        zip(
            [*Columns.LoadForce, *Columns.LoadMoment],
            [*Columns.DroneForce, *Columns.DroneMoment],
        )
    )

    df.rename(columns=col_map, inplace=True)

    # Apply the moment transform correction
    df[Columns.DroneMoment] -= moment_correction


def add_body_frame_kinematics(df: DataFrame) -> None:
    """Computes body-frame kinematics (p,q,r), (u,v,w), and (alpha,beta)."""

    df.sort_values(by=Columns.Time, inplace=True)
    df.reset_index(drop=True, inplace=True)

    times = df[Columns.Time].to_numpy()
    dt = diff(times)  # type: ignore

    # Create rotation objects for the drone's attitude
    rot = Rotation.from_euler(ROT_ANGLE_SEQ, df[Columns.RobotRot].to_numpy())

    # Save the quaternion representation of the drone's attitude (x, y, z, w)
    df[Columns.Attitude] = rot.as_quat(scalar_first=False)

    # 1. Calculate Body-Frame Angular Velocity (p, q, r)
    delta_r = rot[1:] * rot[:-1].inv()
    rot_vecs_world = delta_r.as_rotvec()
    angular_velocity_world = rot_vecs_world / dt[:, newaxis]
    angular_velocity_body = rot[:-1].inv().apply(angular_velocity_world)

    # Pad the first row to maintain shape
    df[Columns.AeroAngularVelocity] = vstack(
        [angular_velocity_body[0], angular_velocity_body]
    )

    # 2. Calculate Body-Frame Relative Velocity (u, v, w)
    #   a) Drone velocity in world frame
    positions = df[Columns.RobotPos].to_numpy()
    vel_drone_world = zeros_like(positions)
    vel_drone_world[1:-1] = (positions[2:] - positions[:-2]) / (times[2:] - times[:-2])[
        :, newaxis
    ]
    vel_drone_world[0] = (positions[1] - positions[0]) / dt[0]
    vel_drone_world[-1] = (positions[-1] - positions[-2]) / dt[-1]

    #   b) Wind velocity in world frame
    wind_speeds = df[Columns.Wind].map(WindSpeedLut)
    wind_dir_norm = array([-1.0, 0.0, 0.0])  # Assumes headwind along world -X
    vel_wind_world = wind_speeds.to_numpy()[:, newaxis] * wind_dir_norm

    #   c) Air velocity in world frame, then rotated to body frame
    vel_air_world = vel_drone_world - vel_wind_world
    vel_air_body = rot.inv().apply(vel_air_world)
    df[Columns.AeroVelocity] = vel_air_body

    # 3. Calculate Aerodynamic Angles (alpha, beta)
    u, v, w = vel_air_body.T

    alpha = -arctan2(w, u)
    beta = arctan2(v, u)

    df[Columns.AeroAngles] = vstack([alpha, beta]).T


def add_aero_forces(
    df: DataFrame,
    force_cols: list[str],
    output_cols: list[str],
) -> None:
    """Computes lift, drag, and side force from body-frame forces and aero angles."""

    forces_body = df[force_cols].to_numpy()
    alpha, beta = df[Columns.AeroAngles].to_numpy().T

    rotation = Rotation.from_euler("xyz", -array([zeros_like(alpha), alpha, beta]).T)
    aero_forces = rotation.apply(forces_body)

    # Give a positive drag profile
    aero_forces[:, 0] = -aero_forces[:, 0]

    df[output_cols] = aero_forces


def add_analytical_model(df: DataFrame) -> None:
    """
    Computes the analytical model of the drone's aerodynamic forces for each row
    in the DataFrame by calling a non-vectorized function.
    """
    # Import the non-vectorized function that works on single rows
    from .model import compute_analytical_forces

    # --- 1. Extract the data from the DataFrame into 2D NumPy arrays ---
    # This is more efficient than accessing the DataFrame in a loop.
    positions = df[Columns.RobotPos].to_numpy()
    attitudes = df[Columns.Attitude].to_numpy()
    angular_velocities = df[Columns.AeroAngles].to_numpy()
    relative_velocities = df[Columns.AeroVelocity].to_numpy()
    drone_actuators = df[Columns.DroneActuators].to_numpy()

    # Get the number of rows to process
    num_rows = len(df)

    # --- 2. Prepare empty lists to store the results from each row ---
    forces_list = []
    moments_list = []

    # print(f"Applying analytical model to {num_rows} data points...")

    # --- 3. Loop through each row index ---
    for i in range(num_rows):
        # Call the non-vectorized function with the data for the current row (i)
        force, moment = compute_analytical_forces(
            positions[i],
            attitudes[i],
            angular_velocities[i],
            relative_velocities[i],
            drone_actuators[i],
        )

        # Append the results to the lists
        forces_list.append(force)
        moments_list.append(moment)

    # print("Computation complete.")

    # --- 4. Assign the collected results back to the DataFrame ---
    # The lists of 1D arrays are converted to 2D NumPy arrays and then
    # assigned to the new columns. Pandas handles the assignment correctly.
    df[Columns.DroneForceModel] = forces_list
    df[Columns.DroneMomentModel] = moments_list


def has_drone_actuation(df: DataFrame) -> bool:
    """Checks if the DataFrame contains drone actuator data."""

    return (
        all(col in df.columns for col in Columns.DroneActuators)
        and not df[Columns.DroneActuators].isnull().all().all()
    )
