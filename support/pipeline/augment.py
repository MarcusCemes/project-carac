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
from .utils import *


def augment_dataframe(df: DataFrame) -> None:
    """
    Applies a series of transformations to augment the flight data DataFrame.
    Modifies the DataFrame in-place.

    Order of operations:
    1. Corrects moments for the center of mass offset.
    2. Computes all kinematic quantities (pqr, uvw, alpha, beta).
    3. Computes aerodynamic forces (lift, drag, side_force).
    """

    transform_wrench(df)
    add_body_frame_kinematics(df)
    add_aero_forces(df)


def transform_wrench(df: DataFrame) -> None:
    """
    Transforms load cell moments to the center of mass. Modifies df in-place.
    Formula: M_new = M_old - r x F, where r is from old to new point.
    """

    forces = df[LF_FORCE].values
    moment_correction = cross(array(COM_OFFSET_M), forces)

    # Rename columns from world frame to drone frame
    col_map = dict(zip([*LF_FORCE, *LF_MOMENT], [*DF_FORCE, *DF_MOMENT]))
    df.rename(columns=col_map, inplace=True)

    # Apply the moment correction
    df[DF_MOMENT] -= moment_correction


def add_body_frame_kinematics(df: DataFrame) -> None:
    """Computes body-frame kinematics (p,q,r), (u,v,w), and (alpha,beta)."""

    df.sort_values(by=TIME_COL, inplace=True)
    df.reset_index(drop=True, inplace=True)

    times = df[TIME_COL].values
    dt = diff(times)  # type: ignore

    # Create rotation objects for the drone's attitude
    rot = Rotation.from_euler(EULER_SEQ, df[WF_ATTITUDE].values)

    # 1. Calculate Body-Frame Angular Velocity (p, q, r)
    delta_r = rot[1:] * rot[:-1].inv()
    rot_vecs_world = delta_r.as_rotvec()
    angular_velocity_world = rot_vecs_world / dt[:, newaxis]
    angular_velocity_body = rot[:-1].inv().apply(angular_velocity_world)
    # Pad the first row to maintain shape
    df[DF_PQR] = vstack([angular_velocity_body[0], angular_velocity_body])

    # 2. Calculate Body-Frame Relative Velocity (u, v, w)
    #   a) Drone velocity in world frame
    positions = df[WF_POSITION].values
    vel_drone_world = zeros_like(positions)
    vel_drone_world[1:-1] = (positions[2:] - positions[:-2]) / (times[2:] - times[:-2])[
        :, newaxis
    ]
    vel_drone_world[0] = (positions[1] - positions[0]) / dt[0]
    vel_drone_world[-1] = (positions[-1] - positions[-2]) / dt[-1]

    #   b) Wind velocity in world frame
    wind_speeds = df[WIND_COL].map(WIND_VEL_LUT)
    wind_dir_norm = array([-1.0, 0.0, 0.0])  # Assumes headwind along world -X
    vel_wind_world = wind_speeds.values[:, newaxis] * wind_dir_norm

    #   c) Air velocity in world frame, then rotated to body frame
    vel_air_world = vel_drone_world - vel_wind_world
    vel_air_body = rot.inv().apply(vel_air_world)
    df[DF_UVW] = vel_air_body

    # 3. Calculate Aerodynamic Angles (alpha, beta)
    u, v, w = vel_air_body.T

    alpha = -arctan2(w, u)
    beta = arctan2(v, u)

    df[DF_AERO_ANGLES] = vstack([alpha, beta]).T


def add_aero_forces(df: DataFrame) -> None:
    """Computes lift, drag, and side force from body-frame forces and aero angles."""

    forces_body = df[DF_FORCE].values
    alpha, beta = df[DF_AERO_ANGLES].values.T

    rotation = Rotation.from_euler("xyz", -array([zeros_like(alpha), alpha, beta]).T)
    aero_forces = rotation.apply(forces_body)

    # Give a positive drag profile
    aero_forces[:, 0] = -aero_forces[:, 0]

    df[DF_AERO_FORCES] = aero_forces
