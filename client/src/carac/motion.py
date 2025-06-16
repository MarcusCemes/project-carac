from dataclasses import dataclass
import math

import noise

from orchestrator.helpers import Point


@dataclass
class TrajectoryPoint:
    time: float
    pose: Point
    velocity_t: float
    velocity_r: float


class BaseTrajectoryGenerator:
    """
    Abstract base class for trajectory generators.
    Handles common functionality like limit checking and output formatting.
    """

    def __init__(self, limits):
        self.pose_order = ["x", "y", "z", "rx", "ry", "rz"]

        if not all(axis in limits for axis in self.pose_order):
            raise ValueError(f"Limits must be provided for all axes: {self.pose_order}")
        self.limits = limits

    def _calculate_axis_kinematics(self, t, axis_name):
        """
        This method must be implemented by subclasses.
        It should return the position and velocity for a single axis at time t.
        """
        raise NotImplementedError("Subclasses must implement this method.")

    def get_trajectory_point(self, t: float) -> TrajectoryPoint:
        """
        Calculates and returns the full 6-DOF pose and velocity at a given time t.
        """
        pose = []
        velocity = []

        for axis in self.pose_order:
            p, v = self._calculate_axis_kinematics(t, axis)

            pose.append(p)
            velocity.append(v)

        velocity_t = math.sqrt(sum(v**2 for v in velocity[0:3]))
        velocity_r = math.sqrt(sum(v**2 for v in velocity[3:6]))

        return TrajectoryPoint(
            time=t,
            pose=Point(*pose),
            velocity_t=velocity_t,
            velocity_r=velocity_r,
        )


class SineTrajectoryGenerator(BaseTrajectoryGenerator):
    """
    Generates trajectories by summing sine waves, automatically scaled to fit within defined limits.
    """

    def __init__(self, limits, motion_params):
        super().__init__(limits)
        self.motion_params = motion_params
        self._scaling_factors = self._precompute_scaling()

    def _precompute_scaling(self):
        """Pre-calculates scaling factors to map sine sums to the defined limits."""
        factors = {}
        for axis in self.pose_order:
            min_lim, max_lim = self.limits[axis]
            center = (max_lim + min_lim) / 2.0
            half_range = (max_lim - min_lim) / 2.0

            # Sum of amplitudes determines the max output of the sine sum
            max_theoretical_amp = sum(
                comp["amp"] for comp in self.motion_params.get(axis, [])
            )
            if max_theoretical_amp == 0:
                max_theoretical_amp = 1.0  # Avoid division by zero for static axes

            factors[axis] = {
                "center": center,
                "scale": half_range / max_theoretical_amp,
            }
        return factors

    def _calculate_axis_kinematics(self, t, axis_name):
        """Calculates position and velocity using analytical derivatives of sines."""
        position_sum = 0.0
        velocity_sum = 0.0

        if axis_name in self.motion_params:
            for component in self.motion_params[axis_name]:
                amp = component["amp"]
                freq = component["freq"]
                phase = component.get("phase", 0)

                omega = 2 * math.pi * freq

                position_sum += amp * math.sin(omega * t + phase)
                velocity_sum += amp * omega * math.cos(omega * t + phase)

        # Apply pre-computed scaling to fit within limits
        factors = self._scaling_factors[axis_name]
        final_position = factors["center"] + position_sum * factors["scale"]
        final_velocity = velocity_sum * factors["scale"]

        return final_position, final_velocity


class PerlinTrajectoryGenerator(BaseTrajectoryGenerator):
    """
    Generates trajectories using Perlin noise, mapped to defined limits.
    Velocity is calculated using a numerical derivative.
    """

    def __init__(self, limits, motion_params, h=1e-6):
        super().__init__(limits)
        if noise is None:
            raise RuntimeError("'noise' library is not installed.")
        self.motion_params = motion_params
        self.h = h  # Small step for numerical differentiation

    def _get_raw_noise_pos(self, t, axis_name):
        """Calculates the raw, unscaled Perlin noise value for an axis."""
        params = self.motion_params.get(axis_name)
        if not params:
            return 0.0

        return noise.pnoise1(
            t * params["freq"] + params["seed"], octaves=params["octaves"]
        )

    def _calculate_axis_kinematics(self, t, axis_name):
        """Calculates position and velocity using numerical differentiation."""
        min_lim, max_lim = self.limits[axis_name]
        center = (max_lim + min_lim) / 2.0
        half_range = (max_lim - min_lim) / 2.0

        if half_range == 0:
            return center, 0.0

        # --- Position Calculation ---
        # Perlin noise is in [-1, 1], so we map it to the desired range
        raw_pos = self._get_raw_noise_pos(t, axis_name)
        final_position = center + raw_pos * half_range

        # --- Velocity Calculation (Numerical Derivative) ---
        # v(t) ≈ (pos(t + h) - pos(t - h)) / (2h)
        raw_pos_h_plus = self._get_raw_noise_pos(t + self.h, axis_name)
        raw_pos_h_minus = self._get_raw_noise_pos(t - self.h, axis_name)

        # The derivative of the raw noise signal
        raw_velocity = (raw_pos_h_plus - raw_pos_h_minus) / (2 * self.h)

        # Scale the velocity by the same factor as the position range
        final_velocity = raw_velocity * half_range

        return final_position, final_velocity
