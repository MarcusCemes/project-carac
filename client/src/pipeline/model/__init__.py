import numpy as np
from numpy.typing import NDArray

from .dyn_casadi_mod_ele_dot_3D import IndoorUAV3D

_model = IndoorUAV3D()

ACTUATION_LIMITS = [
    (0.05, 1.0),
    (-1.0, 0.5),
    (-1.0, 0.5),
    (-1.0, 1.0),
    (-1.0, 1.0),
]


def compute_analytical_forces(
    position: NDArray,
    attitude: NDArray,
    angular_velocity: NDArray,
    relative_velocity: NDArray,
    actuation: NDArray,
) -> tuple[NDArray, NDArray]:

    global _model

    assert position.shape == (3,)
    assert attitude.shape == (4,)
    assert angular_velocity.shape == (3,)
    assert relative_velocity.shape == (3,)
    assert actuation.shape == (5,)

    validate_actuation(actuation)

    omega = _model.motor_throttle_to_omega(actuation[0])
    drone_state = np.array([omega, actuation[1], 0, actuation[2], 0, 0, 0])

    state = np.concatenate(
        [position, relative_velocity, attitude, angular_velocity, drone_state]
    )

    assert state.shape == (20,)

    F_casadi, M_casadi = _model.force_moment_cg_total(state, actuation)

    return np.array(F_casadi).flatten(), np.array(M_casadi).flatten()


def validate_actuation(actuation: NDArray) -> None:
    if not all(
        min_limit <= value <= max_limit
        for value, (min_limit, max_limit) in zip(actuation, ACTUATION_LIMITS)
    ):
        raise ValueError(
            f"Actuation values {actuation} are out of bounds. "
            f"Expected limits: {ACTUATION_LIMITS}"
        )
