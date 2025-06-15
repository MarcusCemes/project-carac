# from numpy import concatenate
# from numpy.typing import NDArray

# from .dyn_casadi_mod_ele_dot_3D import IndoorUAV3D


# _model = IndoorUAV3D()


# def compute_analytical_forces(
#     position: NDArray,
#     attitude: NDArray,
#     angular_velocity: NDArray,
#     relative_velocity: NDArray,
#     drone_actuators: NDArray,
# ):
#     global _model

#     # vel_norm, q, elevator_ang, area, ac_chord, aoa, sweep_angle_l, sweep_angle_r
#     assert attitude.shape == (4,)
#     assert angular_velocity.shape == (3,)
#     assert relative_velocity.shape == (3,)
#     assert drone_actuators.shape == (5,)

#     input = correct_input(drone_actuators)

#     state = concatenate(
#         [
#             position,
#             relative_velocity,
#             attitude,
#             angular_velocity,
#             input[0:1],
#             input[1],
#             0.0,  # internal state
#             input[2],
#             0.0,  # internal state
#             input[3:],
#         ]
#     )

#     assert state.shape == (20,)

#     return _model.force_moment_cg_total(state, input)


from numpy import array, concatenate, zeros, ndarray as NDArray

from .dyn_casadi_mod_ele_dot_3D import IndoorUAV3D

_model = IndoorUAV3D()


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

    state = concatenate(
        [
            position,
            relative_velocity,
            attitude,
            angular_velocity,
            zeros(7),
        ]
    )

    assert state.shape == (20,)

    actuation = correct_input(actuation)

    F_casadi, M_casadi = _model.force_moment_cg_total(state, actuation)

    return array(F_casadi).flatten(), array(M_casadi).flatten()


def correct_input(input: NDArray) -> NDArray:
    input = input.copy()
    input[0] = 0.5 * input[0] + 0.5  # Normalize motor speed to [0, 1]

    return input
