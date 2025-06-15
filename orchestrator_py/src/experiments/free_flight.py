from math import sqrt
from threading import Thread
from time import sleep

from rich.status import Status

from orchestrator.orchestrator import Orchestrator
from orchestrator.instructions import *
from orchestrator.helpers import *
from orchestrator.motion import BaseTrajectoryGenerator, SineTrajectoryGenerator
from orchestrator.tui import ExperimentStatus, ExperimentTui


# == Parameters == #

DURATION_S: int = 240

WIND_SPEED: float = 0.0
WING_L: float = -1  # -1 = closed, 0.5 = open
WING_R: float = 1  # 1 = closed, -0.5 = open
THROTTLE: float = -1

POINT_FREQUENCY = 10
PLOT = False


LIMITS_S = {
    "x": (-200.0, 00.0),
    "y": (-200.0, 00.0),
    "z": (-200.0, 00.0),
    "rx": (-50.0, 50.0),
    "ry": (-20.0, 20.0),
    "rz": (-15.0, 15.0),
}

SINE_PARAMS_S = {
    "x": [
        {"amp": 120.0, "freq": 0.4},
        {"amp": 80, "freq": 1.2, "phase": 0.2},
    ],
    "y": [
        {"amp": 120.0, "freq": 0.9, "phase": 0.15},
        {"amp": 80, "freq": sqrt(1.3), "phase": 0.5},
    ],
    "z": [
        {"amp": 120.0, "freq": 0.6, "phase": 0.05},
        {"amp": 80, "freq": 0.3, "phase": 0.1},
    ],
    "rx": [
        {"amp": 35, "freq": 0.7},
        {"amp": 20, "freq": 0.5, "phase": 0.3},
    ],
    "ry": [
        {"amp": 25, "freq": 0.8},
        {"amp": 15, "freq": 0.6, "phase": 0.7},
    ],
    "rz": [
        {"amp": 15, "freq": 0.7, "phase": 0.2},
        {"amp": 8, "freq": sqrt(0.4), "phase": 0.8},
    ],
}


LIMITS_L = {
    "x": (-250.0, 00.0),
    "y": (-250.0, 00.0),
    "z": (-250.0, 00.0),
    "rx": (-60.0, 60.0),
    "ry": (-25.0, 25.0),
    "rz": (-20.0, 20.0),
}

SINE_PARAMS_L = {
    "x": [
        {"amp": 150.0, "freq": 0.4},
        {"amp": 80, "freq": 1.2, "phase": 0.2},
    ],
    "y": [
        {"amp": 150.0, "freq": 0.9, "phase": 0.15},
        {"amp": 80, "freq": sqrt(1.3), "phase": 0.5},
    ],
    "z": [
        {"amp": 150.0, "freq": 0.6, "phase": 0.05},
        {"amp": 80, "freq": 0.3, "phase": 0.1},
    ],
    "rx": [
        {"amp": 40, "freq": 0.7},
        {"amp": 25, "freq": 0.5, "phase": 0.3},
    ],
    "ry": [
        {"amp": 28, "freq": 0.8},
        {"amp": 17, "freq": 0.6, "phase": 0.7},
    ],
    "rz": [
        {"amp": 18, "freq": 0.7, "phase": 0.2},
        {"amp": 10, "freq": sqrt(0.4), "phase": 0.8},
    ],
}

LIMITS = LIMITS_S
SINE_PARAMS = SINE_PARAMS_S

# == Definitions == #

WORK_POINT: Point = BASE_POINT_DRONE.add(Point(x=600, y=-50, z=-200))


# == Implementation == #


def main():
    generator = SineTrajectoryGenerator(LIMITS, SINE_PARAMS)

    with Orchestrator() as o:
        init(o)
        run_experiment(o, generator)
        finalise(o)


def init(o: Orchestrator) -> None:

    with Status("Initialising free-flight experiment") as status:

        o.execute(
            [
                Reset(),
                Device("drone", DRONE_ACT_ZERO),
                Robot(SetConfig(Config())),
                Robot(SetProfile(FAST_PROFILE)),
                Robot(ReturnHome()),
                Robot(WaitSettled()),
                Robot(SetToolOffset(DRONE_OFFSET)),
                Robot(SetBlending(BLEND_S)),
                Robot(Move(MotionLinear(BASE_POINT_DRONE.add(Point(x=300))))),
                Robot(SetBlending(BLEND_NONE)),
                Robot(Move(MotionLinear(WORK_POINT))),
                Robot(WaitSettled()),
                Robot(SetBlending(BLEND_S)),
                Robot(SetProfile(FAST_PROFILE)),
                Robot(ResetMoveId()),
            ]
        )

        status.update("Biasing")
        o.execute(
            [
                Sleep(1),
                Load(SetBias()),
                Device("drone", [THROTTLE, WING_L, WING_R, 0.0, 0.0]),
            ]
        )

        if WIND_SPEED > 0.0:
            status.update("Settling wind speed")
            o.execute(
                [
                    Wind(SetPowered(True)),
                    Wind(SetFanSpeed(WIND_SPEED)),
                    Wind(WaitSettled()),
                ]
            )


def finalise(o: Orchestrator) -> None:
    o.execute(
        [
            Robot(SetBlending(BLEND_S)),
            Robot(Move(MotionLinear(BASE_POINT_DRONE.add(Point(x=300))))),
            Robot(ReturnHome()),
            Robot(WaitSettled()),
        ]
    )


def run_experiment(o: Orchestrator, generator: BaseTrajectoryGenerator) -> None:

    with ExperimentTui() as ui:
        ui.update(
            title="Free-Flight",
            total_progress=None,
            message="Generating trajectory",
        )

        all_points = []
        instructions = []

        for i in range(DURATION_S):
            timesteps = [i + k / POINT_FREQUENCY for k in range(POINT_FREQUENCY)]
            points = [generator.get_trajectory_point(t) for t in timesteps]

            for point in points:
                profile = Profile(
                    limit=ProfileLimit(
                        translation=point.velocity_t,
                        rotation=rad_to_deg(point.velocity_r),
                    )
                )

                if PLOT:
                    all_points.append(point.pose)

                point = WORK_POINT.add(point.pose)

                instructions.append(Robot(SetProfile(profile)))
                instructions.append(Robot(Move(MotionLinear(point))))
                instructions.append(Sleep(1 / POINT_FREQUENCY))

            instructions.append(Robot(WaitProgress((i + 1) * POINT_FREQUENCY - 5)))

        # Modify blend just before the last motion
        instructions.insert(-3, Robot(SetBlending(BLEND_NONE)))
        instructions.insert(-3, Robot(SetProfile(FAST_PROFILE)))

        # This is crucial, but was forgotten in most measurements!
        # instructions.append(Robot(WaitSettled()))

        if not PLOT:
            ui.update(
                message="Executing trajectory",
                status=ExperimentStatus.RUNNING,
                total_progress=DURATION_S,
            )

            stop = False

            def thread_fn():
                for i in range(DURATION_S):
                    sleep(1)

                    if stop:
                        break

                    ui.update(current_progress=i)

                ui.update(
                    message="Waiting for last motion to complete",
                    status=ExperimentStatus.RUNNING,
                    current_progress=DURATION_S,
                    total_progress=None,
                )

            thread = Thread(target=thread_fn, daemon=True)
            thread.start()

            o.new_experiment(f"free-flight_w{WIND_SPEED}t{WING_L}u{WING_R}v{THROTTLE}")
            o.record(instructions)

            o.execute(
                [
                    Wind(SetFanSpeed()),
                    Wind(SetPowered(False)),
                    Device("drone", DRONE_ACT_ZERO),
                    Robot(SetBlending(BLEND_NONE)),
                    Robot(SetProfile(FAST_PROFILE)),
                    Robot(Move(MotionLinear(WORK_POINT))),
                ]
            )

            ui.update(message="Saving experiment", status=ExperimentStatus.COMPLETE)
            o.save_experiment()

            stop = True
            thread.join()

    if PLOT:
        plot_trajectory(all_points)


def plot_trajectory(points: list[Point]) -> None:
    import matplotlib.pyplot as plt

    x = [p.x for p in points]
    y = [p.y for p in points]
    z = [p.z for p in points]
    rx = [p.rx for p in points]
    ry = [p.ry for p in points]
    rz = [p.rz for p in points]

    # 2. Create the plots
    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(16, 7), subplot_kw={"projection": "3d"}
    )

    fig.suptitle("Robot Trajectory Visualization", fontsize=16)

    # --- Subplot 1: Position (x, y, z) ---
    ax1.plot(x, y, z)

    ax1.set_title("Position Trajectory (meters)")
    ax1.set_xlabel("X axis")
    ax1.set_ylabel("Y axis")
    ax1.set_zlabel("Z axis")

    # # --- Subplot 2: Orientation (rx, ry, rz) ---
    ax2.plot(rx, ry, rz)

    ax2.set_title("Orientation Trajectory (radians)")
    ax2.set_xlabel("RX axis (Pitch)")
    ax2.set_ylabel("RY axis (Roll)")
    ax2.set_zlabel("RZ axis (Yaw)")

    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    plt.show()


if __name__ == "__main__":
    main()
