from orchestrator.orchestrator import Orchestrator
from orchestrator.instructions import *


def main():
    with Orchestrator() as o:

        for state, duration in [
            ([0, 0, 0, 0, 0], 0.2),
            ([0, 1, 0, 0, 0], 0.5),
            ([0, -1, 1, 0, 0], 0.5),
            ([0, 0, -1, 0, 0], 0.5),
            ([0, 0, 0, 0, 0], 1.0),
        ]:
            o.execute(
                [
                    Device("drone", state),
                    Sleep(duration),
                ]
            )

        o.new_experiment("drone_sweep")
        o.record([*iter_sweep_instructions(25)])
        o.save_experiment()

        o.execute([Device("drone", [0, 0, 0, 0, 0])])


def iter_sweep_instructions(count: int):
    for i in range(count):
        v = -1 + i * (2 / count)

        yield Device("drone", [0, v, v, 0, 0])
        yield Sleep(0.05)


if __name__ == "__main__":
    main()
