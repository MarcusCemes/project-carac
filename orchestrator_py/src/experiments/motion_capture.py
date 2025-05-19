from orchestrator.orchestrator import Orchestrator
from orchestrator.instructions import *


def main():
    with Orchestrator() as o:

        o.new_experiment("motion_capture")

        o.record(
            [
                Sleep(1.0),
            ]
        )

        o.save_experiment()


if __name__ == "__main__":
    main()
