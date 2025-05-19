from orchestrator.orchestrator import Orchestrator
from orchestrator.instructions import *


def main():
    with Orchestrator() as o:

        o.new_experiment("sleep")

        o.record(
            [
                Sleep(3.0),
            ]
        )

        o.save_experiment()


if __name__ == "__main__":
    main()
