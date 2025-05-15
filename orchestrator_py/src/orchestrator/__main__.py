from .orchestrator import Orchestrator
from .instructions import *

with Orchestrator() as o:
    print(f"Status: {o.status()}")

    point = Point()

    print(o.new_experiment("Test").json())

    print(
        o.execute(
            [
                LoadCell(SetBias()),
                Sleep(2),
            ]
        ).json()
    )

    print(o.save_experiment().json())
