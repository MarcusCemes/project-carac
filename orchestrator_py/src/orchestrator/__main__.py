from .orchestrator import Orchestrator
from .instructions import *


def main():
    with Orchestrator() as o:
        print(f"Status: {o.status()}")


if __name__ == "__main__":
    main()
