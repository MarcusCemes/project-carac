from .orchestrator import Orchestrator
from .instructions import *

with Orchestrator() as o:
    print(f"Status: {o.status()}")
