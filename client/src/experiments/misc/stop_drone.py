from asyncio import run

from carac.defs import DRONE_NO_ACTUATION
from carac.prelude import *


DEVICE = "drone"


async def main():
    async with Orchestrator() as o:

        await o.execute([Device(DEVICE, DRONE_NO_ACTUATION)])


if __name__ == "__main__":
    run(main())
