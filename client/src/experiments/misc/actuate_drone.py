from asyncio import run

from carac.prelude import *


DEVICE = "drone"

# Throttle range: [-1, 1]
# Left sweep O/C: 1 -> -0.5
# Right sweep O/C: -1 -> 0.5


async def main():
    async with Orchestrator() as o:
        await o.execute([*iter_sweep_instructions(25)])

        await o.execute([Device(DEVICE, [-1, -0.5, 0.5, 0, 0]), Sleep(1.0)])
        await o.execute([Device(DEVICE, [-0.5, -0.5, 0.5, 0, 0]), Sleep(1.0)])
        await o.execute([Device(DEVICE, [0.0, -0.5, 0.5, 0, 0]), Sleep(1.0)])
        await o.execute([Device(DEVICE, [0.5, -0.5, 0.5, 0, 0]), Sleep(1.0)])
        await o.execute([Device(DEVICE, [-1, -0.5, 0.5, 0, 0])])


def iter_sweep_instructions(count: int):
    for i in range(count):
        p = i / count
        l = -0.5 + 1.5 * p
        r = 0.5 - 1.5 * p

        yield Device(DEVICE, [-1, l, r, 0, 0])
        yield Sleep(0.05)


if __name__ == "__main__":
    run(main())
