from asyncio import run

from carac.prelude import *


DEVICE = "drone"


async def main():
    async with Orchestrator() as o:

        for state, duration in [
            ([-1, 0, 0, 0, 0], 0.2),
            ([-1, 0.5, 0, 0, 0], 0.5),
            ([-1, -1, 0.5, 0, 0], 0.5),
            ([-1, 0, -1, 0, 0], 0.5),
            ([-1, 0, 0, 0, 0], 1.0),
        ]:
            await o.execute(
                [
                    Device(DEVICE, state),
                    Sleep(duration),
                ]
            )

        await o.execute([*iter_sweep_instructions(25)])

        await o.execute([Device(DEVICE, [-1, 0, 0, 0, 0])])
        await o.execute([Device(DEVICE, [0.0, 0.2, 0.2, 0, 0]), Sleep(2)])
        await o.execute([Device(DEVICE, [-1, 0, 0, 0, 0]), Sleep(0.5)])


def iter_sweep_instructions(count: int):
    for i in range(count):
        v = -1 + i * (2 / count)

        yield Device(DEVICE, [0, v, v, 0, 0])
        yield Sleep(0.05)


if __name__ == "__main__":
    run(main())
