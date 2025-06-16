from asyncio import (
    AbstractEventLoop,
    CancelledError,
    Event,
    all_tasks,
    new_event_loop,
    set_event_loop,
)
from contextlib import AbstractAsyncContextManager
from queue import Queue
from sys import stderr
from threading import Thread
from typing import Awaitable

from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

THREAD_NAME = "Background"


class AsyncNode(Node, AbstractAsyncContextManager):
    pass


def spin_async(node: AsyncNode, thread_name=THREAD_NAME):
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    abort = Event()
    ready = Queue[bool](1)

    loop = new_event_loop()
    task = _background_task(node, ready, abort)

    Thread(name=thread_name, target=_run_task, args=(loop, task)).start()

    try:
        if ready.get():
            node.get_logger().info("Node ready, commencing spin")
            executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if loop.is_running():
            loop.call_soon_threadsafe(abort.set)

        executor.shutdown()


def _run_task(loop: AbstractEventLoop, task: Awaitable):
    set_event_loop(loop)

    try:
        loop.run_until_complete(task)

    except CancelledError:
        pass

    except Exception as e:
        print("Task error:", e, file=stderr)
        _shutdown_loop(loop)


async def _background_task(node: AsyncNode, ready: Queue[bool], abort: Event):
    try:
        async with node:
            ready.put(True)
            await abort.wait()

    except Exception:
        ready.put(False)
        raise


def _shutdown_loop(loop: AbstractEventLoop):
    for task in all_tasks(loop):
        task.cancel()

    loop.run_until_complete(loop.shutdown_asyncgens())
    loop.close()
