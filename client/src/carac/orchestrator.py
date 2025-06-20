from typing import Sequence

from httpx import AsyncClient, Response

from .instructions import Instruction

DEFAULT_IP = "127.0.0.1"
DEFAULT_PORT = 8080

Instructions = Sequence[Instruction]


class OrchestratorError(Exception):
    pass


class Orchestrator:

    def __init__(self, ip: str = DEFAULT_IP, port: int = DEFAULT_PORT):
        self._ip = ip
        self._port = port
        self._client = None

    async def __aenter__(self):
        self._client = AsyncClient(
            base_url=f"http://{self._ip}:{self._port}",
            timeout=None,
            headers={
                "User-Agent": "carac-python",
                "Content-Type": "application/json",
            },
        )

        return self

    async def __aexit__(self, *_) -> None:
        assert self._client
        await self._client.aclose()

    # == Public == #

    async def start_recording(self) -> None:
        await self._call("POST", "/start-recording")

    async def stop_recording(self) -> None:
        await self._call("POST", "/stop-recording")

    async def execute(self, instructions: Instructions) -> None:
        await self._call("POST", "/execute", json=pack_i(instructions))

    async def record(self, instructions: Sequence[Instruction]) -> None:
        await self._call("POST", "/record", json=pack_i(instructions))

    async def new_experiment(self, name: str) -> None:
        await self._call("POST", "/new-experiment", json={"name": name})

    async def save_experiment(self) -> None:
        await self._call("POST", "/save-experiment")

    async def status(self) -> str:
        response = await self._call("GET", "/status")
        return response.text

    async def progress(self) -> float:
        response = await self._call("GET", "/progress")
        return response.json().get("progress")

    # == Private == #

    async def _call(self, *args, **kwargs) -> Response:
        assert self._client, "Context manger not entered"

        response = await self._client.request(*args, **kwargs)

        if response.is_error:
            try:
                msg = response.json().get("message", response.text)

            except Exception:
                msg = f"{response.text} (HTTP {response.status_code})"

            raise OrchestratorError(msg)

        return response


def pack_i(instructions: Sequence[Instruction]) -> dict:
    return {"instructions": [i.toJSON() for i in instructions]}
