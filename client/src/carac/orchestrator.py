from typing import Sequence

from httpx import AsyncClient, Request, Response

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
        await self._call(Request("POST", "/start-recording"))

    async def stop_recording(self) -> None:
        await self._call(Request("POST", "/stop-recording"))

    async def execute(self, instructions: Instructions) -> None:
        await self._call(Request("POST", "/execute", json=pack_i(instructions)))

    async def record(self, instructions: Sequence[Instruction]) -> None:
        await self._call(Request("POST", "/record", json=pack_i(instructions)))

    async def new_experiment(self, name: str) -> None:
        await self._call(Request("POST", "/new-experiment", json={"name": name}))

    async def save_experiment(self) -> None:
        await self._call(Request("POST", "/save-experiment"))

    async def status(self) -> str:
        response = await self._call(Request("GET", "/status"))
        return response.text

    async def progress(self) -> float:
        response = await self._call(Request("GET", "/progress"))
        return response.json().get("progress")

    # == Private == #

    async def _call(self, request: Request) -> Response:
        assert self._client, "Context manger not entered"

        response = await self._client.send(request)

        if response.is_error:
            try:
                msg = response.json().get("message", response.text)

            except Exception:
                msg = f"{response.text} (HTTP {response.status_code})"

            raise OrchestratorError(msg)

        return response


def pack_i(instructions: Sequence[Instruction]) -> list:
    return [i.toJSON() for i in instructions]
