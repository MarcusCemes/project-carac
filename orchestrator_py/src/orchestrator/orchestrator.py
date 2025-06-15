from typing import Sequence
from httpx import Client, Request, Response

from .instructions import Instruction

DEFAULT_IP = "127.0.0.1"
DEFAULT_PORT = 8080


class OrchestratorError(Exception):
    """Base class for Orchestrator-related exceptions."""

    pass


class Orchestrator:

    def __init__(self, ip: str = DEFAULT_IP, port: int = DEFAULT_PORT):
        self._ip = ip
        self._port = port

    def __enter__(self):
        self._client = Client(base_url=f"http://{self._ip}:{self._port}", timeout=None)
        return self

    def __exit__(self, *_):
        self._client.close()

    # == Commands == #

    def execute(self, instructions: Sequence[Instruction]):
        response = self._client.post(
            "/execute",
            json={"instructions": [i.toJSON() for i in instructions]},
        )

        return self._handle_response(response)

    def record(self, instructions: Sequence[Instruction]):
        response = self._client.post(
            "/record",
            json={"instructions": [i.toJSON() for i in instructions]},
        )

        return self._handle_response(response)

    def new_experiment(self, name: str):
        response = self._client.post(
            "/new-experiment",
            json={"name": name},
        )

        return self._handle_response(response)

    def save_experiment(self):
        response = self._client.post("/save-experiment")
        return self._handle_response(response)

    def start_recording(self):
        response = self._client.post("/start-recording")
        return self._handle_response(response)

    def stop_recording(self):
        response = self._client.post("/stop-recording")
        return self._handle_response(response)

    def status(self) -> str:
        response = self._client.get("/status")
        return self._handle_response(response).text

    def progress(self) -> float:
        response = self._client.get("/progress")
        return self._handle_response(response).json().get("progress")

    @staticmethod
    def _handle_response(request: Response):
        if request.is_error:
            try:
                msg = request.json()["message"]
            except Exception:
                msg = f"{request.text} (HTTP {request.status_code})"

            raise OrchestratorError(msg)

        return request
