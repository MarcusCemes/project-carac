from httpx import Client

from .instructions import Instruction

DEFAULT_IP = "127.0.0.1"
DEFAULT_PORT = 8080


class Orchestrator:

    def __init__(self, ip: str = DEFAULT_IP, port: int = DEFAULT_PORT):
        self._ip = ip
        self._port = port

    def __enter__(self):
        self._client = Client(base_url=f"http://{self._ip}:{self._port}")
        return self

    def __exit__(self, *_):
        self._client.close()

    # == Commands == #

    def execute(self, instructions: list[Instruction]):
        return self._client.post(
            "/execute",
            json={"instructions": [i.toJSON() for i in instructions]},
        )

    def new_experiment(self, name: str):
        return self._client.post(
            "/new_experiment",
            json={"name": name},
        )

    def save_experiment(self):
        return self._client.post(
            "/save_experiment",
        )

    def status(self) -> str:
        return self._client.get("/status").text
