from httpx import Client

from .instructions import Instruction

DEFAULT_IP = "127.0.0.1"
DEFAULT_PORT = 8080


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

    def execute(self, instructions: list[Instruction]):
        return self.handle_request(
            self._client.post(
                "/execute",
                json={"instructions": [i.toJSON() for i in instructions]},
            )
        )

    def record(self, instructions: list[Instruction]):
        return self.handle_request(
            self._client.post(
                "/record",
                json={"instructions": [i.toJSON() for i in instructions]},
            )
        )

    def new_experiment(self, name: str):
        return self._client.post(
            "/new_experiment",
            json={"name": name},
        ).raise_for_status()

    def save_experiment(self):
        return self._client.post("/save_experiment").raise_for_status()

    def status(self) -> str:
        return self._client.get("/status", timeout=3).text

    @staticmethod
    def handle_request(request):
        if request.is_error:
            print(f"Request failed: {request.text}")
            request.raise_for_status()

        return request
