from pathlib import Path
from shutil import move
from subprocess import run

ROOT_PATH = Path(__file__).parent.parent.parent.resolve()

DATA_PATH = ROOT_PATH / "data"

SERVER_PATH = ROOT_PATH / "server"
SERVER_DATA_PATH = SERVER_PATH / "data"
KIT_PATH = SERVER_PATH / "target/release/carac-kit.exe"

UNPROCESSED_PATH = DATA_PATH / "unprocessed"

SMALL_EXPERIMENTS = [
    ("attack-rotations", "attack-rotations/loaded"),
    ("attack-rotations-unloaded", "attack-rotations/unloaded"),
    ("axis", "axis/loaded"),
    ("axis-unloaded", "axis/unloaded"),
    ("axis-2", "axis-2/loaded"),
    ("axis-2-unloaded", "axis-2/unloaded"),
    ("plunge", "plunge/loaded"),
    ("plunge-unloaded", "plunge/unloaded"),
]

LARGE_EXPERIMENTS = [
    ("free-flight", "free-flight/loaded"),
    ("free-flight-unloaded", "free-flight/unloaded"),
    ("free-flight-2", "free-flight-2/loaded"),
    ("free-flight-2-unloaded", "free-flight-2/unloaded"),
    ("free-flight-3", "free-flight-3/loaded"),
    ("free-flight-3-unloaded", "free-flight-3/unloaded"),
    ("free-flight-extended", "free-flight-extended"),
]


def main():
    for input, output in SMALL_EXPERIMENTS:
        export_session(input, output, 1000)

    for input, output in LARGE_EXPERIMENTS:
        export_session(input, output, 10000)


def export_session(session_path: str, output_path: str, divisions: int) -> None:
    session = SERVER_DATA_PATH / session_path

    print(
        f"> Exporting session {session_path} to {output_path} with {divisions} divisions..."
    )

    result = run(
        [
            str(KIT_PATH),
            "export",
            "-c",
            "10",
            "--order",
            "3",
            "-f",
            "parquet",
            "-r",
            "0",
            "-d",
            str(divisions),
            str(session),
        ]
    )

    if result.returncode != 0:
        raise RuntimeError(
            f"Failed to export session {session_path} with return code {result.returncode}"
        )

    move(session / "output", UNPROCESSED_PATH / output_path)


if __name__ == "__main__":
    main()
