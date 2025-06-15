from rich.console import Group
from rich.live import Live
from rich.panel import Panel
from rich.progress import Progress, BarColumn, TextColumn
from rich.table import Table
from rich.status import Status
from rich.text import Text


class ExperimentUi:

    def __enter__(self):
        self._progress = Progress(
            "{task.description}",
            BarColumn(),
            TextColumn("[progress.percentage]{task.completed}/{task.total}"),
        )

        self._experiment_task = self._progress.add_task("[green]Progress", total=None)

        self._status = Status("Getting ready...")
        self._message = Text()
        self._status_group = Group(self._status, self._message)

        self._table = Table.grid(expand=True)
        self._table.add_column(ratio=1)
        self._table.add_column(ratio=1)

        self._table.add_row(
            Panel(
                self._progress,
                title="[b]Orchestrator[/]",
                border_style="green",
                padding=(2, 2),
            ),
            Panel(
                self._status_group,
                title="[b]Experiment[/]",
                border_style="red",
                padding=(1, 2),
                expand=True,
            ),
        )

        self._live = Live(self._table)
        self._live.start()

        return self

    def __exit__(self, *_):
        self._live.stop()

    # == Public == #

    def set_total(self, total: int):
        self._progress.update(self._experiment_task, total=total)
        self._live.refresh()

    def advance(self, advance: int = 1):
        self._progress.advance(self._experiment_task, advance=advance)
        self._live.refresh()

    def set_status(self, status: str):
        self._status.update(status)
        self._live.refresh()

    def set_message(self, message: str | None):
        self._message.truncate(0)

        if message:
            self._message.append(message)

        self._live.refresh()
