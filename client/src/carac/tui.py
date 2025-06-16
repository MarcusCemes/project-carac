from rich.console import Console
from rich.panel import Panel
from rich.align import Align
from rich.text import Text
from rich.live import Live
from rich.layout import Layout
from enum import Enum


class ExperimentStatus(Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    COMPLETE = "COMPLETE"


class ExperimentTui:
    def __init__(self):
        self.console = Console()
        self.title = ""
        self.current_progress = 0
        self.total_progress = 100
        self.status = ExperimentStatus.IDLE
        self.message = ""
        self._live = None

    def __enter__(self):
        # Create the layout and start Live with screen=True for alternate buffer
        layout = self._make_layout()
        self._live = Live(layout, refresh_per_second=4, screen=True)
        self._live.__enter__()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._live:
            self._live.__exit__(exc_type, exc_val, exc_tb)

    def _make_layout(self) -> Layout:
        """Create the main layout structure"""
        layout = Layout(name="root")

        # Split into three sections: top padding, content, bottom padding
        layout.split(
            Layout(name="top", ratio=1),
            Layout(name="center", size=16),  # Increased size for full panel
            Layout(name="bottom", ratio=1),
        )

        # Set top and bottom to empty content (just spaces)
        layout["top"].update("")
        layout["bottom"].update("")

        # Update the center with our content
        layout["center"].update(self._create_content())

        return layout

    def _get_status_color(self):
        """Get color based on current status"""
        colors = {
            ExperimentStatus.IDLE: "white",
            ExperimentStatus.RUNNING: "green",
            ExperimentStatus.PAUSED: "yellow",
            ExperimentStatus.COMPLETE: "blue",
        }
        return colors.get(self.status, "white")

    def _create_content(self):
        """Create the content panel"""
        # Create title section
        title_text = Text(self.title, style="bold white", justify="center")

        # Create status section
        status_color = self._get_status_color()
        status_text = Text(f"Status: {self.status.value}", style=f"bold {status_color}")

        # Create progress section
        progress_percentage = (
            (self.current_progress / self.total_progress * 100)
            if self.total_progress > 0
            else 0
        )
        progress_text = f"Progress: {self.current_progress}/{self.total_progress} ({progress_percentage:.1f}%)"

        # Create a visual progress bar
        bar_width = 40
        filled_width = (
            int((self.current_progress / self.total_progress) * bar_width)
            if self.total_progress > 0
            else 0
        )
        bar = "█" * filled_width + "░" * (bar_width - filled_width)
        progress_bar = Text(f"[{bar}]", style="green")

        # Create message section
        message_text = Text(self.message, style="white", justify="center")

        # Combine all sections
        content = Text.assemble(
            title_text,
            "\n\n",
            status_text,
            "\n\n",
            progress_text,
            "\n",
            progress_bar,
            "\n\n",
            "Current Task:\n",
            message_text,
        )

        # Create a centered panel with fixed width
        panel = Panel(
            Align.center(content),
            title="Task Monitor",
            border_style=status_color,
            padding=(1, 4),
            width=60,
        )

        # Center the panel horizontally
        return Align.center(panel)

    def update(
        self,
        title=None,
        current_progress=None,
        total_progress=None,
        status=None,
        message=None,
    ):
        """Update the state and refresh display"""
        if title is not None:
            self.title = title
        if current_progress:
            self.current_progress = current_progress
        if total_progress is not None:
            self.total_progress = total_progress
        if status is not None:
            self.status = status
        if message is not None:
            self.message = message

        # Update the layout with new content
        if self._live:
            layout = self._make_layout()
            self._live.update(layout)
