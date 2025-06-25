from dataclasses import dataclass
from datetime import datetime
from sys import argv

from matplotlib import pyplot as plt
from pandas import DataFrame
from rich import print as rprint
from rich.status import Status
from torch import save
from torch.utils.data import Dataset

from .datasets import (
    AxisDataset,
    Batch,
    FreeFlight3Dataset,
    FreeFlightExtended,
)
from .defs import *
from .networks.mlp import MLPNet
from .networks.lstm import LSTMNet
from .train import TrainingResult, train_model


@dataclass
class TrainingJob:
    name: str
    model: MLPNet | LSTMNet
    dataset: Dataset[Batch]


def main():
    if "--models" in argv:
        print_model(MLPNet.default())
        print_model(LSTMNet.default())
        exit(0)

    with Status("Loading datasets..."):
        dataset_free_flight_baby = FreeFlight3Dataset()
        dataset_free_flight_extended = FreeFlightExtended()
        dataset_axis = AxisDataset()

        dataset_free_flight = dataset_free_flight_baby + dataset_free_flight_extended

    jobs = [
        TrainingJob("ff-all-mlp", MLPNet.default(), dataset_free_flight),
        TrainingJob("ff-all-lstm", LSTMNet.default(), dataset_free_flight),
        TrainingJob("ff-mlp", MLPNet.default(), dataset_free_flight_baby),
        TrainingJob("ff-lstm", LSTMNet.default(), dataset_free_flight_baby),
        TrainingJob("axis-mlp", MLPNet.default(), dataset_axis),
        TrainingJob("axis-lstm", LSTMNet.default(), dataset_axis),
    ]

    output_dir = OUTPUT_DIR / datetime.now().strftime("%y%m%d_%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=True)

    for job in jobs:
        dataset_name = job.dataset.__class__.__name__
        model_name = job.model.__class__.__name__

        rprint(
            f"\nJob: [cyan]{job.name}[/] | Model: [yellow]{model_name}[/] | Dataset: [green]{dataset_name}[/]"
        )

        result = train_model(job.model, job.dataset)
        save_result(job, result, output_dir)

    rprint("\n[green bold]Training complete![/green bold]")
    rprint(f"Saved to [b]{output_dir.relative_to(OUTPUT_DIR.parent)}[/]")


def print_model(model: MLPNet | LSTMNet):
    model_name = model.__class__.__name__
    params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    rprint(f"\n[b]Model [cyan]{model_name}[/] ({params} params)\n{model}", end="\n\n")


def save_result(job: TrainingJob, result: TrainingResult, output_dir: Path) -> None:
    path = output_dir / job.name

    # Save the model weights
    save(result.model.state_dict(), path.with_suffix(".pth"))

    # Save the textual model architecture
    with open(path.with_suffix(".txt"), "w") as file:
        file.write(str(job.model))

    # Save the training loss data
    df = DataFrame({"loss": result.loss_history, "val_loss": result.val_loss_history})
    df.to_csv(path.with_suffix(".log"))

    df.plot.line()

    plt.savefig(path.with_suffix(".png"))
    plt.close()


if __name__ == "__main__":
    main()
