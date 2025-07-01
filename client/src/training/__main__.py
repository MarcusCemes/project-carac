from dataclasses import dataclass
from datetime import datetime
from sys import argv

import matplotlib
from matplotlib import pyplot as plt
from pandas import DataFrame
from rich import print as rprint
from rich.status import Status
from torch import save
from torch.utils.data import Dataset

from .datasets import (
    AxisDataset,
    Batch,
    FreeFlightDataset,
    FreeFlightExtendedDataset,
)
from .defs import *
from .networks.mlp import MLPNet
from .networks.lstm import LSTMNet
from .train import TrainingResult, train_model


matplotlib.use("Agg")

TOPIC = "carac-training"


@dataclass
class TrainingJob:
    name: str
    model: MLPNet | LSTMNet
    dataset: Dataset[Batch]
    validation: Dataset[Batch] | None = None


def main():
    if "--models" in argv:
        print_model(MLPNet.default())
        print_model(LSTMNet.default())
        exit(0)

    if "multi" in argv:
        train_multi()
    else:
        train_mlp_lstm()


def train_mlp_lstm():
    rprint("[b]Training MLP and LSTM models[/]")

    with Status("Loading datasets..."):
        dataset_ff = FreeFlightDataset()
        dataset_ff_ext = FreeFlightExtendedDataset()
        dataset_axis = AxisDataset()

        dataset_ff_complete = dataset_ff + dataset_ff_ext
        dataset_ff_complete.__class__.__name__ = "FreeFlightCompleteDataset"

    jobs = [
        TrainingJob("ff-all-mlp", MLPNet.default(), dataset_ff_complete),
        TrainingJob("ff-all-lstm", LSTMNet.default(), dataset_ff_complete),
        TrainingJob("ff-mlp", MLPNet.default(), dataset_ff),
        TrainingJob("ff-lstm", LSTMNet.default(), dataset_ff),
        TrainingJob("axis-mlp", MLPNet.default(), dataset_axis),
        TrainingJob("axis-lstm", LSTMNet.default(), dataset_axis),
    ]

    execute_jobs(jobs)


def train_multi():
    rprint("[b]Training MLP models with multiple layers[/]")

    with Status("Loading datasets..."):
        dataset_ff = FreeFlightDataset()
        dataset_ff_ext = FreeFlightExtendedDataset()
        dataset_axis = AxisDataset()

        dataset_all = dataset_axis + dataset_ff + dataset_ff_ext
        dataset_all.__class__.__name__ = "EntireDataset"

    jobs = [
        TrainingJob(f"mlp-{i}", MLPNet(size), dataset_all)
        for i, size in enumerate(MLP_MULTI_LAYER_SIZES)
    ]

    execute_jobs(jobs)


def execute_jobs(jobs: list[TrainingJob]):
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
    notify_complete()


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


def notify_complete():
    from httpx import post

    post(f"https://ntfy.sh/{TOPIC}", content="Training complete!")


if __name__ == "__main__":
    main()
