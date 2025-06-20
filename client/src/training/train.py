from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, IntEnum
from pathlib import Path

from matplotlib import pyplot as plt
from rich import print
from rich.progress import track
from rich.status import Status
from torch import cuda, device as get_device, no_grad, save, tensor
from torch.optim import Adam
from torch.nn import MSELoss
from torch.utils.data import DataLoader, Subset

from .datasets import Batch, SequentialFlightDataset
from .defs import *
from .networks.mlp import MLP
from .networks.lstm import LSTMNet

INPUT_FEATURES = len(INPUT_COLUMNS)
OUTPUT_FEATURES = len(OUTPUT_COLUMNS)


class ModelType(Enum):
    Wrench = 0
    Residual = 1


@dataclass
class ModelContext:
    name: str
    model: MLP | LSTMNet
    optimiser: Adam
    type: ModelType

    loss_history: list[float] = field(default_factory=list)
    val_loss_history: list[float] = field(default_factory=list)

    loss_history_tmp: list[float] = field(default_factory=list)
    val_loss_history_tmp: list[float] = field(default_factory=list)


def train_and_save_models(dataset: SequentialFlightDataset) -> Path:
    """
    Trains four model variants in parallel on the same data stream and saves the results.

    The four variants are:
    1. MLP on direct wrench values
    2. MLP on residuals (experimental - analytical)
    3. LSTM on direct wrench values
    4. LSTM on residuals (experimental - analytical)
    """

    device = get_device("cuda" if cuda.is_available() else "cpu")
    print(f"Using device: [b]{device}[/]")

    contexts = init_models()

    for ctx in contexts:
        ctx.model.to(device)
        print(f"\n[b]Model [cyan]{ctx.name}[/]\n{ctx.model}", end="\n\n")

    batch: Batch
    criterion = MSELoss()
    train_loader, val_loader = create_dataloaders(dataset)

    for epoch in track(
        range(EPOCHS),
        "Executing training loop",
        transient=True,
        total=EPOCHS,
    ):
        for batch in train_loader:

            batch_device = map(lambda x: x.to(device), batch)
            (batch_sequences, batch_wrench, batch_residual) = batch_device

            inputs_mlp = batch_sequences[:, -1, :]  # Use the last time step for MLP
            inputs_lstm = batch_sequences  # Use the full sequence for LSTM

            for ctx in contexts:
                ctx.model.train()
                ctx.optimiser.zero_grad()

                input = inputs_mlp if isinstance(ctx.model, MLP) else inputs_lstm

                target = (
                    batch_wrench if ctx.type == ModelType.Wrench else batch_residual
                )

                prediction = ctx.model(input)
                loss = criterion(prediction, target)

                loss.backward()
                ctx.optimiser.step()

                ctx.loss_history_tmp.append(loss.item())

        with no_grad():
            for batch in val_loader:
                batch_device = map(lambda x: x.to(device), batch)
                (batch_sequences, batch_wrench, batch_residual) = batch_device

                inputs_mlp = batch_sequences[:, -1, :]
                inputs_lstm = batch_sequences

                for ctx in contexts:
                    ctx.model.eval()

                    input = inputs_mlp if isinstance(ctx.model, MLP) else inputs_lstm

                    target = (
                        batch_wrench if ctx.type == ModelType.Wrench else batch_residual
                    )

                    prediction = ctx.model(input)

                    val_loss = criterion(prediction, target)
                    ctx.val_loss_history_tmp.append(val_loss.item())

        line = f"E{epoch+1}"

        for ctx in contexts:
            train_avg = tensor(ctx.loss_history_tmp).mean().item()
            val_avg = tensor(ctx.val_loss_history_tmp).mean().item()

            ctx.loss_history.append(train_avg)
            ctx.val_loss_history.append(val_avg)

            line += f" | {train_avg:.4f} ({val_avg:.4f})"

        print(line)

    print("[green bold]Training complete![/green bold]")

    return save_result(OUTPUT_DIR, contexts)


def create_dataloaders(
    dataset: SequentialFlightDataset,
) -> tuple[DataLoader, DataLoader]:
    """Creates a training a validation data loader using a chronological split."""

    dataset_size = len(dataset)
    split_idx = int(dataset_size * (1 - VAL_SPLIT))

    train_indices = list(range(split_idx))
    val_indices = list(range(split_idx, dataset_size))

    train_dataset = Subset(dataset, train_indices)
    val_dataset = Subset(dataset, val_indices)

    train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False)

    return train_loader, val_loader


def save_result(output_dir: Path, contexts: list[ModelContext]) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    model_dir = output_dir / str(timestamp)
    model_dir.mkdir(parents=True, exist_ok=True)

    with Status("Saving results"):
        for ctx in contexts:
            save_path = model_dir / f"{ctx.name}.pth"

            save(ctx.model.state_dict(), save_path)

            with open(model_dir / save_path.with_suffix(".txt"), "w") as f:
                f.write(str(ctx.model))

        save_plots(model_dir, contexts)

    return model_dir


def save_plots(output_dir: Path, contexts: list[ModelContext]) -> None:
    plt.figure(figsize=FIG_SIZE)

    for ctx in contexts:
        plt.plot(ctx.loss_history, label=ctx.name, alpha=0.8)

    plt.title("Training Loss Comparison")
    plt.xlabel("Training Steps")
    plt.ylabel("MSE Loss (Log Scale)")
    plt.legend()
    plt.grid(True, which="both", ls="--")
    plt.yscale("log")

    plt.savefig(output_dir / "loss_comparison.png")

    plt.close()


def init_models() -> list[ModelContext]:
    return [
        init_mlp("mlp-w", ModelType.Wrench),
        init_mlp("mlp-r", ModelType.Residual),
        init_lstm("lstm-w", ModelType.Wrench),
        init_lstm("lstm-r", ModelType.Residual),
    ]


def init_mlp(name: str, type: ModelType) -> ModelContext:
    model = MLP(INPUT_FEATURES, OUTPUT_FEATURES, MLP_HIDDEN_LAYERS)

    return ModelContext(
        name=name,
        model=model,
        optimiser=Adam(model.parameters(), lr=LEARNING_RATE),
        type=type,
    )


def init_lstm(name: str, type: ModelType) -> ModelContext:
    model = LSTMNet(INPUT_FEATURES, LSTM_HIDDEN_SIZE, LSTM_NUM_LAYERS, OUTPUT_FEATURES)

    return ModelContext(
        name=name,
        model=model,
        optimiser=Adam(model.parameters(), lr=LEARNING_RATE),
        type=type,
    )
