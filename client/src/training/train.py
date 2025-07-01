from dataclasses import dataclass

from rich import print
from rich.progress import track
from torch import cuda, device as get_device, no_grad, tensor
from torch.optim import Adam
from torch.nn import MSELoss
from torch.utils.data import DataLoader, Dataset, Subset

from .datasets import Batch
from .defs import *
from .networks.mlp import MLPNet
from .networks.lstm import LSTMNet

INPUT_FEATURES = len(INPUT_COLUMNS)
OUTPUT_FEATURES = len(OUTPUT_COLUMNS)


DEVICE = get_device("cuda" if cuda.is_available() else "cpu")
print(f"Using device [b]{DEVICE}[/b]")

PATIENCE = 5
WEIGHT_DECAY = 1e-5


@dataclass
class TrainingResult:
    model: MLPNet | LSTMNet
    loss_history: list[float]
    val_loss_history: list[float] | None


def train_model(
    model: MLPNet | LSTMNet,
    dataset: Dataset[Batch],
    validation: Dataset[Batch] | None = None,
) -> TrainingResult:
    model_name = model.__class__.__name__
    model.to(DEVICE)

    criterion = MSELoss()
    optimiser = Adam(model.parameters(), lr=LEARNING_RATE, weight_decay=WEIGHT_DECAY)

    train_loader, val_loader = create_dataloaders(dataset, validation)

    loss_history: list[float] = []
    val_loss_history: list[float] = []

    patience_counter = 0
    best_val_loss = float("inf")
    best_model_state = None

    for epoch in track(
        range(EPOCHS),
        f"Training {model_name}",
        transient=True,
        total=EPOCHS,
    ):
        losses: list[float] = []

        for batch in train_loader:
            batch_sequences, batch_wrench = batch

            inputs = (
                batch_sequences[:, -1, :]
                if isinstance(model, MLPNet)
                else batch_sequences
            )

            model.train()
            optimiser.zero_grad()

            prediction = model(inputs.to(DEVICE))
            loss = criterion(prediction, batch_wrench.to(DEVICE))

            loss.backward()
            optimiser.step()

            losses.append(loss.item())

        avg_epoch_loss = tensor(losses).mean().item()
        loss_history.append(avg_epoch_loss)

        line = f"  E{epoch+1:02d} | L: {avg_epoch_loss:.6f}"

        avg_val_loss = None

        if val_loader:
            model.eval()
            val_losses = []

            with no_grad():
                for val_batch in val_loader:
                    val_sequences, val_wrench = val_batch

                    val_inputs = (
                        val_sequences[:, -1, :]
                        if isinstance(model, MLPNet)
                        else val_sequences
                    )

                    val_prediction = model(val_inputs.to(DEVICE))
                    val_loss = criterion(val_prediction, val_wrench.to(DEVICE))

                    val_losses.append(val_loss.item())

            avg_val_loss = tensor(val_losses).mean().item()
            val_loss_history.append(avg_val_loss)
            line += f" | V: {avg_val_loss:.6f}"

        if avg_val_loss is not None:
            if avg_val_loss < best_val_loss:
                best_val_loss = avg_val_loss
                patience_counter = 0
                best_model_state = model.state_dict()
            else:
                patience_counter += 1
                line += f" [bright_black](patience {patience_counter}/{PATIENCE})[/]"

            if patience_counter >= PATIENCE:
                print(f"[b yellow]Early stop after {epoch + 1} epochs[/]")
                break

        print(line)

    if best_model_state is not None:
        model.load_state_dict(best_model_state)

    return TrainingResult(model, loss_history, val_loss_history)


def create_dataloaders(
    dataset: Dataset[Batch],
    validation: Dataset[Batch] | None,
) -> tuple[DataLoader, DataLoader]:
    """Creates a training a validation data loader using a chronological split."""

    if validation is not None:
        train_dataset = dataset
        val_dataset = validation
    else:
        dataset_size = len(dataset)  # type: ignore[arg-type]
        split_idx = int(dataset_size * (1 - VAL_SPLIT))

        train_indices = list(range(split_idx))
        val_indices = list(range(split_idx, dataset_size))

        train_dataset = Subset(dataset, train_indices)
        val_dataset = Subset(dataset, val_indices)

    train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False)

    return train_loader, val_loader
