from pathlib import Path
from typing import TypeVar

from rich.progress import Progress
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Subset, random_split


from .dataset import FreeFlightDataset
from .defs import *
from .network import SimpleFeedForwardNN

T = TypeVar("T")


INPUT_SIZE = len(INPUT_COLUMNS)
OUTPUT_SIZE = len(OUTPUT_COLUMNS)


def train(dataset: FreeFlightDataset, save_path: Path):
    train_loader, val_loader = map(create_loader, split_dataset(dataset))
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    model = SimpleFeedForwardNN(
        input_size=INPUT_SIZE,
        output_size=OUTPUT_SIZE,
        hidden_layers=HIDDEN_LAYERS,
    )

    model.to(device)

    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)

    with Progress(transient=True) as progress:
        task = progress.add_task("Training", total=EPOCHS)

        for epoch in range(EPOCHS):

            model.train()
            train_loss = 0.0

            for inputs, targets in train_loader:
                inputs = inputs.to(device)
                targets = targets.to(device)

                outputs = model(inputs)
                loss = criterion(outputs, targets)

                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                train_loss += loss.item() * inputs.size(0)

            model.eval()
            val_loss = 0.0

            with torch.no_grad():
                for inputs, targets in val_loader:
                    inputs = inputs.to(device)
                    targets = targets.to(device)

                    outputs = model(inputs)
                    loss = criterion(outputs, targets)

                    val_loss += loss.item() * inputs.size(0)

            train_loss /= len(train_loader.dataset)  # type: ignore
            val_loss /= len(val_loader.dataset)  # type: ignore

            progress.update(task, advance=1)
            progress.console.log(
                f"Epoch {epoch+1}/{EPOCHS} \t Training Loss: {train_loss:.6f} \t Validation Loss: {val_loss:.6f}"
            )

    print("Saving model to", save_path)
    torch.save(model.state_dict(), save_path)

    with open(save_path.with_suffix(".txt"), "w") as f:
        f.write(str(model))


def split_dataset(dataset: FreeFlightDataset, val_split: float = VAL_SPLIT):
    val_size = int(len(dataset) * val_split)
    train_size = len(dataset) - val_size

    train_dataset, val_dataset = random_split(dataset, [train_size, val_size])
    return train_dataset, val_dataset


def create_loader(dataset: Subset[T], batch_size: int = BATCH_SIZE) -> DataLoader[T]:
    return DataLoader(dataset, batch_size=batch_size, shuffle=True)
