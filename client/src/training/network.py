from torch import Tensor
from torch.nn import Linear, Module, ReLU, Sequential


class SimpleFeedForwardNN(Module):

    def __init__(
        self,
        input_size: int,
        output_size: int,
        hidden_layers: list[int],
    ):
        """
        Args:
            input_size (int): The number of input features.
            output_size (int): The number of output features.
            hidden_layers (List[int]): A list where each element is the number of
                                       neurons in a hidden layer.
        """
        super().__init__()

        layers = []
        # Input layer
        current_size = input_size

        # Hidden layers
        for layer_size in hidden_layers:
            layers.append(Linear(current_size, layer_size))
            layers.append(ReLU())
            # Optional: Add Batch Normalization or Dropout for regularization
            # layers.append(nn.BatchNorm1d(layer_size))
            # layers.append(nn.Dropout(0.2))
            current_size = layer_size

        # Output layer
        layers.append(Linear(current_size, output_size))

        # Combine all layers into a sequential model
        self.model = Sequential(*layers)

    def forward(self, x: Tensor) -> Tensor:
        return self.model(x)
