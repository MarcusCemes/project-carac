from torch.nn import Module, Linear, ReLU, Dropout, Sequential

from ..defs import DROPOUT_RATE, INPUT_COLUMNS, OUTPUT_COLUMNS, MLP_HIDDEN_LAYERS


class MLPNet(Module):
    """
    A configurable Multi-Layer Perceptron (MLP).

    Args:
        input_size (int): The dimensionality of the input features.
        output_size (int): The dimensionality of the output.
        hidden_sizes (list of int): A list where each element is the number
                                    of neurons in a hidden layer. The length
                                    of the list determines the number of hidden
                                    layers.
        dropout_p (float): Dropout probability to apply after each hidden layer.
    """

    def __init__(
        self,
        hidden_sizes: list[int],
        input_size: int = len(INPUT_COLUMNS),
        output_size: int = len(OUTPUT_COLUMNS),
        dropout_p: float = DROPOUT_RATE,
    ):
        super(MLPNet, self).__init__()

        self.input_size = input_size
        self.output_size = output_size
        self.hidden_sizes = hidden_sizes

        layers = []
        in_size = input_size

        # Hidden layers
        for h_size in hidden_sizes:
            layers.append(Linear(in_size, h_size))
            layers.append(ReLU())  # A common activation function
            layers.append(Dropout(p=dropout_p))
            in_size = h_size  # The next layer's input is the current layer's output

        layers.append(Linear(in_size, output_size))

        self.net = Sequential(*layers)

    def forward(self, x):
        return self.net(x)

    @staticmethod
    def default() -> "MLPNet":
        return MLPNet(MLP_HIDDEN_LAYERS)
