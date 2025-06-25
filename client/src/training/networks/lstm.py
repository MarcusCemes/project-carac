from torch.nn import Module, LSTM, Linear

from ..defs import (
    INPUT_COLUMNS,
    OUTPUT_COLUMNS,
    LSTM_HIDDEN_SIZE,
    LSTM_NUM_LAYERS,
    LSTM_SEQUENCE_LENGTH,
)


class LSTMNet(Module):
    """
    An LSTM network for sequence prediction.

    Args:
        input_size (int): The dimensionality of the input features.
        hidden_size (int): The number of features in the hidden state of the LSTM.
        num_layers (int): The number of recurrent layers in the LSTM.
        output_size (int): The dimensionality of the output.
        dropout_p (float): Dropout probability for the LSTM layers.
    """

    def __init__(
        self,
        input_size: int,
        hidden_size: int,
        num_layers: int,
        output_size: int,
        dropout_p: float = 0.1,
    ):
        super(LSTMNet, self).__init__()

        self.hidden_size = hidden_size
        self.num_layers = num_layers

        self.lstm = LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,  # Input shape: (batch_size, sequence_length, input_size)
            dropout=dropout_p if num_layers > 1 else 0,
        )

        self.fc = Linear(hidden_size, output_size)

    def forward(self, x):
        """
        The forward pass of the LSTM.

        Args:
            x (torch.Tensor): Input tensor of shape (batch_size, sequence_length, input_size).

        Returns:
            torch.Tensor: The output tensor of shape (batch_size, output_size).
        """
        # The LSTM returns:
        # 1. output: The output of the last layer for each time step (batch, seq_len, hidden_size)
        # 2. (h_n, c_n): The final hidden and cell states
        output, (h_n, c_n) = self.lstm(x)

        del h_n
        del c_n

        # We only care about the output of the very last time step
        # output[:, -1, :] gives us the hidden state of the last time step for each sequence in the batch
        last_time_step_output = output[:, -1, :]

        # Pass the last time step's output through the final fully connected layer
        out = self.fc(last_time_step_output)
        return out

    @staticmethod
    def default() -> "LSTMNet":
        return LSTMNet(
            input_size=len(INPUT_COLUMNS),
            hidden_size=LSTM_HIDDEN_SIZE,
            num_layers=LSTM_NUM_LAYERS,
            output_size=len(OUTPUT_COLUMNS),
        )
