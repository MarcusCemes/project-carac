use burn::{nn::*, prelude::*};

const N_OUTPUTS: usize = 6;

#[derive(Module, Debug)]
pub struct Model<B: Backend> {
    activation: Relu,
    linear0: Linear<B>,
    linear1: Linear<B>,
}

#[derive(Config, Debug)]
pub struct ModelConfig {
    num_inputs: usize,
    hidden_size: usize,
}

impl ModelConfig {
    pub fn init<B: Backend>(&self, device: &B::Device) -> Model<B> {
        Model {
            activation: Relu::new(),
            linear0: LinearConfig::new(self.num_inputs, self.hidden_size).init(device),
            linear1: LinearConfig::new(self.hidden_size, N_OUTPUTS).init(device),
        }
    }
}

impl<B: Backend> Model<B> {
    /// # Shapes
    ///   - Images [batch_size, inputs]
    ///   - Output [batch_size, N_OUTPUTS]
    pub fn forward(&self, data: Tensor<B, 2>) -> Tensor<B, 2> {
        let x = self.linear0.forward(data);
        let x = self.activation.forward(x);
        self.linear1.forward(x)
    }
}
