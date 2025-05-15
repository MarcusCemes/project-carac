use burn::{
    nn::{
        Linear, LinearConfig, Relu,
        loss::{MseLoss, Reduction::Mean},
    },
    prelude::*,
    tensor::backend::AutodiffBackend,
    train::{RegressionOutput, TrainOutput, TrainStep, ValidStep},
};

const N_OUTPUTS: usize = 6;

#[derive(Clone, Debug)]
pub struct DataBatch<B: Backend> {
    pub inputs: Tensor<B, 2>,
    pub targets: Tensor<B, 2>,
}

#[derive(Module, Debug)]
pub struct DroneModel<B: Backend> {
    activation: Relu,
    linear0: Linear<B>,
    linear1: Linear<B>,
}

#[derive(Config, Debug)]
pub struct ModelConfig {
    pub num_inputs: usize,
    pub hidden_size: usize,
}

impl ModelConfig {
    pub fn init<B: Backend>(&self, device: &B::Device) -> DroneModel<B> {
        DroneModel {
            activation: Relu::new(),
            linear0: LinearConfig::new(self.num_inputs, self.hidden_size).init(device),
            linear1: LinearConfig::new(self.hidden_size, N_OUTPUTS).init(device),
        }
    }
}

impl<B: Backend> DroneModel<B> {
    pub fn forward(&self, data: Tensor<B, 2>) -> Tensor<B, 2> {
        let x = self.linear0.forward(data);
        let x = self.activation.forward(x);
        self.linear1.forward(x)
    }

    pub fn forward_step(&self, item: DataBatch<B>) -> RegressionOutput<B> {
        let targets: Tensor<B, 2> = item.targets.unsqueeze_dim(1);
        let output: Tensor<B, 2> = self.forward(item.inputs);

        let loss = MseLoss::new().forward(output.clone(), targets.clone(), Mean);

        RegressionOutput {
            loss,
            output,
            targets,
        }
    }
}

impl<B: AutodiffBackend> TrainStep<DataBatch<B>, RegressionOutput<B>> for DroneModel<B> {
    fn step(&self, item: DataBatch<B>) -> TrainOutput<RegressionOutput<B>> {
        let item = self.forward_step(item);

        TrainOutput::new(self, item.loss.backward(), item)
    }
}

impl<B: Backend> ValidStep<DataBatch<B>, RegressionOutput<B>> for DroneModel<B> {
    fn step(&self, item: DataBatch<B>) -> RegressionOutput<B> {
        self.forward_step(item)
    }
}
