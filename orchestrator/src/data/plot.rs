use core::f32;

use eyre::{Result, eyre};
use plotters::{coord::Shift, prelude::*};

pub fn create_scatter_plot<T>(
    area: &DrawingArea<T, Shift>,
    data: (&[f32], &[f32]),
    chart_name: &str,
    axis_name: &str,
) -> Result<()>
where
    T: DrawingBackend,
{
    if data.0.is_empty() {
        return Ok(());
    }

    let t_min = *data.0.first().unwrap();
    let t_max = *data.0.last().unwrap();

    let mut v_min = f32::INFINITY;
    let mut v_max = f32::NEG_INFINITY;

    for value in data.1 {
        v_min = v_min.min(*value);
        v_max = v_max.max(*value);
    }

    let mut chart = ChartBuilder::on(area)
        .caption(chart_name, ("sans-serif", 24))
        .margin(24)
        .x_label_area_size(64)
        .y_label_area_size(64)
        .build_cartesian_2d(t_min..t_max, v_min..v_max)
        .map_err(|e| eyre!("Failed to build chart {e}"))?;

    chart
        .configure_mesh()
        .x_desc("Time [s]")
        .y_desc(axis_name)
        .x_label_style(("sans-serif", 16))
        .y_label_style(("sans-serif", 16))
        .draw()
        .map_err(|e| eyre!("Failed to draw chart {e}"))?;

    let style = ScatterStyle::new(data.0.len());

    let series = data
        .0
        .iter()
        .zip(data.1)
        .map(|(&x, &y)| style.circle((x, y)));

    chart
        .draw_series(series)
        .map_err(|e| eyre!("Failed to draw series {e}"))?;

    Ok(())
}

struct ScatterStyle {
    opacity: f64,
    size: i32,
}

impl ScatterStyle {
    const COLOUR: RGBColor = RGBColor(149, 81, 150);

    fn new(n_samples: usize) -> Self {
        let (opacity, size) = match n_samples {
            0..500 => (1.0, 4),
            500..1000 => (0.5, 2),
            _ => (0.25, 1),
        };

        Self { opacity, size }
    }

    fn circle(&self, coords: (f32, f32)) -> Circle<(f32, f32), i32> {
        Circle::new(coords, self.size, ShapeStyle::from(self))
    }
}

impl From<&ScatterStyle> for ShapeStyle {
    fn from(value: &ScatterStyle) -> Self {
        ShapeStyle {
            color: ScatterStyle::COLOUR.mix(value.opacity),
            filled: true,
            stroke_width: 0,
        }
    }
}
