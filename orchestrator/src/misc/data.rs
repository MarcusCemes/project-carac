/// De-interleaves a slice of data into multiple channels.
///
/// Given a flat slice `data` where samples from different channels are interleaved,
/// this function separates them into `width` distinct buffers. Each buffer `k`
/// will contain all samples corresponding to the k-th channel.
///
/// # Panics
///
/// This function will panic if `width` is `0` or if `data.len()` is not a multiple of `width`.
pub fn deinterleave_data<T: Copy>(data: &[T], width: usize) -> Box<[Box<[T]>]> {
    let n_samples = data.len() / width;

    assert_eq!(
        data.len(),
        n_samples * width,
        "Data length is not a multiple of width"
    );

    let mut buffers = (0..width)
        .map(|_| {
            let buffer = Box::new_uninit_slice(n_samples);

            // SAFETY: the buffer is completely filled below
            unsafe { buffer.assume_init() }
        })
        .collect::<Vec<_>>()
        .into_boxed_slice();

    for i in 0..n_samples {
        for j in 0..width {
            // SAFETY: data length asserted above, buffers have correct size
            unsafe {
                let source = data.get_unchecked(i * width + j);
                let destination = buffers.get_unchecked_mut(j).get_unchecked_mut(i);

                *destination = *source;
            }
        }
    }

    buffers
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uninterleave_data() {
        let interleaved_data = vec![1, 10, 2, 20, 3, 30, 4, 40];
        let width = 2;
        let deinterleaved = deinterleave_data(&interleaved_data, width);

        assert_eq!(deinterleaved.len(), 2);
        assert_eq!(*deinterleaved[0], [1, 2, 3, 4]);
        assert_eq!(*deinterleaved[1], [10, 20, 30, 40]);
    }

    #[test]
    #[should_panic]
    fn test_uninterleave_data_panic() {
        let interleaved_data = vec![1, 10, 2, 20, 3, 30, 4, 40, 5];
        let width = 2;

        deinterleave_data(&interleaved_data, width);
    }
}
