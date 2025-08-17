use pyo3::prelude::*;


/// A Python module implemented in Rust
#[pymodule]
fn _mjrs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(add, m)?)?;
    Ok(())
}