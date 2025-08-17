use pyo3::prelude::*;

/// A simple function that adds two numbers
#[pyfunction]
fn add(a: i64, b: i64) -> PyResult<i64> {
    Ok(a + b)
}

/// A Python module implemented in Rust
#[pymodule]
fn _mjrs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(add, m)?)?;
    Ok(())
}