use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let python = get_python_interpreter();
    
    let mujoco_lib_path = find_mujoco_lib(&python).unwrap_or_else(|| {
        panic!(
            "Could not locate MuJoCo shared library.\n\
             Ensure 'mujoco>=3.0.0' is in [build-system].requires in pyproject.toml."
        );
    });

    configure_linking(&mujoco_lib_path);
    
    let include_path = find_mujoco_include(&python);
    generate_bindings(include_path);
}

fn get_python_interpreter() -> String {
    env::var("PYO3_PYTHON")
        .or_else(|_| env::var("PYTHON_SYS_EXECUTABLE"))
        .unwrap_or_else(|_| "python3".to_string())
}

fn find_mujoco_lib(python: &str) -> Option<PathBuf> {
    let script = r#"
import mujoco
import glob
import os

mujoco_path = mujoco.__path__[0]
patterns = [
    os.path.join(mujoco_path, "libmujoco*.so*"),
    os.path.join(mujoco_path, "libmujoco*.dylib"),
    os.path.join(mujoco_path, "mujoco*.dll"),
]

lib_files = []
for pattern in patterns:
    lib_files.extend(glob.glob(pattern))

if lib_files:
    print(os.path.dirname(lib_files[0]))
"#;

    let output = Command::new(python)
        .arg("-c")
        .arg(script)
        .output()
        .ok()?;

    if output.status.success() {
        let stdout = String::from_utf8_lossy(&output.stdout);
        let path = stdout.trim();
        if !path.is_empty() {
            return Some(PathBuf::from(path));
        }
    }
    
    None
}

fn find_mujoco_include(python: &str) -> Option<PathBuf> {
    let script = r#"
import mujoco
import os

mujoco_path = mujoco.__path__[0]
include_candidates = [
    os.path.join(mujoco_path, "include"),
    os.path.join(mujoco_path, "..", "include"),
]

for include_path in include_candidates:
    if os.path.exists(include_path):
        print(include_path)
        break
"#;

    let output = Command::new(python)
        .arg("-c")
        .arg(script)
        .output()
        .ok()?;
    
    if output.status.success() {
        let stdout = String::from_utf8_lossy(&output.stdout);
        let path = stdout.trim();
        if !path.is_empty() {
            return Some(PathBuf::from(path));
        }
    }
    
    None
}

fn configure_linking(mujoco_lib_path: &PathBuf) {
    println!("cargo:rustc-link-search=native={}", mujoco_lib_path.display());
    
    // Create symlink for versioned libraries if needed
    ensure_standard_library_link(mujoco_lib_path);
    
    println!("cargo:rustc-link-lib=mujoco");
}

fn ensure_standard_library_link(lib_path: &PathBuf) {
    let standard_lib = lib_path.join("libmujoco.so");
    
    if standard_lib.exists() {
        return;
    }

    // Find versioned library files
    if let Ok(entries) = std::fs::read_dir(lib_path) {
        let versioned_libs: Vec<_> = entries
            .filter_map(|entry| entry.ok())
            .filter(|entry| {
                let name = entry.file_name();
                let name_str = name.to_string_lossy();
                name_str.starts_with("libmujoco") && 
                (name_str.contains(".so") || name_str.contains(".dylib"))
            })
            .collect();

        if let Some(versioned_lib) = versioned_libs.first() {
            let _ = std::os::unix::fs::symlink(versioned_lib.path(), &standard_lib);
        }
    }
}

fn generate_bindings(include_path: Option<PathBuf>) {
    println!("cargo:rerun-if-changed=bind.h");
    
    let mut builder = bindgen::Builder::default()
        .header("bind.h")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()));
    
    if let Some(path) = include_path {
        builder = builder.clang_arg(format!("-I{}", path.display()));
    }
    
    let bindings = builder
        .generate()
        .expect("Unable to generate bindings from bind.h");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Failed to write bindings");
}