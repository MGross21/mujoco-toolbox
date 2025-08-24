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
    let output = Command::new(python)
        .arg("-c")
        .arg(r#"
import mujoco
import os
import glob

try:
    mujoco_path = mujoco.__path__[0]
    
    # Search for MuJoCo library files with wildcards for any version
    patterns = [
        os.path.join(mujoco_path, "libmujoco*.so*"),
        os.path.join(mujoco_path, "libmujoco*.dylib"),
        os.path.join(mujoco_path, "mujoco*.dll"),
    ]
    
    lib_files = []
    for pattern in patterns:
        lib_files.extend(glob.glob(pattern))
    
    if lib_files:
        # Return the directory containing the first library file
        lib_dir = os.path.dirname(lib_files[0])
        print(f"LIB_PATH:{lib_dir}")
    else:
        print("LIB_PATH:NOT_FOUND")
        
except Exception as e:
    print(f"ERROR:{e}")
    exit(1)
"#)
        .output()
        .ok()?;

    if !output.status.success() {
        return None;
    }
    
    let stdout = String::from_utf8_lossy(&output.stdout);
    for line in stdout.lines() {
        if let Some(path) = line.strip_prefix("LIB_PATH:") {
            if path != "NOT_FOUND" {
                return Some(PathBuf::from(path));
            }
        }
    }
    
    None
}

fn find_mujoco_include(python: &str) -> Option<PathBuf> {
    let output = Command::new(python)
        .arg("-c")
        .arg(r#"
import mujoco
import os

try:
    mujoco_path = mujoco.__path__[0]
    include_candidates = [
        os.path.join(mujoco_path, "include"),
        os.path.join(mujoco_path, "..", "include"),
    ]
    
    for include_path in include_candidates:
        if os.path.exists(include_path):
            print(include_path)
            break
except:
    pass
"#)
        .output()
        .ok()?;
    
    if output.status.success() {
        let stdout = String::from_utf8_lossy(&output.stdout);
        let stdout_trimmed = stdout.trim();
        if !stdout_trimmed.is_empty() && stdout_trimmed != "NOT_FOUND" {
            return Some(PathBuf::from(stdout_trimmed));
        }
    }
    
    None
}

fn configure_linking(mujoco_lib_path: &PathBuf) {
    println!("cargo:rustc-link-search=native={}", mujoco_lib_path.display());
    
    // Check for versioned library and create symlink if needed
    let lib_files: Vec<_> = std::fs::read_dir(mujoco_lib_path)
        .unwrap_or_else(|_| panic!("Could not read MuJoCo lib directory"))
        .filter_map(|entry| entry.ok())
        .filter(|entry| {
            let name = entry.file_name();
            let name_str = name.to_string_lossy();
            name_str.starts_with("libmujoco") && 
            (name_str.contains(".so") || name_str.contains(".dylib"))
        })
        .collect();

    let standard_lib = mujoco_lib_path.join("libmujoco.so");
    
    if !standard_lib.exists() && !lib_files.is_empty() {
        // Try to create symlink to versioned library
        if let Some(versioned_lib) = lib_files.first() {
            let _ = std::os::unix::fs::symlink(versioned_lib.path(), &standard_lib);
        }
    }
    
    println!("cargo:rustc-link-lib=mujoco");
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

    let out_path = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR not set"));
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Failed to write bindings");
}