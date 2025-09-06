## Using the Rust Implementation of NMPC

To use the Rust implementation of NMPC (Nonlinear Model Predictive Control), follow these steps:

### 1. Install Rust and Cargo

- Download and install Rust from [rust-lang.org](https://www.rust-lang.org/tools/install).
- Cargo is included with Rust and is the package manager and build tool for Rust projects.

### 2. Dependencies

- The `Cargo.toml` includes dependencies and is the configuration file for the Rust project.
- Make sure any dependencies are included here

### 3. Build and Run

- Build project in the terminal:

```sh
cargo build
```

- Run project in the terminal:

```sh
cargo run
```

### 4. Check Results

- It should output 3 graphs as pngs. They will show gimbal angles, thrust, and rocket position.

