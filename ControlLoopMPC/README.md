# Lander Control Loop

There are two control loops:

- **`LoopHop`**  
  Includes navigation (from the *Lossless* algorithm).

- **`LoopTetheredHover`**  
  Maintains a tethered hover position and does **not** require navigation.

---

## Architecture Overview

The system uses three sensor algorithms:

- IMU  
- UWB  
- GPS  

The software is organized into **four asynchronous threads**:

1. **Sensor threads (×3)**  
   Each sensor runs in its own thread and continuously updates its portion of a **shared state estimate**.

2. **MPC control thread**  
   Runs as fast as possible, using the most recent available state estimate to compute optimal control inputs.

---

## File Structure

Proposed file structure (at least for `LoopTetheredHover`):

```text
src/
├── main.rs              // Spawns threads and owns the shared state estimate
├── mpc_control.rs       // MPC control loop logic (control thread)
├── mpc_crate.rs         // MPC solver + internal dynamics model
├── sensors/
│   ├── imu.rs           // IMU reading and preprocessing
│   ├── uwb.rs           // UWB ranging and trilateration
│   └── gps.rs           // GPS reading and parsing
