# Lander Control Loop

There are two control loops. LoopHop and LoopTetheredHover. LoopHop will have navigation (from the Lossless algorithm) and LoopTetheredHover will maintain a tethered hover position with no need for navigation.

## Structure

There are three sensor algorithms: IMU, UWB, and GPS.

There will be four threads which run asynchronously?

Three sensor threads continuously update their portion of a shared state estimate.

The MPC control thread runs as fast as it can, using the latest available state estimate to compute the optimal control inputs.

## File Structure
This is what i think the file structure should be (for LoopTetheredHover at least)?

src/ 
├── main.rs              // Spawns threads and stores the shared state estimate
├── mpc_control.rs       // Control logic (MPC thread)
├── sensors/
│   ├── imu.rs           // IMU reading & processing
│   ├── uwb.rs           // UWB ranging & trilateration
│   └── gps.rs           // GPS reading & parsing
├── mpc_crate.rs         // MPC solver and internal dynamics model for the MPC
