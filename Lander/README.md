You need to design a control loop for the lander.

## The Control loop has 3 main components:

1. Sensor Fusion - 500 Hz (ideally)
Algorithms are in ../Sensing&Controls/AttitudeEstimation/rust-ekf/
Input: Takes in sensor data from Avionics (via update function of EKF)
Output: State for Rocket


We need to do this for our Orientation and Position (Look at flight termination slides to see what sensors to fall back to when some fail).

2. Lossless (Navigation) - 1 Hz (ideally)
Algorithms are in ../Algorithms/LosslessConvexification/rust_lossless/
Input: Take in current position and goal
Output: Trajectory

We will need to do an Ascent Trajectory, Hover, and then a Landing Trajectory.
Our Ascent is just the reverse of our Landing, Lossless can handle this
Hovering is telling our MPC to keep our orientation and position set to upright and 0 motion.

Update the trajectory when possible, otherwise let MPC use whatever it can.

3. MPC - 50 Hz (ideally)
Algorithms are in ../Algorithms/MPC/
Input: Take in current state and trajectory
Output: Controls/Actuations


## The Flight Termination plan is as follows:

1. IMU
Regardless of other sensor failure, terminate flight by zeroing the throttle and stopping all actuation.

2. GPS or UWB
Immediately attempt to land while using the remaining sensor for position.

3. GPS and UWB
Immediately attempt to land while using the double integrated accelerometer data as position.

4. Chamber Pressure Sensor or Tank Pressure Sensor
Immediately attempt landing with MTV lookup table values with no correction.
Implement some sort of chamber PT failure detection for wrong values.
Too much/too little acceleration for chamber pressure

5. Angle Constraint
angle = arcsin(Torque from Thruster / (9.81 * Mass))
This angle comes out to approximately 19.2 degrees.
If we tip past this angle from the vertical, we will terminate flight because we can no longer recover without guaranteeing a loss of altitude.

6. Position
If we drift more than 2 meters from the desired path, then we will immediately attempt to land.
If we detect any position lower than the landing position, we will terminate flight immediately.


We should organize the code as followed. We should have a struct defining the state of our vehicle and a struct defining the state of our control loop.  We will need time to initialize the control loop and vehicle states and then have a main loop that updates the control loop and vehicle states. For now assume sensor readings are going to implemented in future firmware.
