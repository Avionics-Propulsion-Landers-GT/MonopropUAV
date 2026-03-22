import rerun as rr
import pandas as pd

def main():
    print("Loading simulation.csv...")
    try:
        df = pd.read_csv("simulation.csv")
    except FileNotFoundError:
        print("Could not find simulation.csv. Make sure to run `cargo run` first!")
        return

    print(f"Loaded {len(df)} rows. Spawning Rerun viewer...")
    
    # Initialize Rerun and open the native viewer
    rr.init("rocket_sim_csv", spawn=True)

    # Setup the ground plane just like the Rust code did
    rr.log(
        "world/ground",
        rr.Boxes3D(
            centers=[0.0, 0.0, -0.05],
            half_sizes=[100.0, 100.0, 0.05],
            colors=[40, 40, 40]
        ),
        static=True
    )

    for index, row in df.iterrows():
        # Update the Rerun timeline using the new v0.22 send_columns API
        time_column = rr.TimeColumn("time", sequence=[int(row['time'] * 1e9)]) # Convert seconds to nanoseconds for TimeColumn
        
        origin = [[row['com_x'], row['com_y'], row['com_z']]]

        # 1. Rocket center-of-mass (Red point)
        rr.send_columns(
            "world/rocket",
            [time_column],
            rr.Points3D.columns(positions=origin, colors=[[255, 0, 0]], radii=[0.1])
        )

        # 2. Thrust Vector (Blue Arrow)
        rr.send_columns(
            "world/rocket/thrust_vector",
            [time_column],
            rr.Arrows3D.columns(
                origins=origin,
                vectors=[[row['thrust_x'] / 1000.0, row['thrust_y'] / 1000.0, row['thrust_z'] / 1000.0]],
                colors=[[0, 0, 255]]
            )
        )

        # 3. Aerodynamic Drag Vector (Cyan Arrow)
        rr.send_columns(
            "world/rocket/aero_drag",
            [time_column],
            rr.Arrows3D.columns(
                origins=origin,
                vectors=[[row['aero_drag_x'] / 1000.0, row['aero_drag_y'] / 1000.0, row['aero_drag_z'] / 1000.0]],
                colors=[[0, 255, 255]]
            )
        )

        # 4. Aerodynamic Torque Vector (Magenta Arrow)
        rr.send_columns(
            "world/rocket/aero_moment",
            [time_column],
            rr.Arrows3D.columns(
                origins=origin,
                vectors=[[row['aero_moment_x'] / 100.0, row['aero_moment_y'] / 100.0, row['aero_moment_z'] / 100.0]],
                colors=[[255, 0, 255]]
            )
        )

    print("Finished streaming data to Rerun!")

if __name__ == "__main__":
    main()
