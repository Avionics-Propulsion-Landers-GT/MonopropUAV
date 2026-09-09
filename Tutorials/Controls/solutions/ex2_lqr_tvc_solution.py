"""
Solution key for exercise 2 (leads only). Imports the student file, swaps in
Bryson weights and the uncontrollable-mode explanation, then runs its main:

  python ex2_lqr_tvc_solution.py [--no-clip] [--planar]

Expected with these weights on the (2, -3, 10) step under clipping: settles in
about 5.6 s, max |qx|,|qy| about 0.09, gimbal pinned at 10 deg for 8 steps,
thrust 4.7..20 N, max closed-loop |eig| below 1. A gentler alternative that
never saturates: Q = diag(1,1,1,100,100,5,5,5,5,5,1,1,1,1), R = diag(1000,1000,1),
settles in 10.8 s with 3.6 deg of gimbal.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "exercises"))
import ex2_lqr_tvc as ex  # noqa: E402


def student_weights():
    max_state_dev = [
        1.0, 1.0, 1.0,        # x, y, z: a metre of error is the most we accept
        0.05, 0.05,           # qx, qy: about 6 deg of lean
        1.0, 1.0, 1.0,        # vx, vy, vz: 1 m/s
        0.5, 0.5,             # wx, wy: 0.5 rad/s
        0.17, 0.17,           # gimbal angles: the 10 deg stop
        1.0, 1.0,             # gimbal rates
    ]
    max_input = [0.17, 0.17, 10.0]   # 10 deg of gimbal, 10 N of thrust away from hover
    return ex.bryson_weights(max_state_dev, max_input)


ex.student_weights = student_weights
ex.UNCONTROLLABLE_EXPLANATION = (
    "qw (idx 6) is fixed by the unit-norm constraint once qx, qy, qz are known, so it is "
    "not an independent state; qz (idx 5) and wz (idx 12) are yaw, and a two-axis gimbal "
    "on the thrust axis produces no yaw torque, so nothing in u can move them."
)

if __name__ == "__main__":
    raise SystemExit(ex.main())
