# params.py
from dataclasses import dataclass, field
import numpy as np

@dataclass
class RocketParams:
    m_static: float = 0.6
    m_gimbal_top: float = 0.05
    m_gimbal_bottom: float = 0.05
    I_body: np.ndarray = field(default_factory=lambda: np.diag([0.00940, 0.00940, 0.00014]))
    I_gimbal_top: np.ndarray = field(default_factory=lambda: np.diag([0.0001]*3))
    I_gimbal_bottom: np.ndarray = field(default_factory=lambda: np.diag([0.0001]*3))
    thrust_offset: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -0.1]))
    COP_offset: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.1]))
    gimbal_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    Cd_x: float = 0.1
    Cd_y: float = 0.1
    Cd_z: float = 0.1
    A_x: float = 0.7
    A_y: float = 0.7
    A_z: float = 0.7
    rcs_offset: float = 0.1
    air_density: float = 1.225
    T_max: float = 100.0
    T_min: float = 0.0
    dt: float = 0.01

    @property
    def m(self) -> float:
        return self.m_static + self.m_gimbal_top + self.m_gimbal_bottom

    @property
    def inv_I(self) -> np.ndarray:
        return np.linalg.inv(self.I_body)
