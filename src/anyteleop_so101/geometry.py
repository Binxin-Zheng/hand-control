"""Column-vector convention: p_A = R_A_B @ p_B + t_A_B. SI units."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


def finite_array(value, shape, name="array") -> np.ndarray:
    a = np.asarray(value, dtype=float)
    if a.shape != shape or not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must be finite with shape {shape}")
    return a.copy()


def proper_rotation(value) -> np.ndarray:
    R = finite_array(value, (3, 3), "rotation")
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(R), 1., atol=1e-6):
        raise ValueError("Rotation must be in SO(3); a mirrored preview is NOT a rotation")
    return R


def unit(v) -> np.ndarray:
    v = finite_array(v, (3,), "vector")
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("Degenerate direction")
    return v / n


@dataclass(frozen=True)
class Pose:
    p: np.ndarray
    R: np.ndarray

    def __post_init__(self):
        p, R = finite_array(self.p, (3,), "position"), proper_rotation(self.R)
        p.setflags(write=False); R.setflags(write=False)
        object.__setattr__(self, "p", p); object.__setattr__(self, "R", R)

    @classmethod
    def identity(cls):
        return cls(np.zeros(3), np.eye(3))

    @classmethod
    def from_xyzw(cls, p, quaternion):
        q = finite_array(quaternion, (4,), "xyzw")
        if abs(np.linalg.norm(q) - 1.) > 1e-3:
            raise ValueError("Quaternion must already be normalized")
        return cls(p, Rotation.from_quat(q).as_matrix())

    def as_dict(self):
        return {"position_m": self.p.tolist(), "quaternion_xyzw": Rotation.from_matrix(self.R).as_quat().tolist()}

    def compose(self, other):
        return Pose(self.R @ other.p + self.p, self.R @ other.R)

    def inverse(self):
        return Pose(-self.R.T @ self.p, self.R.T)


def rotation_error(target_R, actual_R):
    return Rotation.from_matrix(proper_rotation(target_R) @ proper_rotation(actual_R).T).as_rotvec()


def cap_norm(v, maximum):
    if not np.isfinite(maximum) or maximum <= 0:
        raise ValueError("maximum must be positive")
    n = float(np.linalg.norm(v))
    return np.asarray(v) * min(1., maximum / max(n, 1e-15))
