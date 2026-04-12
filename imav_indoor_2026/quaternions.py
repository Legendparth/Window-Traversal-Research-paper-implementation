import numpy as np
import math

class Quaternions:
    def __init__(self, r, vector):
        self.r = r
        self.V = vector if isinstance(vector, np.ndarray) else np.array(vector, dtype=np.float64)
    
    def __str__(self):
        return f"({self.r:.2f}, [{self.V[0]:.2f}, {self.V[1]:.2f}, {self.V[2]:.2f}])"
    
    @classmethod
    def from_axis_angle(cls, angle, vector):
        if len(vector) != 3 or not isinstance(vector, (list, tuple, np.ndarray)):
            raise ValueError("Vector must a 3-dimensional list, tuple, or numpy array")
        
        vector = np.array(vector, dtype=np.float64)
        norm = np.linalg.norm(vector)

        if norm == 0:
            raise ValueError("Vector must be non-zero")
        
        r = math.cos(angle / 2.0)
        v = (vector / norm) * math.sin(angle / 2.0)

        return cls(r, v)
    
    def as_np_array(self):
        return np.array([self.r, self.V[0], self.V[1], self.V[2]])
    
    def conjugate(self):
        return Quaternions(self.r, -self.V)

    def add(self, other):
        return Quaternions(
            self.r + other.r,
            self.V + other.V
        )

    def multiply(self, other):
        return Quaternions(
            self.r * other.r - np.dot(self.V, other.V),
            self.r * other.V + other.r * self.V + np.cross(self.V, other.V)
        )

    def rotate_vector(self, vector):
        """Rotates a 3D vector by this quaternion."""
        vector = np.array(vector, dtype=np.float64)

        p = Quaternions(0.0, vector)

        q_conjugate = self.conjugate()
        p_rotated = self.multiply(p).multiply(q_conjugate)

        return p_rotated.V        
    
def get_quaternion_from_3_2_1_euler(roll, pitch, yaw):
    qx = Quaternions.from_axis_angle(roll, [1, 0, 0])
    qy = Quaternions.from_axis_angle(pitch, [0, 1, 0])
    qz = Quaternions.from_axis_angle(yaw, [0, 0, 1])

    return (qz.multiply(qy)).multiply(qx).as_np_array()

def get_quaternion_from_3_2_3_euler(precession, nutation, spin):
    qp = Quaternions.from_axis_angle(precession, [0, 0, 1])
    qn = Quaternions.from_axis_angle(nutation, [0, 1, 0])
    qs = Quaternions.from_axis_angle(spin, [0, 0, 1])

    return (qp.multiply(qn)).multiply(qs).as_np_array()


