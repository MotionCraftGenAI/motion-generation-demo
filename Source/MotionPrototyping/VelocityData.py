import struct
from dataclasses import dataclass
import ctypes

@dataclass
class VelocityData:
    speed: float
    direction_x: float
    direction_y: float
    direction_z: float
    
    @classmethod
    def from_bytes(cls, data: bytes):
        # Format: 'f' for float, total 4 floats (speed + direction xyz)
        speed, dx, dy, dz = struct.unpack('ffff', data)
        return cls(speed, dx, dy, dz)
    
    def to_bytes(self):
        return struct.pack('ffff', 
                         self.speed,
                         self.direction_x,
                         self.direction_y,
                         self.direction_z)