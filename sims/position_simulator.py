from dataclasses import dataclass
from enum import Enum, auto

class SensorStatus(Enum):
    OK = auto()
    FAILED = auto()

@dataclass(frozen=True)
class PositionSensorReading:
    status: SensorStatus
    position_norm: float  # 0.0 = fully up, 1.0 = fully down
