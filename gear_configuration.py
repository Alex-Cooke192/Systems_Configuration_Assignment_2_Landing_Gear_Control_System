from dataclasses import dataclass

@dataclass(frozen=True)
class GearConfiguration:
    # Immutable landing gear hardware configuration.
    name: str
    pump_latency_ms: int
    actuator_speed_mm_per_100ms: float
    extension_distance_mm: int
    lock_time_ms: int
    requirement_time_ms: int
