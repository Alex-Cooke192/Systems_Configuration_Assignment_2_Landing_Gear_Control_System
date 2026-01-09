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

    def compute_deploy_time_ms(self) -> float:
        # Worst-case theoretical deploy time.
        # Pure calculation, no side effects.
        actuator_speed_mm_per_ms = self.actuator_speed_mm_per_100ms / 100.0

        extension_time_ms = (
            self.extension_distance_mm / actuator_speed_mm_per_ms
        )

        return (
            self.pump_latency_ms +
            extension_time_ms +
            self.lock_time_ms
        )
