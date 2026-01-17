classDiagram
  direction LR

  class LandingGearController {
    <<core>>
    "Holds all data + logic"
    LandingGearController
  ---------------------
    - state: GearStates
    - config: GearConfiguration
    - fault_recorder: FaultRecorder
    - command_recorder: CommandRecorder
    - altitude_simulator: AltitudeSimulator
    - position_simulator: PositionSimulator
    - _deploy_cmd_active: bool
    - _retract_cmd_active: bool
    - _auto_deploy_latched: bool
    - _maintenance_fault_active: bool
    - _maintenance_fault_codes: set[str]
    - ... (timestamps & latches)

    + command_gear_down(enabled): bool
    + command_gear_up(enabled): bool
    + update(dt_s): None

    - _handle_state_*()
    - _actuate_up/down()
    - _check_auto_deploy()
    - _handle_sensor_failures()
    - _record_fault()
  }

  class GearStates {
    <<enumeration>>
    RESET
    UP_LOCKED
    DOWN_LOCKED
    TRANSITIONING_UP
    TRANSITIONING_DOWN
    FAULT
  }

  class CLI {
    <<boundary>>
    <<interactive>>

    %% ==== Collaborators (held during run) ====
    - _ctx: AppContext
    - _controller: LandingGearController
    - _recorder: CommandRecorder

    %% ==== Core entrypoint ====
    + run_rich_cli(ctx: AppContext) int

    %% ==== Command processing ====
    - dispatch(raw_cmd: str) bool
    - record(command: str, action: str, success: bool) void

    %% ==== Presentation / diagnostics ====
    - _print_help() void
    - _print_status() void
    - _reset_controller() void
    - _fmt_sensor(...) str

    %% ==== Loop hookup ====
    - _attach_state_annunciator() void
  }

  %% Relationships (only to your allowed classes)
  CLI --> AppContext : uses
  CLI --> LandingGearController : commands/reads state
  CLI --> CommandRecorder : logs actions

  class AppContext {
    <<composition root>>

    %% ==== Core system objects ====
    + controller: LandingGearController
    + config: GearConfiguration

    %% ==== Recorders ====
    + fault_recorder: FaultRecorder
    + command_recorder: CommandRecorder

    %% ==== Simulators / Inputs ====
    + altitude_simulator: AltitudeSimulator
    + position_simulator: PositionSimulator

    %% ==== Lifecycle / timing ====
    + clock: () -> float

    %% ==== Construction ====
    + __init__(...)
  }

    class FaultRecord {
    <<dataclass>>
    + timestamp_s: float
    + fault_code: str
  }

  %% =========================
  %% Fault Recorder Utility
  %% =========================
  class FaultRecorder {
    <<utility>>
    <<append-only>>

    %% ==== Internal state ====
    - _path: Path
    - _clock: () -> float

    %% ==== Construction ====
    + __init__(filepath: str | Path, clock: () -> float)

    %% ==== Public API ====
    + record(fault_code: str) FaultRecord
  }

  %% =========================
  %% Relationships
  %% =========================
  FaultRecorder --> FaultRecord : creates

  class GearConfiguration {
    <<dataclass>>
    <<immutable>>

    %% ==== Immutable attributes ====
    + name: str
    + pump_latency_ms: int
    + actuator_speed_mm_per_100ms: float
    + extension_distance_mm: int
    + lock_time_ms: int
    + requirement_time_ms: int

    %% ==== Derived calculations ====
    + compute_deploy_time_ms() float
    + meets_deploy_requirement() bool
  }


  class CommandRecorder {
    <<dataclass>>
    <<utility>>

    %% ==== Public attributes (dataclass fields) ====
    + filepath: Path
    + clock: () -> float

    %% ==== Internal state ====
    - _lock: Lock

    %% ==== Lifecycle ====
    + __post_init__() void

    %% ==== Public API ====
    + record(command: str, action: str, success: bool) void
  
  }

  class AltitudeSimulator {
    <<utility>>
    <<simulator>>

    %% ==== Configuration bounds ====
    + min_alt: float
    + max_alt: float
    + max_speed: float
    + max_accel: float

    %% ==== Runtime state ====
    + altitude: float
    + vert_speed: float
    - _last_time: float?

    %% ==== External dependencies ====
    + rng: Random
    + clock: () -> float

    %% ==== Construction ====
    + __init__(
        min_alt: float,
        max_alt: float,
        max_fpm: float,
        max_accel_fps2: float,
        rng: Random?,
        clock: () -> float
      )

    %% ==== Simulation control ====
    + step(dt: float) float
    + update() float

    %% ==== Observation / forcing ====
    + read_altitude_ft() float
    + set_altitude_ft(altitude_ft: float) void
  }

  class SensorStatus {
    <<enumeration>>
    OK
    FAILED
  }

  %% =========================
  %% Position sensor reading
  %% =========================
  class PositionSensorReading {
    <<dataclass>>
    <<immutable>>

    + status: SensorStatus
    + position_norm: float
  }

  %% =========================
  %% Relationships
  %% =========================
  PositionSensorReading --> SensorStatus : has

  %% Relationships (no extra classes introduced)
  CLI --> AppContext : builds/uses
  CLI --> LandingGearController : calls commands + tick/update

  AppContext o-- LandingGearController : owns
  AppContext o-- GearConfiguration : owns
  AppContext o-- FaultRecorder : owns
  AppContext o-- CommandRecorder : owns
  AppContext o-- AltitudeSimulator : owns
  AppContext o-- PositionSimulator : owns

  LandingGearController --> GearStates : state machine uses
  LandingGearController --> GearConfiguration : reads thresholds/timings
  LandingGearController --> FaultRecorder : records faults
  LandingGearController --> CommandRecorder : records accepted/rejected commands
  LandingGearController --> AltitudeSimulator : reads altitude input
  LandingGearController --> PositionSensorReading: reads sensor/position input

  
