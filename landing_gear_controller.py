"""
Title: Landing Gear Control State Machine (LGCS Controller)
Author: Alex Cooke
Date Created: 2026-01-09
Last Modified: 2026-01-10
Version: 1.6

Purpose:
Implements a deterministic landing gear control state machine responsible for
commanding gear deployment and retraction based on system state, configuration-
derived timing, and safety interlocks (e.g., weight-on-wheels and power-loss
behaviour). The controller also integrates basic fault detection/handling using
position sensor inputs, supports maintenance fault flagging, and optionally
records faults to non-volatile storage for traceability and test validation.

Targeted Requirements:
- LGCS-FR001: Landing gear shall transition from UP to DOWN within the required time.
- LGCS-FR002: Landing gear shall transition from DOWN to UP only when weight-on-wheels = FALSE.
- LGCS-FR004: Landing gear shall ignore transition commands while in FAULT or ABNORMAL states.
- LGCS-FR003: Provide state indications for TRANSITIONING_UP, TRANSITIONING_DOWN, UP_LOCKED, DOWN_LOCKED.
- LGCS-SR001: Auto-deploy when below 1000 ft under normal conditions and gear not DOWN.
- LGCS-SR002: Low altitude warning when below 2000 ft under normal conditions and gear not DOWN.
- LGCS-SR003: Weight-on-wheels interlock (supports retract inhibit behaviour).
- LGCS-SR004: Default to DOWN following loss of primary control power and override pilot input.
- LGCS-FTHR001: Tolerate a single position sensor failure and continue using remaining valid sensors.
- LGCS-FTHR002: Persistent (>500 ms) conflicting position sensor inputs cause transition to FAULT.
- LGCS-FTHR003: Record detected faults with timestamp and code to non-volatile storage.
- LGCS-FTHR004: Determine gear state from validated sensor inputs after reset.
- LGCS-PR004: Record fault occurrence and classification timing for validation.

Scope and Limitations:
- Assumes symmetric deploy and retract timing derived from GearConfiguration.
- Does not model partial extension, hydraulic failures, mechanical jams, or realistic dynamics.
- Sensor handling is simplified: uses averaging of valid sensors and a fixed disagreement threshold.
- FAULT and ABNORMAL states are treated as actuator-inhibited safe states.
- RESET-state sensor determination uses a simple position threshold policy; ambiguous readings
  default to a prototype-safe UP_LOCKED assumption and may inhibit command acceptance depending
  on reset validation policy.
- Intended for simulation, design exploration, and requirements validation only.

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python 3.10+
- time (standard library)
- typing (standard library)
- gear_configuration.py
- gear_states.py
- sims/position_simulator.py (PositionSensorReading, SensorStatus)
- Optional: fault_recorder.py (FaultRecorder integration via dependency injection)

Related Documents:
- LGCS Requirements Specification
- LGCS System Safety Assessment
- SRATS-006, SRATS-011 Traceability Records
- Fault Handling and Diagnostics Notes

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

# Change Log (requirements coverage summary):
#
# 1.6 (2026-01-10)
#   - Implemented fault-tolerant sensor logic and fault handling:
#       * LGCS-FTHR001 (single sensor failure tolerance)
#       * LGCS-FTHR002 (persistent sensor conflict -> FAULT)
#       * LGCS-FTHR003 (non-volatile fault recording via FaultRecorder)
#       * LGCS-FTHR004 (RESET: determine state from sensor inputs)
#   - Added LGCS-PR004 fault occurrence vs classification timing instrumentation.
#
# 1.5 (2026-01-10)
#   - Implemented LGCS-SR004 power-loss behaviour (override pilot input and default to DOWN when power absent).
#
# 1.4 (2026-01-10)
#   - Implemented altitude-driven safety behaviours:
#       * LGCS-SR001 auto-deploy (<1000 ft, normal conditions)
#       * LGCS-SR002 low-altitude warning (<2000 ft, normal conditions)
#
# 1.3 (2026-01-09)
#   - Implemented command/state safety gating:
#       * LGCS-FR004 ignore/inhibit deploy/retract while in FAULT or ABNORMAL.
#   - Established state annunciation outputs used by CLI for LGCS-FR003
#     (UP_LOCKED, DOWN_LOCKED, TRANSITIONING_UP, TRANSITIONING_DOWN).
#
# 1.2 (2026-01-09)
#   - Implemented weight-on-wheels retract interlock:
#       * LGCS-FR002 (retract only when WOW=FALSE)
#       * LGCS-SR003 (supports WOW inhibit behaviour)
#
# 1.1 (2026-01-09)
#   - Added retract path + TRANSITIONING_UP state integration (core retraction capability groundwork).
#
# 1.0 (2026-01-09)
#   - Initial deploy state machine implementation and configuration-derived timing:
#       * LGCS-FR001 (UP -> DOWN within required time under valid deploy command).


import time
from gear_configuration import GearConfiguration
from gear_states import GearState
from sims.position_simulator import PositionSensorReading, SensorStatus
from typing import Callable, Sequence


class LandingGearController:
    def __init__(
        self,
        config: GearConfiguration,
        clock=time.monotonic,
        altitude_provider=None,
        normal_conditions_provider=None,
        primary_power_present_provider=None,
        position_sensors_provider=None, 
        fault_recorder=None, 
    ):
        self._config = config
        self._state = GearState.RESET
        self._reset_validated = False

        self._clock = clock
        self._state_entered_at = self._clock()

        # Altitude instrumentation
        self.altitude_provider = altitude_provider
        self.normal_conditions_provider = normal_conditions_provider

        # Auto-deploy latch for SR001
        self._auto_deploy_latched = False
        # Warning latch for SR002
        self._low_alt_warning_active = False

        self._weight_on_wheels: bool = True

        # Timing instrumentation
        self._deploy_cmd_ts: float | None = None
        self._deploy_transition_ts: float | None = None
        self._deploy_actuation_ts: float | None = None

        self._retract_cmd_ts: float | None = None
        self._retract_transition_ts: float | None = None
        self._retract_actuation_ts: float | None = None

        self._deploy_time_s = self._config.compute_deploy_time_ms() / 1000.0
        
        # Deploy/retract variables
        self._deploy_requested = False
        self._retract_requested = False

        # Power instrumentation
        self.primary_power_present_provider = primary_power_present_provider
        self._sr004_power_loss_latched = False

        # Position data
        self.position_sensors_provider: Callable[[], Sequence[PositionSensorReading]] | None = position_sensors_provider
        self._maintenance_fault_active = False
        self._maintenance_fault_codes: set[str] = set()

        # Position estimate from sensor data 
        self._position_estimate_norm: float | None = None

        # Fault recorder
        self._fault_recorder = fault_recorder
        self._recorded_fault_codes: set[str] = set()

        # FTHR002: sensor conflict persistence tracking
        self._sensor_conflict_started_at: float | None = None
        self._sensor_conflict_fault_latched: bool = False

        # FTHR002 tuning constants (can be config if desired)
        self._sensor_conflict_persist_s: float = 0.5  # 500 ms
        self._sensor_conflict_tolerance_norm: float = 0.20  # 20% of travel

        # Fault timing record store
        self._fault_occurrence_ts: dict[str, float] = {}
        self._fault_classified_ts: dict[str, float] = {}

        # Remember last value (to avoid continuous spamming)
        self._last_gear_down_cmd: bool | None = None
        self._last_gear_up_cmd: bool | None = None
        


    @property
    def state(self) -> GearState:
        return self._state

    @property
    def position_estimate_norm(self) -> float | None:
        return self._position_estimate_norm
    
    @property
    def fault_recorder(self):
        return self._fault_recorder

    @fault_recorder.setter
    def fault_recorder(self, recorder) -> None:
        self._fault_recorder = recorder

    def log(self, msg: str) -> None:
        print(msg)

    def set_weight_on_wheels(self, wow: bool) -> None:
        self._weight_on_wheels = wow

    def weight_on_wheels(self) -> bool:
        return self._weight_on_wheels

    def enter_state(self, new_state: GearState) -> None:
        self._state = new_state
        self._state_entered_at = self._clock()

    def down_requested(self) -> bool:
        return self._deploy_requested

    def up_requested(self) -> bool:
        return self._retract_requested

    def update(self) -> None:
        # Advances landing gear state machine by one control tick
        now = self._clock()
        elapsed_s = now - self._state_entered_at

        self._apply_fthr002_conflicting_position_sensors_fault()

        # Once in FAULT/ABNORMAL, stop early (your existing logic)
        if self._state in (GearState.FAULT, GearState.ABNORMAL):
            self._actuate_down(False)
            self._actuate_up(False)
            return


        self._apply_sr004_power_loss_default_down()
        self._position_estimate_norm = self._apply_fthr001_single_sensor_failure_handling()
        self._deliver_low_altitude_warning()
        self._apply_sr001_auto_deploy()

        if self._state in (GearState.FAULT, GearState.ABNORMAL):
            # FR004: Transition commands are ignored in FAULT or ABNORMAL states.
            self._actuate_down(False)
            self._actuate_up(False)
            return
        
        if self._state == GearState.RESET:
            determined = self._determine_state_from_sensors()

            if determined is None:
                # PROTOTYPE POLICY: if sensors can't determine state at startup,
                # assume UP_LOCKED (safe default for flight), keep actuators off.
                self.enter_state(GearState.UP_LOCKED)
                self._reset_validated = False  # or True if you want to accept commands
                self.log("RESET: sensors unknown, defaulting to UP_LOCKED")
                return

            self.enter_state(determined)
            self._reset_validated = True
            return

        if self._state == GearState.UP_LOCKED:
            if self.down_requested():
                self.command_gear_down(True)
            return

        if self._state == GearState.TRANSITIONING_DOWN:
            # Maintains actuation during transition
            self._actuate_down(True)

            # Completes transition using computed deploy time
            if elapsed_s >= self._deploy_time_s:
                self.command_gear_down(False)
            return

        if self._state == GearState.DOWN_LOCKED:
            if self.up_requested():
                if self.weight_on_wheels():  # LGCS-FR002/LGCS-SR003
                    self.log("Retract inhibited: weight-on-wheels=TRUE")
                    return
                self._retract_requested = False
                self.command_gear_up(True)
            return

        if self._state == GearState.TRANSITIONING_UP:
            # Maintain actuation during retraction
            self._actuate_up(True)

            # Complete transition using computed deploy time
            if elapsed_s >= self._deploy_time_s:
                self.command_gear_up(False)
            return

    def _apply_sr001_auto_deploy(self) -> None:
        # LGCS-SR001:
        # Automatic deploy is initiated if altitude is below 1000 ft under normal conditions
        # and gear state is not DOWN.
        if self.altitude_provider is None or self.normal_conditions_provider is None:
            return

        altitude_ft = self.altitude_provider()
        if altitude_ft is None:
            return
        normal = self.normal_conditions_provider()

        if not normal:
            self._auto_deploy_latched = False
            return

        if altitude_ft >= 1000.0:
            self._auto_deploy_latched = False
            return

        if self._state in (GearState.DOWN_LOCKED, GearState.TRANSITIONING_DOWN):
            return

        if self._auto_deploy_latched:
            return

        accepted = self.command_gear_down(True)
        if accepted:
            self._auto_deploy_latched = True
    
    def _deliver_low_altitude_warning(self) -> None:
        # LGCS-SR002:
        # Deliver visual and auditory warning when altitude < 2000 ft under normal conditions
        # and landing gear is not DOWN.

        WARNING_TEXT = "WARNING: ALTITUDE LOW - LANDING GEAR NOT DEPLOYED"

        if self.altitude_provider is None or self.normal_conditions_provider is None:
            return

        altitude_ft = self.altitude_provider()
        if altitude_ft is None:
            return

        normal = self.normal_conditions_provider()

        if (
            normal
            and altitude_ft < 2000.0
            and self._state not in (GearState.DOWN_LOCKED, GearState.TRANSITIONING_DOWN)
        ):
            if not self._low_alt_warning_active:
                # Visual warning
                self.log(WARNING_TEXT)
                # Auditory warning (placeholder)
                self.log("AURAL WARNING: GEAR")

                self._low_alt_warning_active = True
        else:
            self._low_alt_warning_active = False
    
    def _apply_sr004_power_loss_default_down(self) -> None:
        # LGCS-SR004:
        # Following loss of primary control power, default to DOWN and override pilot input
        # while primary control power is not present.

        if self.primary_power_present_provider is None:
            return

        power_present = self.primary_power_present_provider()

        if power_present:
            self._sr004_power_loss_latched = False
            return

        # Override pilot input while power is not present.
        self._deploy_requested = False
        self._retract_requested = False

        # Default to DOWN while power is not present.
        if self._state in (GearState.DOWN_LOCKED, GearState.TRANSITIONING_DOWN):
            return

        if not self._sr004_power_loss_latched:
            self.command_gear_down(True)
            self._sr004_power_loss_latched = True

    def command_gear_down(self, enabled: bool) -> bool:
        # Applies actuator command and performs any required state transition
        now = self._clock()

        if enabled:
            if self._state in (GearState.FAULT, GearState.ABNORMAL):
                self.log(f"Deploy rejected: state={self._state.name}")
                return False
            
            if self._state == GearState.RESET:
                self.log("Deploy rejected: system in RESET state")
                return False

            if self._state != GearState.UP_LOCKED:
                self.log(f"Deploy rejected: state={self._state.name}")
                return False

            self._deploy_requested = False

            self._deploy_cmd_ts = now
            self._deploy_actuation_ts = None
            self._deploy_transition_ts = now
            self._actuate_down(True)
            self.enter_state(GearState.TRANSITIONING_DOWN)
            return True

        if self._state == GearState.TRANSITIONING_DOWN:
            self._actuate_down(False)
            self.enter_state(GearState.DOWN_LOCKED)
            return True

        self._actuate_down(False)
        return False

    def command_gear_up(self, enabled: bool) -> bool:
        # Applies actuator command and performs any required state transition
        now = self._clock()

        if enabled:
            if self.primary_power_present_provider is not None:
                if not self.primary_power_present_provider():  # LGCS-SR004
                    self.log("Retract ignored: primary control power not present")
                    return False

            if self._state in (GearState.FAULT, GearState.ABNORMAL):
                self.log(f"Retract ignored: state={self._state.name}")
                return False
            
            if self._state == GearState.RESET:
                self.log("Command ignored: system in RESET state")
                return False

            if self._state != GearState.DOWN_LOCKED:
                self.log(f"Retract rejected: state={self._state.name}")
                return False

            if self.weight_on_wheels(): #LGCS-FR002/LGCS-SR003
                self.log("Retract rejected: weight-on-wheels=TRUE")
                return False

            self._retract_requested = False

            self._retract_cmd_ts = now
            self._retract_actuation_ts = None
            self._retract_transition_ts = now
            self._actuate_up(True)
            self.enter_state(GearState.TRANSITIONING_UP)
            return True

        if self._state == GearState.TRANSITIONING_UP:
            self._actuate_up(False)
            self.enter_state(GearState.UP_LOCKED)
            return True

        self._actuate_up(False)
        return False

    def _actuate_down(self, enabled: bool) -> None:
        if enabled and self._deploy_cmd_ts is not None and self._deploy_actuation_ts is None:
            self._deploy_actuation_ts = self._clock()
        if enabled != self._last_gear_down_cmd:
            self.log(f"Gear down actuator command: {enabled}")
            self._last_gear_down_cmd = enabled

    def _actuate_up(self, enabled: bool) -> None:
        if enabled and self._retract_cmd_ts is not None and self._retract_actuation_ts is None:
            self._retract_actuation_ts = self._clock()
        if enabled != self._last_gear_up_cmd:
            self.log(f"Gear up actuator command: {enabled}")
            self._last_gear_up_cmd = enabled
    
    def _apply_fthr001_single_sensor_failure_handling(self) -> float | None:
        # LGCS-FTHR001:
        # Continue operation using remaining valid sensors after a single sensor failure
        # and flag a maintenance fault.

        if self.position_sensors_provider is None:
            return None

        readings = self.position_sensors_provider()
        if not readings:
            return None

        valid = [r for r in readings if r.status == SensorStatus.OK]
        failed_count = len(readings) - len(valid)

        if failed_count == 1 and len(valid) >= 1:
            fault_code = "FTHR001_SINGLE_SENSOR_FAILURE"

            self._maintenance_fault_active = True
            self._maintenance_fault_codes.add(fault_code)

            # FTHR003: record fault to non-volatile storage
            self._record_fault(fault_code)

            # PR004: fault occurrence and classification are immediate
            self._mark_fault_classified(
                fault_code=fault_code,
                occurrence_ts=self._clock()
            )

            return sum(r.position_norm for r in valid) / len(valid)


        if failed_count == 0:
            return sum(r.position_norm for r in readings) / len(readings)

        fault_code = "MULTIPLE_SENSOR_FAILURE"
        self._maintenance_fault_active = True
        self._maintenance_fault_codes.add(fault_code)
        self._record_fault(fault_code)
        self._mark_fault_classified(fault_code=fault_code, occurrence_ts=self._clock())
        return None


    def _record_fault(self, fault_code: str) -> None:
        # LGCS-FTHR003:
        # Records detected faults with timestamp and fault code to non-volatile storage.
        if self._fault_recorder is None:
            return

        if fault_code in self._recorded_fault_codes:
            return

        self._fault_recorder.record(fault_code)
        self._recorded_fault_codes.add(fault_code)

    def _apply_fthr002_conflicting_position_sensors_fault(self) -> None:
        # LGCS-FTHR002:
        # Conflicting gear position sensor inputs persisting longer than 500 ms
        # cause transition to FAULT and inhibit further gear commands.

        if self.position_sensors_provider is None:
            self._sensor_conflict_started_at = None
            return

        readings = self.position_sensors_provider()
        if not readings:
            self._sensor_conflict_started_at = None
            return

        valid = [r for r in readings if r.status == SensorStatus.OK]
        if len(valid) < 2:
            self._sensor_conflict_started_at = None
            return

        positions = [float(r.position_norm) for r in valid]
        disagreement = max(positions) - min(positions)

        conflicting = disagreement > self._sensor_conflict_tolerance_norm

        now = self._clock()

        if not conflicting:
            self._sensor_conflict_started_at = None
            return

        if self._sensor_conflict_fault_latched:
            return

        if self._sensor_conflict_started_at is None:
            self._sensor_conflict_started_at = now
            return

        if (now - self._sensor_conflict_started_at) >= self._sensor_conflict_persist_s:
            self._sensor_conflict_fault_latched = True
            self.enter_state(GearState.FAULT)

            fault_code = "FTHR002_SENSOR_CONFLICT_PERSISTENT"
            self._record_fault(fault_code)

            occurrence_ts = self._sensor_conflict_started_at + self._sensor_conflict_persist_s
            self._mark_fault_classified(fault_code=fault_code, occurrence_ts=occurrence_ts)


    def _mark_fault_classified(self, fault_code: str, occurrence_ts: float) -> None:
        # LGCS-PR004:
        # Fault detection and classification timing is recorded for validation.

        if fault_code in self._fault_classified_ts:
            return

        self._fault_occurrence_ts[fault_code] = float(occurrence_ts)
        self._fault_classified_ts[fault_code] = float(self._clock())

    def fault_classification_latency_ms(self, fault_code: str) -> float | None:
        occ = self._fault_occurrence_ts.get(fault_code)
        cls = self._fault_classified_ts.get(fault_code)
        if occ is None or cls is None:
            return None
        return (cls - occ) * 1000.0

    def _determine_state_from_sensors(self) -> GearState | None:
        # LGCS-FTHR004:
        # Determine gear state using validated sensor inputs after reset.

        if self.position_sensors_provider is None:
            return None

        readings = self.position_sensors_provider()
        if not readings:
            return None

        valid = [r for r in readings if r.status == SensorStatus.OK]
        if len(valid) == 0:
            return None

        avg_pos = sum(r.position_norm for r in valid) / len(valid)

        if avg_pos <= 0.1:
            return GearState.UP_LOCKED
        if avg_pos >= 0.9:
            return GearState.DOWN_LOCKED

        return None  # ambiguous → do not accept commands

