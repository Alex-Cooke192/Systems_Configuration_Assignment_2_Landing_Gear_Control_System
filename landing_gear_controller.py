"""
Title: Landing Gear Control State Machine (LGCS Controller)
Author: Alex Cooke
Date Created: 2026-01-09
Last Modified: 2026-01-09
Version: 1.0

Purpose:
Implements a deterministic landing gear control state machine responsible for
commanding gear deployment and retraction based on system state, timing derived
from configuration, and safety interlocks such as weight-on-wheels and fault states.
The controller separates command intent, timing, and actuator outputs to enable
traceable and testable behavior.

Targeted Requirements:
- LGCS-FR001: Landing gear shall transition from UP to DOWN within the required time.
- LGCS-FR002: Landing gear shall transition from DOWN to UP only when weight-on-wheels = FALSE.
- LGCS-FR004: Landing gear shall ignore retract commands while in FAULT or ABNORMAL states.

Scope and Limitations:
- Assumes symmetric deploy and retract timing derived from GearConfiguration.
- Does not model partial extension, hydraulic failures, or sensor disagreement.
- FAULT and ABNORMAL states are treated as actuator-inhibited safe states.
- Intended for simulation, design exploration, and requirements validation only.

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python 3.10+
- gear_configuration.py
- gear_states.py

Related Documents:
- LGCS Requirements Specification
- LGCS System Safety Assessment
- SRATS-006, SRATS-011 Traceability Records

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""


import time
from gear_configuration import GearConfiguration
from gear_states import GearState


class LandingGearController:
    def __init__(
        self,
        config: GearConfiguration,
        clock=time.monotonic,
        altitude_provider=None,
        normal_conditions_provider=None,
    ):
        self._config = config
        self._state = GearState.UP_LOCKED

        self._clock = clock
        self._state_entered_at = self._clock()

        # Altitude instrumentation
        self.altitude_provider = altitude_provider
        self.normal_conditions_provider = normal_conditions_provider

        # Auto-deploy latch for SR001
        self._auto_deploy_latched = False

        self._weight_on_wheels: bool = True

        # Timing instrumentation
        self._deploy_cmd_ts: float | None = None
        self._deploy_transition_ts: float | None = None
        self._deploy_actuation_ts: float | None = None

        self._retract_cmd_ts: float | None = None
        self._retract_transition_ts: float | None = None
        self._retract_actuation_ts: float | None = None

        self._deploy_time_s = self._config.compute_deploy_time_ms() / 1000.0

        self._deploy_requested = False
        self._retract_requested = False

    @property
    def state(self) -> GearState:
        return self._state

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

        self._apply_sr001_auto_deploy()

        if self._state in (GearState.FAULT, GearState.ABNORMAL):
            # FR004: Transition commands are ignored in FAULT or ABNORMAL states.
            self._actuate_down(False)
            self._actuate_up(False)
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
                if self.weight_on_wheels():  # LGCS-FR002
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

    def command_gear_down(self, enabled: bool) -> bool:
        # Applies actuator command and performs any required state transition
        now = self._clock()

        if enabled:
            if self._state in (GearState.FAULT, GearState.ABNORMAL):
                self.log(f"Deploy ignored: state={self._state.name}")
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
            if self._state in (GearState.FAULT, GearState.ABNORMAL):
                self.log(f"Retract ignored: state={self._state.name}")
                return False

            if self._state != GearState.DOWN_LOCKED:
                self.log(f"Retract rejected: state={self._state.name}")
                return False

            if self.weight_on_wheels():
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
        self.log(f"Gear down actuator command: {enabled}")

    def _actuate_up(self, enabled: bool) -> None:
        if enabled and self._retract_cmd_ts is not None and self._retract_actuation_ts is None:
            self._retract_actuation_ts = self._clock()
        self.log(f"Gear up actuator command: {enabled}")
