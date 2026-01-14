# LGCS Requirements Traceability Matrix

## Document Purpose
This document provides bidirectional traceability between the Landing Gear Control System (LGCS) requirements and the implemented software artefacts within this prototype. The document explains each requirements implementation, simulation, and QA verification. Structured tooling and structured problem-solving techniques are also given. 

This artefact supports:
- Verification and validation activities
- Quality assurance assessment
- Safety and fault-tolerance review
- Auditability and maintainability of the prototype

---

## Functional Requirements (FR)

### LGCS-FR001 – Deploy Transition Timing
**Requirement:**  
When the system is in the UP_LOCKED state and a valid deploy command is received, the landing gear system shall transition to the DOWN_LOCKED state within the configured requirement time (usually 8000 milliseconds).

**Implementation:**  
- `landing_gear_controller.py`
  - Deploy command acceptance logic
  - State transition timing and completion checks

**Verification:**  
- `test_deploy.py`
  - Deploy completion timing tests
  - Boundary and large time-step cases

**Evidence:**  
- Passing automated unit tests using deterministic fake clock
- CLI integration demo showing deploy timing behaviour

---

### LGCS-FR002 – Retract Interlock (Weight-on-Wheels)
**Requirement:**  
When the system is in the DOWN_LOCKED state, the landing gear system shall transition to the UP_LOCKED state only when weight-on-wheels is FALSE.

**Implementation:**  
- `landing_gear_controller.py`
  - Retract command gating logic
  - Weight-on-wheels interlock

**Verification:**  
- `test_retract.py`
- `test_sr003_inhibits_retract_when_weight_on_wheels_true`

**Evidence:**  
- Unit test verification of retract rejection and acceptance
- CLI command audit log demonstrating WOW gating

---

### LGCS-FR003 – State Annunciation
**Requirement:**  
The landing gear system shall provide visual status indications for UP_LOCKED, DOWN_LOCKED, TRANSITIONING_UP, and TRANSITIONING_DOWN via the
operator interface.

**Implementation:**  
- `cli.py`
  - `StateAnnunciator` class
  - State change detection and output

**Verification:**  
- `test_fr003_state_annunciator_prints_required_states`
- `test_fr003_state_annunciator_prints_only_on_change`

**Evidence:**  
- Captured CLI output
- Automated stdout assertions

---

### LGCS-FR004 – Retract Rejection in Fault / Abnormal States
**Requirement:**  
The landing gear system shall ignore retract commands while in the FAULT or ABNORMAL states, never energising the actuator.

**Implementation:**  
- `landing_gear_controller.py`
  - State-based command rejection logic

**Verification:**  
- `test_fr004_retract_command_rejected_in_fault_or_abnormal`
- Spy-controller actuator energisation tests

**Evidence:**  
- Unit test results
- CLI command audit showing rejected retract attempts

---

## Performance Requirements (PR)

### LGCS-PR001 – Actuation Latency
**Requirement:**  
The landing gear system shall begin actuation of a deploy or retract command within 200 milliseconds of command acceptance.

**Implementation:**  
- `landing_gear_controller.py`
  - Actuation timestamp capture
  - Latency measurement API

**Verification:**  
- `TestPR001`
  - Boundary tests at 200 ms
  - Repeated command behaviour tests

---

### LGCS-PR002 – Transition Update Rate
**Requirement:**  
The landing gear system shall update gear state indications at a minimum rate of 10 Hz during TRANSITIONING_DOWN or TRANSITIONING_UP states.

**Implementation:**  
- `ControlLoop`
- Controller `update()` scheduling

**Verification:**  
- `TestPR002`
  - Fixed-rate and jittered update simulations

---

### LGCS-PR003 – Steady-State Update Rate
**Requirement:**  
The landing gear system shall update gear state indications at a minimum rate of 4 Hz during UP_LOCKED or DOWN_LOCKED states.

**Implementation:**  
- `ControlLoop`
- Steady-state monitoring logic

**Verification:**  
- `TestPR003`
  - Boundary and violation tests

---

### LGCS-PR004 – Fault Classification Latency
**Requirement:**  
The landing gear system shall complete fault detection and classification within 400 milliseconds of the fault exceeding qualification threshold.

**Implementation:**  
- `landing_gear_controller.py`
  - Persistent fault qualification timers
  - Classification timestamping

**Verification:**  
- `TestPR004`
  - Boundary tests at 400 ms

---

## Safety Requirements (SR)

### LGCS-SR001 – Auto-Deploy Below 1000 ft
**Requirement:**  
If the measured altitude drops below 1000 ft. under normal conditions and landing gear state ≠ DOWN_LOCKED, the landing gear system will automatically deploy the landing gear. 

**Implementation:**  
- Altitude-based deploy logic in `landing_gear_controller.py`

**Verification:**  
- `test_sr001_auto_deploy_when_altitude_below_1000ft`
- Boundary and invalid-input tests

---

### LGCS-SR002 – Low Altitude Warning
**Requirement:**
If the altitude drops below 2000ft. in normal conditions and landing gear state ≠ DOWN_LOCKED, a visual warning shall be delivered to the pilot. The visual warning shall contain the text ‘WARNING: ALTITUDE LOW - LANDING GEAR NOT DEPLOYED’.  

**Implementation:**  
- Warning annunciation logic in controller and CLI

**Verification:**  
- `test_sr002_warning_when_below_2000ft_and_gear_not_down`
- Threshold and inhibition tests

---

### LGCS-SR003 – WOW Retract Inhibition
**Requirement:**
The landing gear system shall inhibit retraction when weight-on-wheels = TRUE, regardless of pilot input. 

**Implementation:**  
- Weight-on-wheels command gating logic

**Verification:**  
- `test_sr003_inhibits_retract_when_weight_on_wheels_true`

---

### LGCS-SR004 – Power Loss Fail-Safe Behaviour
**Requirement:**
Following a loss of primary control power, the landing gear system shall default to DOWN_LOCKED state, overriding further input from the pilot interface while primary control power is not present. 

**Implementation:**  
- Primary power monitoring
- Forced deploy and command inhibition logic

**Verification:**  
- `test_sr004_power_loss_forces_deploy_from_up_locked`
- Power restoration tests

---

## Fault Tolerance & Handling Requirements (FTHR)

### LGCS-FTHR001 – Single Sensor Failure Tolerance
**Requirement:**
Upon detection of exactly one landing gear position sensor as FAILED, the system shall continue operation using remaining valid sensors and flag a maintenance fault. 

**Implementation:**  
- Position sensor fusion logic
- Maintenance fault tracking

**Verification:**  
- `test_fthr001_single_sensor_failure_continues_using_remaining_sensors`

---

### LGCS-FTHR002 – Persistent Sensor Conflict
**Requirement:**
Upon detection of conflicting gear position sensor inputs between 2 or more sensors, which persists longer than 500 milliseconds, the system shall enter the FAULT state and inhibit further gear commands.

**Implementation:**  
- Conflict detection timer
- FAULT state transition and command inhibition

**Verification:**  
- `test_fthr002_conflict_persisting_over_500ms_enters_fault`
- Conflict clearing tests

---

### LGCS-FTHR003 – Fault Recording
**Requirement:**
The landing gear system shall record all detected faults with timestamp and fault code in non-volatile memory. Fault codes shall be logged once per occurrence. 

**Implementation:**  
- `fault_recorder.py`
  - Timestamped fault logging
  - De-duplication logic

**Verification:**  
- `test_fthr003_records_fault_with_timestamp_and_code`
- `test_fthr003_does_not_duplicate_same_fault_code`

---

### LGCS-FTHR004 – RESET Validation Gate
**Requirement:**
Following entry to the RESET state, the landing gear system shall determine gear state based solely on validated sensor inputs before accepting further commands. 

**Implementation:**  
- RESET state handling
- Sensor validation before command acceptance

**Verification:**  
- `test_fthr004_reset_ignores_commands_until_sensors_validated`
- RESET recovery tests

---

## Simulation Support Components

### Position Sensor Simulator
- `sims/position_simulator.py`
- Verified for immutability, tolerance of invalid data, and equality semantics

### Altitude Simulator
- `sims/altitude_simulator.py`
- Verified for bounded motion, deterministic updates, and clock injection

---

## Integration and QA Tooling Evidence

- `integration_demo.ps1` or `integration_demo.sh` – End-to-end integration demonstration
- `pytest` – Automated unit and integration testing
- CLI command audit log – Operator action traceability
- Fault recorder log – Persistent fault evidence

---

## Notes
This traceability matrix demonstrates that all implemented requirements are:
- Explicitly mapped to software components
- Verified through deterministic automated tests
- Supported by simulation and tooling
- Auditable through logs and structured QA artefacts

This document supports academic assessment and does not represent
certified airborne software documentation.
