"""
Title: Landing Gear Deploy Command – Manual Test Script
Author: Alex Cooke
Date Created: 2026-01-09
Last Modified: 2026-01-09
Version: 1.0

Purpose:
Provides a lightweight, manual test harness for the landing gear deploy
command logic. The script exercises controller construction, deploy command
acceptance and rejection, and basic timing instrumentation. It supports
incremental development and early validation of deploy-related behaviour.

Targeted Requirements:
- None (supporting analysis or tooling only)

Scope and Limitations:
- Tests are executed manually and are not automated within a test framework
- Output is observed via console logging rather than structured assertions
- Several tests are expected to fail due to known implementation gaps
- No physical actuator, sensor, or fault modelling is included

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python 3.11+
- gear_configuration module
- landing gear controller module

Related Documents:
- Landing Gear Control System (LGCS) Requirements
- Academic assessment brief

Safety and Certification Disclaimer:
All artefacts in this repository are produced for academic assessment purposes only.
They do not represent certified software and must not be used in real-world aviation
or safety-critical systems.
"""

import time

# Update imports to match the controller module filename
from controller import LandingGearController, GearState
from gear_configuration import GearConfiguration


def banner(title: str) -> None:
    # Print a visual separator for test output
    print("\n" + "=" * 80)
    print(title)
    print("=" * 80)


def run_test(name: str, fn) -> None:
    # Execute a test function and report the result
    banner(f"TEST: {name}")
    try:
        fn()
        print(f"[PASS] {name}")
    except AssertionError as e:
        print(f"[FAIL] {name} (assertion): {e}")
    except Exception as e:
        print(f"[ERROR] {name} (exception): {type(e).__name__}: {e}")


def test_construct_without_config_should_error():
    # Constructor requires a GearConfiguration argument
    _ = LandingGearController()


def test_construct_with_config_should_work():
    # Controller creation with a configuration object
    cfg = GearConfiguration()
    controller = LandingGearController(cfg)
    assert controller.state == GearState.UP_LOCKED


def test_first_deploy_command_accepted():
    # Deploy command should be accepted in UP_LOCKED state
    cfg = GearConfiguration()
    controller = LandingGearController(cfg)

    ok = controller.command_gear_down()
    assert ok is True
    assert controller.state == GearState.TRANSITIONING_DOWN


def test_second_deploy_command_rejected():
    # Deploy command should be rejected when not in UP_LOCKED
    cfg = GearConfiguration()
    controller = LandingGearController(cfg)

    ok1 = controller.command_gear_down()
    ok2 = controller.command_gear_down()

    assert ok1 is True
    assert ok2 is False


def test_deploy_start_delay_is_timestamp_like():
    # deploy_start_delay stores a monotonic timestamp
    cfg = GearConfiguration()
    controller = LandingGearController(cfg)

    controller.command_gear_down()
    t1 = controller.deploy_start_delay

    time.sleep(0.01)
    t2 = time.monotonic()

    assert isinstance(t1, float)
    assert t2 >= t1


def test_log_formatting_placeholder_problem():
    # Log output currently contains unformatted placeholders
    cfg = GearConfiguration()
    controller = LandingGearController(cfg)

    controller.command_gear_down()
    controller.command_gear_down()


if __name__ == "__main__":
    run_test("Construct without config should error (expected)", test_construct_without_config_should_error)
    run_test("Construct with config should work", test_construct_with_config_should_work)
    run_test("First deploy command accepted", test_first_deploy_command_accepted)
    run_test("Second deploy command rejected", test_second_deploy_command_rejected)
    run_test("deploy_start_delay is timestamp-like", test_deploy_start_delay_is_timestamp_like)
    run_test("Log formatting placeholder problem (expected)", test_log_formatting_placeholder_problem)

    banner("DONE")
