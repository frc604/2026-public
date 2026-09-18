#!/usr/bin/env python3
"""
End-to-End Simulation & Telemetry Verification Test
Automates the workflow: Scenario Input -> Robot Simulation -> .wpilog Generation -> Telemetry Extraction & Verification.
"""

import pytest
import os
import sys
import subprocess
import glob
import json
import time


@pytest.fixture
def scenario_file():
    """Fixture to set up the simulation scenario file and ensure clean teardown."""
    scenario_content = {
        "alliance": "blue",
        "steps": [
            {
                "time_seconds": 0.0,
                "mode": "teleop",
                "enabled": True,
                "joysticks": {
                    "0": {
                        "axes": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "buttons": [False] * 10,
                        "povs": [-1],
                    }
                },
            },
            {
                "time_seconds": 0.5,
                "mode": "teleop",
                "enabled": True,
                "joysticks": {
                    "0": {
                        "axes": [
                            0.0,
                            -0.6,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],  # Drive forward at 60% speed
                        "buttons": [False] * 10,
                        "povs": [-1],
                    }
                },
            },
            {
                "time_seconds": 1.8,
                "mode": "teleop",
                "enabled": True,
                "joysticks": {
                    "0": {
                        "axes": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Stop driving
                        "buttons": [False] * 10,
                        "povs": [-1],
                    }
                },
            },
            {
                "time_seconds": 2.0,
                "mode": "teleop",
                "enabled": True,
                "joysticks": {
                    "0": {
                        "axes": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "buttons": [False] * 10,
                        "povs": [-1],
                    }
                },
            },
        ],
    }

    filename = "scenario_ci_temp.json"
    with open(filename, "w") as f:
        json.dump(scenario_content, f, indent=2)

    yield filename

    if os.path.exists(filename):
        os.remove(filename)


def test_simulation_drivetrain_displacement(scenario_file):
    """E2E Test: Run simulation with simulation scenario and verify robot displaced >= 0.5 meters."""
    start_time = time.time()

    # Run Headless Simulation
    print(f"Starting simulation with scenario file: {scenario_file}...")
    env = os.environ.copy()
    env["SIM_SCENARIO"] = scenario_file

    result = subprocess.run(
        ["./gradlew", "simulateJava", "-Pnogui"],
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )
    print(f"Robot simulation stdout:\n{result.stdout}")
    print(f"Robot simulation stderr:\n{result.stderr}")
    assert result.returncode == 0, "Robot simulation exited with errors"

    # Find newly created log file
    log_files = glob.glob("logs/*.wpilog")
    new_log = None
    for filepath in log_files:
        mtime = os.path.getmtime(filepath)
        if mtime >= start_time - 10:
            new_log = filepath
            break

    assert (
        new_log is not None
    ), "Error: No new .wpilog file was generated in logs/ directory."
    print(f"Recorded telemetry log discovered: {new_log}")

    try:
        # Verify telemetry using read_wpilog
        verify_cmd = [
            sys.executable,
            "offboard/scripts/read_wpilog.py",
            new_log,
            "-t",
            "/RealOutputs/Swerve/Odometry",
            "-f",
            "json",
        ]
        verify_result = subprocess.run(
            verify_cmd, capture_output=True, text=True, check=True
        )
        data = json.loads(verify_result.stdout)

        topics_data = data.get("topics", {})
        odometry_samples = topics_data.get("/RealOutputs/Swerve/Odometry", {}).get(
            "samples", []
        )

        assert (
            len(odometry_samples) >= 2
        ), f"Insufficient swerve odometry samples recorded. Count: {len(odometry_samples)}"

        first_pose = odometry_samples[0].get("v", {})
        last_pose = odometry_samples[-1].get("v", {})

        x0, y0 = first_pose.get("x", 0.0), first_pose.get("y", 0.0)
        x1, y1 = last_pose.get("x", 0.0), last_pose.get("y", 0.0)

        displacement = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        print(f"Start Pose: X={x0:.3f}, Y={y0:.3f}")
        print(f"End Pose:   X={x1:.3f}, Y={y1:.3f}")
        print(f"Calculated displacement: {displacement:.3f} meters")

        # Assert that the robot moved forward by at least 0.5 meters
        assert (
            displacement >= 0.5
        ), f"Telemetry validation failed. Robot displacement ({displacement:.3f} m) was below the threshold of 0.5 m."

    except subprocess.CalledProcessError as e:
        print(f"Verification subprocess failed with exit code {e.returncode}")
        print(f"Stdout:\n{e.stdout}")
        print(f"Stderr:\n{e.stderr}")
        try:
            print("Listing all topics in the generated log file for debugging:")
            list_cmd = [
                sys.executable,
                "offboard/scripts/read_wpilog.py",
                new_log,
                "--list",
            ]
            subprocess.run(list_cmd, check=True)
        except Exception as list_err:
            print(f"Failed to list topics: {list_err}")
        raise

    finally:
        # Teardown: Clean up the generated telemetry log
        if new_log and os.path.exists(new_log):
            os.remove(new_log)
