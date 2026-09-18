import csv
import numpy as np
import os


def in2m(inches):
    # Inches to meters
    return inches * 2.54 / 100.0


def rotate_around_origin(point, theta):
    x, y = point
    return (
        x * np.cos(theta) - y * np.sin(theta),
        y * np.cos(theta) + x * np.sin(theta),
    )


def transform_geometry(geometry, pose):
    x, y, theta = pose
    transformed_geometry = []
    for point1, point2 in geometry:
        new_point1 = rotate_around_origin(point1, theta) + np.array([x, y])
        new_point2 = rotate_around_origin(point2, theta) + np.array([x, y])
        transformed_geometry.append((new_point1, new_point2))
    return transformed_geometry


def interp_state_vector(times, states, new_dt, derivatives=None):
    """
    Interpolate state vector using cubic Hermite interpolation if derivatives are provided,
    otherwise fall back to linear interpolation.

    Args:
        times: Array of time points
        states: Array of state values at those times
        new_dt: Time step for interpolated output
        derivatives: Optional array of derivatives (velocities) at each time point

    Returns:
        interp_times: Interpolated time array
        interp_states: Interpolated state values
    """
    num_points = int(np.ceil((times[-1] - times[0]) / new_dt)) + 1
    interp_times = np.linspace(times[0], times[-1], num_points)

    if derivatives is None:
        # Fall back to linear interpolation if no derivatives provided
        return interp_times, np.interp(interp_times, times, states)

    # Use cubic Hermite interpolation with derivative information
    interp_states = np.zeros_like(interp_times)

    for i, t in enumerate(interp_times):
        # Find the interval containing this time
        idx = np.searchsorted(times, t, side="right") - 1
        idx = max(0, min(idx, len(times) - 2))  # Clamp to valid range

        # Get interval endpoints
        t0 = times[idx]
        t1 = times[idx + 1]
        x0 = states[idx]
        x1 = states[idx + 1]
        v0 = derivatives[idx]  # derivative at t0
        v1 = derivatives[idx + 1]  # derivative at t1

        # Normalize time to [0, 1]
        dt = t1 - t0
        if dt < 1e-10:  # Handle near-zero dt
            interp_states[i] = x0
            continue

        tau = (t - t0) / dt

        # Cubic Hermite basis functions (from Hermite-Simpson collocation)
        h00 = 2 * tau**3 - 3 * tau**2 + 1  # Value at t0
        h10 = tau**3 - 2 * tau**2 + tau  # Derivative at t0
        h01 = -2 * tau**3 + 3 * tau**2  # Value at t1
        h11 = tau**3 - tau**2  # Derivative at t1

        # Interpolate using Hermite polynomial
        interp_states[i] = h00 * x0 + h10 * dt * v0 + h01 * x1 + h11 * dt * v1

    return interp_times, interp_states


def interp_actions_vector(times, actions, new_dt, null_action):
    num_points = int(np.ceil((times[-1] - times[0]) / new_dt)) + 1
    interp_times = np.linspace(times[0], times[-1], num_points)

    interp_action_type = []
    interp_grid_id = []
    interp_node_id = []
    action_idx = 0
    for time in interp_times:
        if action_idx < len(actions) and time >= times[action_idx]:
            interp_action_type.append(int(actions[action_idx].action_type))
            scoring_loc = actions[action_idx].scoring_loc
            if scoring_loc is not None:
                interp_grid_id.append(scoring_loc[0])
                interp_node_id.append(scoring_loc[1])
            else:
                interp_grid_id.append(-1)
                interp_node_id.append(-1)
            action_idx += 1
        else:
            interp_action_type.append(int(null_action.action_type))
            interp_grid_id.append(-1)
            interp_node_id.append(-1)

    return interp_action_type, interp_grid_id, interp_node_id


def write_to_csv(traj, name):
    with open(
        os.path.join(
            os.path.dirname(__file__), "../../../src/main/deploy/{}.csv".format(name)
        ),
        "w",
        newline="",
    ) as outfile:
        writer = csv.writer(outfile, delimiter=",")
        formatted_traj = [[round(val, 3) for val in row] for row in traj]
        writer.writerows(formatted_traj)
    outfile.close()
