import casadi as ca
from enum import IntEnum
import numpy as np
import matplotlib.pyplot as plt

from utils.helpers import in2m, interp_state_vector, interp_actions_vector


class BasePoseConstraint(object):
    def __init__(self, pose):
        self.pose = pose


class StoppedPoseConstraint(BasePoseConstraint):
    pass


class PoseConstraint(BasePoseConstraint):
    pass


class XConstraint(BasePoseConstraint):
    pass


class YConstraint(BasePoseConstraint):
    pass


class XYConstraint(BasePoseConstraint):
    pass


class StoppedXYConstraint(BasePoseConstraint):
    pass


class AngularConstraint(BasePoseConstraint):
    pass


class GoalConstraint(object):
    def __init__(self, goal_pose, shot_velocity=None):
        self.goal_pose = goal_pose
        self.shot_velocity = shot_velocity


class DistanceConstraint(object):
    def __init__(self, target_pose, distance):
        self.target_pose = target_pose
        self.distance = distance


class StayStoppedConstraint(object):
    def __init__(self, time):
        self.time = time


# Hacky way to enforce stop time
class SpeedConstraint(object):
    def __init__(self, speed):
        self.speed = speed


class InitializationConstraint(object):
    pass


class ActionType(IntEnum):
    NONE = 0
    # copied from 2024
    ENABLE_SHOOT = 1  # start launcher
    DISABLE_SHOOT = 2  # stop launcher
    DEPLOY_INTAKE = 3  # deploy intake and start rollers
    STOP_ROLLERS = 4  # stop rollers, don't retract intake
    RETRACT_INTAKE = 5  # retract intake and stop rollers
    STOP_ROLLERS_AND_ENABLE_SHOOT = (
        6  # stop rollers, don't retract intake, start launcher
    )
    DEPLOY_AND_ENABLE_SHOOT = 7  # deploy intake, start rollers, start launcher
    DEPLOY_AND_DISABLE_SHOOT = 8  # deploy intake, start rollers, stop launcher
    RETRACT_AND_DISABLE_SHOOT = 9  # retract intake, stop rollers, stop launcher
    RETRACT_AND_DISABLE_SHOOT_AND_CLIMB = (
        10  # retract intake, stop rollers, stop launcher, climb
    )


class Action(object):
    def __init__(self, action_type=ActionType.NONE, scoring_loc=None):
        self.action_type = action_type
        self.scoring_loc = scoring_loc


class QuikPlan(object):
    def __init__(
        self,
        field,
        robot,
        start_pose,
        start_constraints,
        start_action=Action(),
        apply_boundaries=False,
    ):
        self._field = field
        self._robot = robot
        self._apply_boundaries = apply_boundaries

        # Keep track of states as an initial guess
        self._states = np.zeros((1, 3))
        self._states[0, :] = start_pose

        # Dict of state_idx: action
        self._actions = {0: start_action}

        # List of the number of control intervals between each waypoint. There are |sum(Ns) + 1| states.
        self._Ns = []

        # List of (state_idx, constraint) tuples
        self._waypoint_idx = -1
        self._constraints = []
        for c in start_constraints:
            self._constraints.append((0, c, self._waypoint_idx))

    def add_waypoint(
        self,
        pose,
        N,
        intermediate_constraints=[],
        end_constraints=[],
        end_action=Action(),
    ):
        # Linearly interpolate to initialize states
        # TODO: Fix excessive copying
        last_state = self._states[-1, :]
        new_states = np.zeros((N, 3))
        new_states[:, 0] = np.linspace(last_state[0], pose[0], N + 1)[1:]
        new_states[:, 1] = np.linspace(last_state[1], pose[1], N + 1)[1:]
        new_states[:, 2] = np.linspace(last_state[2], pose[2], N + 1)[1:]
        self._states = np.vstack((self._states, new_states))

        # Store end action
        self._actions[len(self._states) - 1] = end_action

        # Save constraints
        self._waypoint_idx += 1
        start_N = sum(self._Ns)
        for i in range(start_N, start_N + N + 1):
            for c in intermediate_constraints:
                self._constraints.append((i, c, self._waypoint_idx))
            if i == start_N + N:
                for c in end_constraints:
                    self._constraints.append((i, c, self._waypoint_idx))

        # Update Ns
        self._Ns.append(N)

    def _plot_init(self):
        fig, ax = plt.subplots()
        self._field.plot_field(ax)

        num_states = self._states.shape[0]
        for i in range(num_states):
            state = self._states[i]
            self._robot.plot(ax, state)
        ax.set_title("Initialization")
        plt.show()

    def _compute_initial_velocities(self):
        """
        Compute initial velocity guesses based on trajectory geometry.
        Uses a simple approach that respects stopped constraints.
        """
        num_states = self._states.shape[0]
        init_xDot = np.zeros(num_states)
        init_yDot = np.zeros(num_states)
        init_thetaDot = np.zeros(num_states)

        # Very conservative speed for initial guess
        CRUISE_SPEED = 1.0  # m/s
        CRUISE_ANGULAR_SPEED = 1.0  # rad/s

        # Process each segment
        start_idx = 0
        for seg_idx, n in enumerate(self._Ns):
            end_idx = start_idx + n

            # Get segment endpoints
            x0, y0, theta0 = self._states[start_idx, :]
            x1, y1, theta1 = self._states[end_idx, :]

            # Compute total displacement
            dx = x1 - x0
            dy = y1 - y0
            dtheta = theta1 - theta0

            # Linear distance
            linear_dist = np.sqrt(dx**2 + dy**2)

            # Check if start/end are stopped (based on constraints)
            start_stopped = self._is_state_stopped(start_idx)
            end_stopped = self._is_state_stopped(end_idx)

            # Only provide velocity hints for middle of segments (not at endpoints with constraints)
            if linear_dist > 0.01 and n > 2:  # Non-trivial motion with enough points
                # Direction of motion
                direction_x = dx / linear_dist
                direction_y = dy / linear_dist

                # Simple smooth profile that respects constraints
                for i in range(1, n):  # Skip first and last points
                    t_normalized = i / n  # 0 to 1

                    # Simple smooth profile using sine
                    speed_scale = np.sin(t_normalized * np.pi)  # 0 at ends, 1 at middle

                    # Reduce speed near endpoints if they're stopped
                    if start_stopped or end_stopped:
                        speed_scale *= 0.5  # Be more conservative

                    speed = CRUISE_SPEED * speed_scale
                    init_xDot[start_idx + i] = direction_x * speed
                    init_yDot[start_idx + i] = direction_y * speed

            # Angular velocity profile (similarly conservative)
            if abs(dtheta) > 0.01 and n > 2:
                direction_theta = np.sign(dtheta)

                for i in range(1, n):  # Skip first and last points
                    t_normalized = i / n

                    # Smooth profile using sine
                    ang_speed_scale = np.sin(t_normalized * np.pi)

                    if start_stopped or end_stopped:
                        ang_speed_scale *= 0.5

                    ang_speed = CRUISE_ANGULAR_SPEED * ang_speed_scale
                    init_thetaDot[start_idx + i] = direction_theta * ang_speed

            start_idx = end_idx

        return init_xDot, init_yDot, init_thetaDot

    def _compute_initial_accelerations(self, init_xDot, init_yDot, init_thetaDot):
        """
        Compute initial acceleration guesses based on velocity changes.
        Uses simple finite differences with conservative estimates.
        """
        num_controls = len(init_xDot) - 1
        init_xDotDot = np.zeros(num_controls)
        init_yDotDot = np.zeros(num_controls)
        init_thetaDotDot = np.zeros(num_controls)

        # Use finite differences with a reasonable dt estimate
        # Estimate based on segment times
        for i in range(num_controls):
            # Simple finite difference, but keep magnitudes small
            # The optimizer will adjust these anyway
            dt_guess = 0.1  # Conservative time step estimate

            dvx = init_xDot[i + 1] - init_xDot[i]
            dvy = init_yDot[i + 1] - init_yDot[i]
            dvtheta = init_thetaDot[i + 1] - init_thetaDot[i]

            # Limit acceleration magnitudes to reasonable values
            MAX_ACCEL = 3.0  # m/s^2
            MAX_ANG_ACCEL = 2.0  # rad/s^2

            init_xDotDot[i] = np.clip(dvx / dt_guess, -MAX_ACCEL, MAX_ACCEL)
            init_yDotDot[i] = np.clip(dvy / dt_guess, -MAX_ACCEL, MAX_ACCEL)
            init_thetaDotDot[i] = np.clip(
                dvtheta / dt_guess, -MAX_ANG_ACCEL, MAX_ANG_ACCEL
            )

        return init_xDotDot, init_yDotDot, init_thetaDotDot

    def _is_state_stopped(self, state_idx):
        """
        Check if a state index has a stopped constraint.
        """
        for idx, constraint, _ in self._constraints:
            if idx == state_idx:
                if type(constraint) in {StoppedPoseConstraint, StoppedXYConstraint}:
                    return True
        return False

    def plan(self, quiet=False):
        # Construct optimization problem
        opti = ca.Opti()
        N = sum(self._Ns)

        # State variables
        X = opti.variable(6, N + 1)
        xpos = X[0, :]
        ypos = X[1, :]
        theta = X[2, :]
        xDot = X[3, :]
        yDot = X[4, :]
        thetaDot = X[5, :]

        # Control variables
        U = opti.variable(3, N)
        xDotDot = U[0, :]
        yDotDot = U[1, :]
        thetaDotDot = U[2, :]

        # Total time variable per segment
        Ts = []
        dts = []
        AVG_SPEED = 1.5  # m/s - conservative average speed for time estimation
        AVG_ANGULAR_SPEED = 2.0  # rad/s
        SAFETY_MULTIPLIER = 1.5  # overestimate is safer than underestimate
        MIN_SEGMENT_TIME = 1.0  # s - minimum time for any segment
        start_idx = 0
        for n in self._Ns:
            T = opti.variable()
            dt = T / n
            Ts.append(T)
            dts.append(dt)

            # Estimate segment time from distance and angular change
            end_idx = start_idx + n
            dx = self._states[end_idx, 0] - self._states[start_idx, 0]
            dy = self._states[end_idx, 1] - self._states[start_idx, 1]
            dtheta = abs(self._states[end_idx, 2] - self._states[start_idx, 2])
            linear_dist = np.sqrt(dx * dx + dy * dy)
            T_init = SAFETY_MULTIPLIER * max(
                linear_dist / AVG_SPEED, dtheta / AVG_ANGULAR_SPEED, MIN_SEGMENT_TIME
            )

            # Apply time constraint & initial guess
            opti.subject_to(T >= 0)
            opti.set_initial(T, T_init)
            start_idx = end_idx

        total_time = sum(Ts)

        # Compute jerk cost using finite differences
        JERK_WEIGHT = 1e-5
        jerk_cost = 0.0

        start_n = 0
        for n, dt in zip(self._Ns, dts):
            end_n = start_n + n
            for k in range(start_n, end_n - 1):
                jerk_x = (xDotDot[k + 1] - xDotDot[k]) / dt
                jerk_y = (yDotDot[k + 1] - yDotDot[k]) / dt
                jerk_theta = (thetaDotDot[k + 1] - thetaDotDot[k]) / dt
                jerk_cost += dt * (
                    jerk_x * jerk_x + jerk_y * jerk_y + jerk_theta * jerk_theta
                )
            start_n = end_n

        opti.minimize(total_time + JERK_WEIGHT * jerk_cost)

        # Apply dynamic constraints using Hermite-Simpson collocation
        start_n = 0
        for n, dt in zip(self._Ns, dts):
            end_n = start_n + n
            for k in range(start_n, end_n):
                # States at interval boundaries
                x_k = X[:, k]
                x_kp1 = X[:, k + 1]
                u_k = U[:, k]

                # Dynamics evaluations at boundaries
                f_k = self._robot.dynamics_model(x_k, u_k)
                f_kp1 = self._robot.dynamics_model(x_kp1, u_k)

                # Midpoint state (cubic Hermite interpolation)
                x_mid = 0.5 * (x_k + x_kp1) + dt / 8 * (f_k - f_kp1)

                # Dynamics evaluation at midpoint
                f_mid = self._robot.dynamics_model(x_mid, u_k)

                # Hermite-Simpson collocation constraint
                # Uses Simpson's rule for integration: (f_k + 4*f_mid + f_kp1) / 6
                x_interp = x_k + dt / 6 * (f_k + 4 * f_mid + f_kp1)
                opti.subject_to(x_kp1 == x_interp)
            start_n = end_n

        # # Apply whole robot limits
        # LINEAR_VEL_LIMIT = 4.0  # m/s
        # ROTATIONAL_VEL_LIMIT = 2.0 * np.pi  # rad/s
        # # Velocity magnitude constraint (squared norm is always non-negative)
        # opti.subject_to(xDot * xDot + yDot * yDot <= LINEAR_VEL_LIMIT * LINEAR_VEL_LIMIT)
        # opti.subject_to(thetaDot * thetaDot <= ROTATIONAL_VEL_LIMIT * ROTATIONAL_VEL_LIMIT)

        # LINEAR_ACCEL_LIMIT = 8.0  # m/s/s
        # ROTATIONAL_ACCEL_LIMIT = 4.0 * np.pi  # rad/s/s
        # # Acceleration magnitude constraint (squared norm is always non-negative)
        # opti.subject_to(xDotDot * xDotDot + yDotDot * yDotDot <= LINEAR_ACCEL_LIMIT * LINEAR_ACCEL_LIMIT)
        # opti.subject_to(thetaDotDot * thetaDotDot <= ROTATIONAL_ACCEL_LIMIT * ROTATIONAL_ACCEL_LIMIT)

        # Apply module torque/friction constraints
        module_data = self._robot.apply_module_constraints(opti, X, U, N)

        if self._apply_boundaries:
            # Apply field boundary constraints
            opti.subject_to(
                opti.bounded(
                    self._robot.WIDTH * 0.5,
                    xpos,
                    self._field.LENGTH - self._robot.WIDTH * 0.5,
                )
            )
            opti.subject_to(
                opti.bounded(
                    self._robot.WIDTH * 0.5,
                    ypos,
                    self._field.WIDTH - self._robot.WIDTH * 0.5,
                )
            )

        # Apply state constraints
        for i, constraint, waypoint_idx in self._constraints:
            if type(constraint) in {StoppedPoseConstraint, PoseConstraint}:
                opti.subject_to(X[0, i] == constraint.pose[0])
                opti.subject_to(X[1, i] == constraint.pose[1])
            if type(constraint) in {
                StoppedPoseConstraint,
                PoseConstraint,
                AngularConstraint,
            }:
                opti.subject_to(X[2, i] == constraint.pose[2])
            if type(constraint) in {StoppedPoseConstraint, StoppedXYConstraint}:
                opti.subject_to(X[3, i] == 0.0)
                opti.subject_to(X[4, i] == 0.0)
                opti.subject_to(X[5, i] == 0.0)
            if type(constraint) in {XConstraint, XYConstraint, StoppedXYConstraint}:
                opti.subject_to(X[0, i] == constraint.pose[0])
            if type(constraint) in {YConstraint, XYConstraint, StoppedXYConstraint}:
                opti.subject_to(X[1, i] == constraint.pose[1])
            if type(constraint) == SpeedConstraint:
                self._robot.apply_speed_constraint(opti, X, U, i, constraint.speed)
            if type(constraint) == StayStoppedConstraint:
                if i > 0:
                    opti.subject_to(X[0:3, i] == X[0:3, i - 1])
                    opti.subject_to(Ts[waypoint_idx] == constraint.time)
            if type(constraint) == InitializationConstraint:
                opti.subject_to(Ts[waypoint_idx] == 0.0)
            if type(constraint) == GoalConstraint:
                if constraint.shot_velocity is not None:
                    # Shot velocity vector + robot velocity vector should point at the goal.
                    shot_xDot = constraint.shot_velocity * ca.cos(theta[i])
                    shot_yDot = constraint.shot_velocity * ca.sin(theta[i])
                    # Combined velocity component
                    vel_sum_x = xDot[i] + shot_xDot
                    vel_sum_y = yDot[i] + shot_yDot
                else:
                    # Point directly at the goal.
                    vel_sum_x = ca.cos(theta[i])
                    vel_sum_y = ca.sin(theta[i])

                goal_dx = constraint.goal_pose[0] - X[0, i]
                goal_dy = constraint.goal_pose[1] - X[1, i]

                # Constrain combined velocity to be parallel to goal vector using cross product
                # Cross product = 0 means vectors are parallel (avoids division and atan2)
                opti.subject_to(vel_sum_x * goal_dy == vel_sum_y * goal_dx)

                # Ensure they point in the same direction (not opposite) using dot product
                opti.subject_to(vel_sum_x * goal_dx + vel_sum_y * goal_dy >= 0)
            if type(constraint) == DistanceConstraint:
                target_dx = constraint.target_pose[0] - X[0, i]
                target_dy = constraint.target_pose[1] - X[1, i]
                dist_sq = target_dx * target_dx + target_dy * target_dy
                opti.subject_to(dist_sq == constraint.distance * constraint.distance)

        # Apply obstacle constraints
        self._robot.apply_obstacle_constraints(
            opti, xpos, ypos, theta, self._field.OBSTACLES
        )

        # Improve initial guess for theta at states with GoalConstraints
        for i, constraint, _ in self._constraints:
            if type(constraint) == GoalConstraint:
                # Set initial theta to point at the goal
                dx = constraint.goal_pose[0] - self._states[i, 0]
                dy = constraint.goal_pose[1] - self._states[i, 1]
                goal_angle = np.arctan2(dy, dx)
                # Find equivalent angle closest to current initial guess
                current_theta = self._states[i, 2]
                k = int(np.round((current_theta - goal_angle) / (2 * np.pi)))
                self._states[i, 2] = goal_angle + 2 * np.pi * k

        if not quiet:
            self._plot_init()

        # Set initial guess with improved velocity and acceleration estimates
        opti.set_initial(xpos, self._states[:, 0])
        opti.set_initial(ypos, self._states[:, 1])
        opti.set_initial(theta, self._states[:, 2])

        # Generate better initial guesses for velocities and accelerations
        init_xDot, init_yDot, init_thetaDot = self._compute_initial_velocities()
        (
            init_xDotDot,
            init_yDotDot,
            init_thetaDotDot,
        ) = self._compute_initial_accelerations(init_xDot, init_yDot, init_thetaDot)

        opti.set_initial(xDot, init_xDot)
        opti.set_initial(yDot, init_yDot)
        opti.set_initial(thetaDot, init_thetaDot)
        opti.set_initial(xDotDot, init_xDotDot)
        opti.set_initial(yDotDot, init_yDotDot)
        opti.set_initial(thetaDotDot, init_thetaDotDot)

        # Solve with relaxed tolerances for faster convergence
        p_opts = {
            "expand": True,  # Expand the NLP function in terms of scalar operations
        }
        s_opts = {
            "max_iter": 200,
            "tol": 1e-4,
            "constr_viol_tol": 1e-4,
            "acceptable_tol": 1e-3,
            "print_level": 0 if quiet else 5,
        }
        opti.solver("ipopt", p_opts, s_opts)
        sol = opti.solve()
        for t in Ts:
            print(sol.value(t))
        print(f"Total time: {sol.value(total_time)}")

        # Interpolate result
        times = [0.0]
        action_idx = 0
        start_action = self._actions.get(0, Action())
        actions = [start_action]
        for n, t, dt in zip(self._Ns, Ts, dts):
            times += list(
                np.linspace(times[-1] + sol.value(dt), times[-1] + sol.value(t), n)
            )
            actions += (n - 1) * [Action()]

            action_idx += n
            action = self._actions.get(action_idx, Action())
            actions.append(action)

        # Use cubic Hermite interpolation with derivative information for better accuracy
        # For positions, derivatives are velocities; for velocities, derivatives are accelerations
        interp_times, interp_x = interp_state_vector(
            times, sol.value(xpos), 0.02, sol.value(xDot)
        )
        _, interp_y = interp_state_vector(times, sol.value(ypos), 0.02, sol.value(yDot))
        _, interp_theta = interp_state_vector(
            times, sol.value(theta), 0.02, sol.value(thetaDot)
        )

        # For velocities, we need to extract accelerations at grid points
        # Accelerations are control variables, so we need to handle the mismatch in size
        # We'll use the control value for the interval and extend by one for the last point
        xDotDot_extended = np.append(sol.value(xDotDot), sol.value(xDotDot)[-1])
        yDotDot_extended = np.append(sol.value(yDotDot), sol.value(yDotDot)[-1])
        thetaDotDot_extended = np.append(
            sol.value(thetaDotDot), sol.value(thetaDotDot)[-1]
        )

        _, interp_xDot = interp_state_vector(
            times, sol.value(xDot), 0.02, xDotDot_extended
        )
        _, interp_yDot = interp_state_vector(
            times, sol.value(yDot), 0.02, yDotDot_extended
        )
        _, interp_thetaDot = interp_state_vector(
            times, sol.value(thetaDot), 0.02, thetaDotDot_extended
        )
        interp_action_type, interp_grid_id, interp_node_id = interp_actions_vector(
            times, actions, 0.02, Action()
        )

        if not quiet:
            # Plot velocities, velocity headings, and forces in 2x3 subplot
            fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(
                2, 3, figsize=(18, 8)
            )

            # Subplot 1: Module Velocities
            _, interp_v0s = interp_state_vector(
                times, [sol.value(v) for v in module_data.velocities[0]], 0.02
            )
            _, interp_v1s = interp_state_vector(
                times, [sol.value(v) for v in module_data.velocities[1]], 0.02
            )
            _, interp_v2s = interp_state_vector(
                times, [sol.value(v) for v in module_data.velocities[2]], 0.02
            )
            _, interp_v3s = interp_state_vector(
                times, [sol.value(v) for v in module_data.velocities[3]], 0.02
            )
            ax1.plot(interp_times, interp_v0s, label="FL")
            ax1.plot(interp_times, interp_v1s, label="BL")
            ax1.plot(interp_times, interp_v2s, label="BR")
            ax1.plot(interp_times, interp_v3s, label="FR")
            ax1.legend(loc="upper left")
            ax1.set_xlabel("Time (s)")
            ax1.set_ylabel("Velocity (m/s)")
            ax1.set_title("Module Velocity")
            ax1.grid(True, alpha=0.3)

            # Subplot 2: Module Headings
            _, interp_h0s = interp_state_vector(
                times, [sol.value(h) for h in module_data.headings[0]], 0.02
            )
            _, interp_h1s = interp_state_vector(
                times, [sol.value(h) for h in module_data.headings[1]], 0.02
            )
            _, interp_h2s = interp_state_vector(
                times, [sol.value(h) for h in module_data.headings[2]], 0.02
            )
            _, interp_h3s = interp_state_vector(
                times, [sol.value(h) for h in module_data.headings[3]], 0.02
            )
            ax2.plot(interp_times, np.rad2deg(interp_h0s), label="FL")
            ax2.plot(interp_times, np.rad2deg(interp_h1s), label="BL")
            ax2.plot(interp_times, np.rad2deg(interp_h2s), label="BR")
            ax2.plot(interp_times, np.rad2deg(interp_h3s), label="FR")
            ax2.legend(loc="upper left")
            ax2.set_xlabel("Time (s)")
            ax2.set_ylabel("Heading (deg)")
            ax2.set_title("Module Heading")
            ax2.grid(True, alpha=0.3)

            # Subplot 3: Hide for now (could be used for other data)
            ax3.axis("off")

            # Subplot 4: Module Forces (Total)
            interp_times_2, interp_f0s = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces[0]], 0.02
            )
            _, interp_f1s = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces[1]], 0.02
            )
            _, interp_f2s = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces[2]], 0.02
            )
            _, interp_f3s = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces[3]], 0.02
            )
            ax4.plot(interp_times_2, interp_f0s, label="FL")
            ax4.plot(interp_times_2, interp_f1s, label="BL")
            ax4.plot(interp_times_2, interp_f2s, label="BR")
            ax4.plot(interp_times_2, interp_f3s, label="FR")

            # Add force limit line
            from utils.robot import G

            max_friction_force = (
                self._robot.MU * self._robot.MASS * G / self._robot.NUM_MODULES
            )
            force_limit = min(self._robot.WHEEL_MAX_FORCE, max_friction_force)
            ax4.axhline(
                y=force_limit,
                color="r",
                linestyle="--",
                linewidth=1.5,
                label="Force Limit",
                alpha=0.7,
            )

            ax4.legend(loc="upper left")
            ax4.set_xlabel("Time (s)")
            ax4.set_ylabel("Force (N)")
            ax4.set_title("Module Force (Total)")
            ax4.grid(True, alpha=0.3)

            # Subplot 5: Module Forces (Longitudinal)
            _, interp_f0_lon = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces_lon[0]], 0.02
            )
            _, interp_f1_lon = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces_lon[1]], 0.02
            )
            _, interp_f2_lon = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces_lon[2]], 0.02
            )
            _, interp_f3_lon = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces_lon[3]], 0.02
            )
            ax5.plot(interp_times_2, interp_f0_lon, label="FL")
            ax5.plot(interp_times_2, interp_f1_lon, label="BL")
            ax5.plot(interp_times_2, interp_f2_lon, label="BR")
            ax5.plot(interp_times_2, interp_f3_lon, label="FR")
            ax5.axhline(y=0, color="k", linestyle="-", linewidth=0.5, alpha=0.3)
            ax5.axhline(
                y=force_limit,
                color="r",
                linestyle="--",
                linewidth=1.5,
                label="Force Limit",
                alpha=0.7,
            )
            ax5.axhline(
                y=-force_limit, color="r", linestyle="--", linewidth=1.5, alpha=0.7
            )
            ax5.legend(loc="upper left")
            ax5.set_xlabel("Time (s)")
            ax5.set_ylabel("Force (N)")
            ax5.set_title("Module Force (Longitudinal)")
            ax5.grid(True, alpha=0.3)

            # Subplot 6: Module Forces (Lateral)
            _, interp_f0_lat = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces_lat[0]], 0.02
            )
            _, interp_f1_lat = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces_lat[1]], 0.02
            )
            _, interp_f2_lat = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces_lat[2]], 0.02
            )
            _, interp_f3_lat = interp_state_vector(
                times[:-1], [sol.value(f) for f in module_data.forces_lat[3]], 0.02
            )
            ax6.plot(interp_times_2, interp_f0_lat, label="FL")
            ax6.plot(interp_times_2, interp_f1_lat, label="BL")
            ax6.plot(interp_times_2, interp_f2_lat, label="BR")
            ax6.plot(interp_times_2, interp_f3_lat, label="FR")
            ax6.axhline(y=0, color="k", linestyle="-", linewidth=0.5, alpha=0.3)
            ax6.axhline(
                y=force_limit,
                color="r",
                linestyle="--",
                linewidth=1.5,
                label="Force Limit",
                alpha=0.7,
            )
            ax6.axhline(
                y=-force_limit, color="r", linestyle="--", linewidth=1.5, alpha=0.7
            )
            ax6.legend(loc="upper left")
            ax6.set_xlabel("Time (s)")
            ax6.set_ylabel("Force (N)")
            ax6.set_title("Module Force (Lateral)")
            ax6.grid(True, alpha=0.3)

            plt.tight_layout()

        return np.transpose(
            np.vstack(
                [
                    interp_times,
                    interp_x,
                    interp_y,
                    interp_theta,
                    interp_xDot,
                    interp_yDot,
                    interp_thetaDot,
                    interp_action_type,
                    interp_grid_id,
                    interp_node_id,
                ]
            )
        )
