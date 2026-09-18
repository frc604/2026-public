import casadi as ca
import matplotlib as mpl
import matplotlib.patches as patches
import numpy as np
from collections import namedtuple

from utils.helpers import in2m, rotate_around_origin, transform_geometry

G = 9.81  # m/s/s
MODULE_X_DIST = in2m(11.0)  # m
MODULE_Y_DIST = in2m(11.0)  # m

# Named tuple to store module data
ModuleData = namedtuple(
    "ModuleData", ["velocities", "headings", "forces", "forces_lon", "forces_lat"]
)


class Robot(object):
    def __init__(self, MAX_RPM=4000, MAX_TORQUE=0.8):
        # Geometry
        self.WIDTH = in2m(34.0)  # m
        self.LENGTH = in2m(34.0)  # m
        self.GEOMETRY = [
            [
                (self.LENGTH / 2, self.WIDTH / 2),
                (self.LENGTH / 2, -self.WIDTH / 2),
            ],  # Front
            [
                (-self.LENGTH / 2, self.WIDTH / 2),
                (-self.LENGTH / 2, -self.WIDTH / 2),
            ],  # Back
            [
                (self.LENGTH / 2, self.WIDTH / 2),
                (-self.LENGTH / 2, self.WIDTH / 2),
            ],  # Left
            [
                (self.LENGTH / 2, -self.WIDTH / 2),
                (-self.LENGTH / 2, -self.WIDTH / 2),
            ],  # Right
        ]
        self.OBSTACLE_BUFFER = 0.1  # m
        self.WHEEL_DIA = in2m(3.94)  # m
        self.MODULE_POSITIONS = [
            (MODULE_X_DIST, MODULE_Y_DIST),  # ID 0: FL
            (-MODULE_X_DIST, MODULE_Y_DIST),  # ID 1: BL
            (-MODULE_X_DIST, -MODULE_Y_DIST),  # ID 2: BR
            (MODULE_X_DIST, -MODULE_Y_DIST),  # ID 3: FR
        ]
        self.NUM_MODULES = len(self.MODULE_POSITIONS)

        # Mass / Inertia
        self.MU = 0.8
        self.MASS = 60.0  # kg
        self.J = (
            self.MASS * (self.LENGTH**2 + self.WIDTH**2)
        ) / 12.0  # Moment of inertia
        self.J *= 1.5  # Conservative scaling

        # Motors
        self.MOTOR_MAX_TORQUE = MAX_TORQUE  # N*m @ 80A
        self.MOTOR_MAX_RPM = MAX_RPM  # RPM @ 10V
        self.DRIVE_RATIO = (54.0 / 10.0) * (16.0 / 40.0) * (45.0 / 15.0)  # 6.48:1
        self.WHEEL_MAX_FORCE = (
            self.MOTOR_MAX_TORQUE * self.DRIVE_RATIO / (self.WHEEL_DIA / 2.0)
        )

    def plot(
        self,
        ax,
        state,
        robot_axis_plot_size=0.1,
        color="k",
        alpha=1.0,
        linestyle="-",
        fill=False,
    ):
        pose = state[:3]
        x, y, theta = pose

        if fill:
            # Create a rectangle for the robot
            rect_points = [
                (self.LENGTH / 2, self.WIDTH / 2),
                (self.LENGTH / 2, -self.WIDTH / 2),
                (-self.LENGTH / 2, -self.WIDTH / 2),
                (-self.LENGTH / 2, self.WIDTH / 2),
                (self.LENGTH / 2, self.WIDTH / 2),
            ]

            # Transform rectangle points manually
            transformed_points = []
            for px, py in rect_points:
                nx, ny = rotate_around_origin((px, py), theta)
                transformed_points.append((nx + x, ny + y))

            # Create lighter fill polygon
            fill_alpha = alpha * 0.3
            polygon = patches.Polygon(
                transformed_points,
                closed=True,
                color=color,
                alpha=fill_alpha,
                fill=True,
            )
            ax.add_patch(polygon)

        # Plot robot geometry outline
        ax.add_collection(
            mpl.collections.LineCollection(
                transform_geometry(self.GEOMETRY, pose),
                color=color,
                alpha=alpha,
                linestyle=linestyle,
            )
        )

        # Plot robot axes
        ax.add_collection(
            mpl.collections.LineCollection(
                transform_geometry([[(0, 0), (robot_axis_plot_size, 0)]], pose),
                color="r",
            )
        )
        ax.add_collection(
            mpl.collections.LineCollection(
                transform_geometry([[(0, 0), (0, robot_axis_plot_size)]], pose),
                color="g",
            )
        )

    def dynamics_model(self, x, u):
        # dx/dt = f(x, u)
        return ca.vertcat(
            x[3],
            x[4],
            x[5],
            u[0],
            u[1],
            u[2],
        )

    def get_vector_to_module(self, module_idx, theta):
        module_x = self.MODULE_POSITIONS[module_idx][0]
        module_y = self.MODULE_POSITIONS[module_idx][1]
        rho = np.sqrt(module_x * module_x + module_y * module_y)
        phi = np.arctan2(module_y, module_x)
        return rho * np.cos(theta + phi), rho * np.sin(theta + phi)

    def apply_speed_constraint(self, opti, x, u, i, speed_limit):
        # Impose additional speed limit on top of motor speed limit
        theta = x[2, i]
        xDot = x[3, i]
        yDot = x[4, i]
        thetaDot = x[5, i]

        r0x, r0y = self.get_vector_to_module(0, theta)
        r1x, r1y = self.get_vector_to_module(1, theta)
        r2x, r2y = self.get_vector_to_module(2, theta)
        r3x, r3y = self.get_vector_to_module(3, theta)

        v0x = xDot - r0y * thetaDot
        v0y = yDot + r0x * thetaDot
        v1x = xDot - r1y * thetaDot
        v1y = yDot + r1x * thetaDot
        v2x = xDot - r2y * thetaDot
        v2y = yDot + r2x * thetaDot
        v3x = xDot - r3y * thetaDot
        v3y = yDot + r3x * thetaDot

        v0_sq = v0x * v0x + v0y * v0y
        v1_sq = v1x * v1x + v1y * v1y
        v2_sq = v2x * v2x + v2y * v2y
        v3_sq = v3x * v3x + v3y * v3y

        max_v_sq = speed_limit * speed_limit
        opti.subject_to(v0_sq <= max_v_sq)
        opti.subject_to(v1_sq <= max_v_sq)
        opti.subject_to(v2_sq <= max_v_sq)
        opti.subject_to(v3_sq <= max_v_sq)

    def apply_module_constraints(self, opti, x, u, N):
        theta = x[2, :]
        xDot = x[3, :]
        yDot = x[4, :]
        thetaDot = x[5, :]
        xDotDot = u[0, :]
        yDotDot = u[1, :]
        thetaDotDot = u[2, :]

        module_data = ModuleData(
            velocities=[[] for _ in range(self.NUM_MODULES)],
            headings=[[] for _ in range(self.NUM_MODULES)],
            forces=[[] for _ in range(self.NUM_MODULES)],
            forces_lon=[[] for _ in range(self.NUM_MODULES)],
            forces_lat=[[] for _ in range(self.NUM_MODULES)],
        )

        # For each state, compute module vectors and apply constraints
        for k in range(N + 1):
            # Compute module vectors and velocities
            module_vecs = [
                self.get_vector_to_module(i, theta[k]) for i in range(self.NUM_MODULES)
            ]
            module_velocities = []

            for i, (rx, ry) in enumerate(module_vecs):
                vx = xDot[k] - ry * thetaDot[k]
                vy = yDot[k] + rx * thetaDot[k]
                v_sq = vx * vx + vy * vy
                module_velocities.append((vx, vy, v_sq))
                module_data.velocities[i].append(np.sqrt(v_sq))

            # Store velocity headings in robot frame
            cos_theta = ca.cos(theta[k])
            sin_theta = ca.sin(theta[k])
            eps = 1e-6

            for i, (vx, vy, v_sq) in enumerate(module_velocities):
                # Convert field frame velocities to robot frame by rotating by -theta
                vx_robot = vx * cos_theta + vy * sin_theta
                vy_robot = -vx * sin_theta + vy * cos_theta

                # Only store heading if velocity magnitude is above threshold
                v_mag = ca.sqrt(v_sq)
                heading = ca.if_else(v_mag > eps, ca.atan2(vy_robot, vx_robot), np.nan)
                module_data.headings[i].append(heading)

            # Apply velocity constraints
            max_v = (
                (self.MOTOR_MAX_RPM / 60 / self.DRIVE_RATIO) * np.pi * self.WHEEL_DIA
            )
            max_v_sq = max_v * max_v
            for _, _, v_sq in module_velocities:
                opti.subject_to(v_sq <= max_v_sq)

            # Everything else depends on control, which only goes up to N-1
            if k >= N:
                continue

            # Define force variables (fx, fy) for each module
            module_forces = [
                (opti.variable(), opti.variable()) for _ in range(self.NUM_MODULES)
            ]
            for i, ((fx, fy), (vx, vy, v_sq)) in enumerate(
                zip(module_forces, module_velocities)
            ):
                # Force magnitude
                f_mag = ca.sqrt(fx * fx + fy * fy)
                module_data.forces[i].append(f_mag)

                # Compute longitudinal and lateral force components
                # f_lon = (f · v) / |v| (dot product - parallel component)
                # f_lat = (f × v) / |v| (cross product - perpendicular component, signed)
                v_mag = ca.sqrt(v_sq)
                f_dot_v = fx * vx + fy * vy
                f_cross_v = fx * vy - fy * vx
                f_lon = ca.if_else(v_mag > eps, f_dot_v / v_mag, np.nan)
                f_lat = ca.if_else(v_mag > eps, f_cross_v / v_mag, np.nan)
                module_data.forces_lon[i].append(f_lon)
                module_data.forces_lat[i].append(f_lat)

            # Forces must respect motor and friction limits
            max_friction_force = self.MU * self.MASS * G / self.NUM_MODULES
            force_limit = min(self.WHEEL_MAX_FORCE, max_friction_force)
            force_limit_sq = force_limit * force_limit

            for fx, fy in module_forces:
                opti.subject_to(fx * fx + fy * fy <= force_limit_sq)

            # Axis-aligned forces needed to achieve this linear acceleration.
            Fx = self.MASS * xDotDot[k]
            Fy = self.MASS * yDotDot[k]
            # Torque about the center of rotation needed to achieve this angular acceleration.
            T = self.J * thetaDotDot[k]

            # Sum of all linear force components must equal |Fx| and |Fy|
            opti.subject_to(Fx == sum(fx for fx, _ in module_forces))
            opti.subject_to(Fy == sum(fy for _, fy in module_forces))

            # Sum of all torques must equal |T|
            # Tangent direction is perpendicular to radius: m_dir = [-ry, rx]
            # Tangential force: ft = dot(f, m_dir) / norm(m_dir)
            # Since norm(m_dir) = norm(r) = rho, we can compute ft = (fx * (-ry) + fy * rx) / rho
            # Torque = ft * rho = fx * (-ry) + fy * rx
            total_torque = 0
            for (fx, fy), (rx, ry) in zip(module_forces, module_vecs):
                rho = ca.sqrt(rx * rx + ry * ry)
                ft = (fx * (-ry) + fy * rx) / rho
                total_torque += ft * rho
            opti.subject_to(T == total_torque)

        return module_data

    def apply_obstacle_constraints(self, opti, xpos, ypos, theta, obstacles):
        for obx, oby, obr in obstacles:
            for p1, p2 in self.GEOMETRY:
                # Transform robot geometry to pose
                x1, y1 = rotate_around_origin(p1, theta)
                x2, y2 = rotate_around_origin(p2, theta)
                x1 += xpos
                y1 += ypos
                x2 += xpos
                y2 += ypos

                # Compute the closest distance between a point and a line segment
                px = x2 - x1
                py = y2 - y1
                norm = px * px + py * py
                u = ((obx - x1) * px + (oby - y1) * py) / norm
                u = ca.fmax(ca.fmin(u, 1), 0)
                x = x1 + u * px
                y = y1 + u * py
                dx = x - obx
                dy = y - oby

                dist = ca.sqrt((dx * dx + dy * dy))
                opti.subject_to(dist > obr + self.OBSTACLE_BUFFER)
