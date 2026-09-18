import numpy as np

from utils.field import Field

# Start positions
FIELD_CENTER_START_POSE = (7.2, Field.WIDTH * 0.5, np.pi)
DS_LEFT_START_POSE = (7.2, 5.4, np.pi)
DS_LEFT_WALL_START_POSE = (7.0, 7.5, np.pi)

# TODO change climb pose placeholder
CLIMB_POSE = (1.7, 4, np.pi)

SCORING_POSE = (2.4, 2.5, np.pi)
SECOND_SWEEP_90_POSE = (2.4, 2.5, 1.5 * np.pi)

FEED_AUTO_TRANSITION_POSE = (7.5, 2.5, 1.5 * np.pi)
CENTER_PAUSE_POSE = (8.4, 4, 0)
CENTER_PAUSE_SLOW_POSE = (6, 4.0, 0)
CENTER_PAUSE_SLOW_XXXX_POSE = (7, 2.5, 0)

INTAKE_ROBOT_SPEED = 1.5

FAST_RACE_END_POSE = (8, 3, 0.25 * np.pi)
GO_AFTER_PARTNER_END_POSE = (8.5, 6.5, 0.5 * np.pi)
