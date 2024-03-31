"""
Определение необходимых констант
"""

##################################################
# GAME SETTING CONSTS
COLOR = 'y'
GK = 10
PENALTY_KICKER = 1
ENEMY_GK = 5
IS_SIMULATOR_USED = False
CAMERAS_COUNT: int = 4
MAX_BALLS_IN_CAMERA: int = 64
MAX_BALLS_IN_FIELD: int = CAMERAS_COUNT * MAX_BALLS_IN_CAMERA
BALL_PACKET_SIZE: int = 3

KEEP_BALL_DIST = 800

ROBOTS_MAX_COUNT: int = 32
TEAM_ROBOTS_MAX_COUNT: int = ROBOTS_MAX_COUNT // 2
SINGLE_ROBOT_PACKET_SIZE = 5
ROBOT_TEAM_PACKET_SIZE: int = SINGLE_ROBOT_PACKET_SIZE * TEAM_ROBOTS_MAX_COUNT

GEOMETRY_PACKET_SIZE: int = 2

DEBUG_ID = 14
DEBUG_CTRL = 14
CONTROL_MAPPING: dict[int, int] = {
    # DEBUG_ID: DEBUG_CTRL
    0: 0,
    1: 1,
    2: 2,
    3: 3,
    4: 4,
    5: 5,
    6: 6,
    7: 7,
    8: 8,
    9: 9,
    10: 10,
    11: 11,
    12: 12,
    13: 13,
    14: 14,
    15: 15,
}

for i in range(TEAM_ROBOTS_MAX_COUNT):
    try:
        CONTROL_MAPPING[i]
    except KeyError:
        CONTROL_MAPPING[i] = -1

TOPIC_SINK = "control-sink"
##################################################

##################################################
# CONTROL CONSTS
Ts = 0.05  # s

# ROBOT SETTING CONSTS
# MAX_SPEED = 100
# MAX_SPEED_R = 50
# ACCELERATION = 3
# BASE_KICKER_VOLTAGE = 7.0
MAX_SPEED = 1000
MAX_SPEED_R = 30
SOFT_MAX_SPEED = 500
SOFT_MAX_SPEED_R = 16
ACCELERATION = 3
BASE_KICKER_VOLTAGE = 7.0

R_KP = 7
R_KD = 0
KP = 0.1

GK_INTERCEPT_SPEED = 300
GK_PEN_KICKOUT_SPEED = 500
##################################################
# GEOMETRY CONSTS

BALL_R = 0.05
ROBOT_R = 0.2
GRAVEYARD_POS_X = -10000

POLARITY = 1  # 1 если ворота синих на +x; -1 если ворота синих на -x
GOAL_DX = 2250
GOAL_DY = 600
GOAL_PEN = 500 # * POLARITY
GOAL_BOUND_OFFSET = 100
GOAL_WALLLINE_OFFSET = 1000
GOAL_WALL_ROBOT_SEPARATION = 100  #расстояние между роботами в стенке

GK_FORW = 300
KICK_ALIGN_DIST = 200
KICK_ALIGN_DIST_MULT = 1.5
KICK_ALIGN_ANGLE = 0.1
KICK_ALIGN_OFFSET = 40
BALL_GRABBED_DIST = 110
BALL_GRABBED_ANGLE = 0.8

# ROUTE CONSTS

VANISH_DIST = 200
