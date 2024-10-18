"""
Определение необходимых констант
"""

from enum import Enum


class Color(Enum):
    """Класс с цветами"""

    ALL = 0
    BLUE = 1
    YELLOW = 2


class Div(Enum):
    """Класс с разными дивизионами"""

    A = 0  # XD
    B = 1
    C = 2


##################################################
# GAME SETTING CONSTS
DIV = Div.B
COLOR = Color.YELLOW
POLARITY = 1  # -1 если ворота синих на +x; 1 если ворота синих на -x


IS_SIMULATOR_USED = True
IS_DRIBBLER_USED = True  # dribbler and upper_kick
SELF_PLAY = False

GK = 0
ENEMY_GK = 0

ROBOTS_MAX_COUNT: int = 32
TEAM_ROBOTS_MAX_COUNT: int = ROBOTS_MAX_COUNT // 2
GEOMETRY_PACKET_SIZE: int = 2

CONTROL_MAPPING: dict[int, int] = {
    # vision_id: control_id,
    # 0: 8,
    # 1: 9,
    # 2: 10,
    # 3: 11,
    # 4: 12,
    # 5: 13,
    # 6: 14,
    # 7: 15,
    # 8: 0,
    # 9: 1,
    # 10: 2,
    # 11: 3,
    # 12: 4,
    # 13: 5,
    # 14: 6,
    # 15: 7,
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
REVERSED_KICK: list[int] = []

for i in range(TEAM_ROBOTS_MAX_COUNT):
    try:
        CONTROL_MAPPING[i]
    except KeyError:
        CONTROL_MAPPING[i] = -1

CONTROL_TOPIC = "control-topic"
FIELD_TOPIC = "field-topic"
GAMESTATE_TOPIC = "gamestate-topic"
IMAGE_TOPIC = "image-topic"
PASSES_TOPIC = "passes-topic"
##################################################

##################################################
# CONTROL CONSTS
Ts = 0.02  # s

# ROBOT SETTING CONSTS
MAX_SPEED = 1250
MAX_SPEED_R = 30
SOFT_MAX_SPEED = 500
SOFT_MAX_SPEED_R = 16

INTERCEPT_SPEED = 50
##################################################
# GEOMETRY CONSTS

BALL_R = 22
ROBOT_R = 100
GRAVEYARD_POS_X = -10000

BALL_MAX_SPEED = 10000  # for filter random balls

match DIV:
    case Div.A:
        GOAL_DX = 1 / 0  # не дорос ещё

    case Div.B:
        GOAL_DX = 4000
        GOAL_DY = 1000
        GOAL_PEN_DX = 1000
        GOAL_PEN_DY = 2000

        FIELD_DX = GOAL_DX
        FIELD_DY = 3000

        GK_FORW = 200 + ROBOT_R

    case Div.C:
        GOAL_DX = 2250
        GOAL_DY = 800
        GOAL_PEN_DX = 500
        GOAL_PEN_DY = 1350

        FIELD_DX = GOAL_DX
        FIELD_DY = 1500

        GK_FORW = 100 + ROBOT_R


# ROUTE CONSTS
KEEP_BALL_DIST = 500 + ROBOT_R
VIEW_DIST = 1500

# is_ball_in
GRAB_ALIGN_DIST = 130
BALL_GRABBED_DIST = 115
BALL_GRABBED_ANGLE = 0.8

# is_kick_aligned
KICK_ALIGN_DIST_MULT = 1.5
KICK_ALIGN_ANGLE = 0.07
KICK_ALIGN_DIST = 150
KICK_ALIGN_OFFSET = 20

# for grabbing ball
GRAB_AREA = 250
GRAB_DIST = 50
GRAB_MULT = 2  # speed = dist * mult
GRAB_OFFSET = 100


# VOLTAGES
VOLTAGE_PASS = 15
VOLTAGE_SHOOT = 15
VOLTAGE_UP = 15
VOLTAGE_ZERO = min(VOLTAGE_PASS, VOLTAGE_SHOOT, VOLTAGE_UP)
