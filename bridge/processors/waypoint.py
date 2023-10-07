"""
Описание структуры путевой точки

Хранит:
- Координата путевой точки
- Желаемый угол поворота робота в точке
- Тип путевой точки
"""

import bridge.processors.auxiliary as aux
from enum import Enum

class WType(Enum):
    ROBOT = -1 # хз, нужен в Route
    ENDPOINT = 0 # Двигаться в точку и остановиться (конечная точка пути)
    PASSTHROUGH = 1 # Двигаться в точку не останавливаясь (промежуточная точка пути)
    IGNOREOBSTACLES = 2 # Двигаться в точку игнорируя препятствия
    BALL_KICK = 3 # Захватить мяч и мгновенно его пнуть
    BALL_GRAB = 4 # Захватить мяч не пиная
    BALL_ALIGN = 5 # Выровнятся напротив мяча, приготовившись его захватить
    STOP = 6 # Kostil for stopping 
    GRAVEYARD = 100

class Waypoint:
    def __init__(self, pos: aux.Point, angle: float, type: WType) -> None:
        self.pos = pos
        self.angle = angle
        self.type = type

    def __str__(self):
        return "WP: " + str(self.pos) + "; angle = %.2f"%self.angle + "; type = " + str(self.type)
