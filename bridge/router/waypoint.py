"""
Описание структуры путевой точки

Хранит:
- Координата путевой точки
- Желаемый угол поворота робота в точке
- Тип путевой точки
"""

from enum import Enum

from bridge.auxiliary import aux


class WType(Enum):
    """
    Типы путевых точек
    """

    T_ROBOT = -1  # хз, нужен в Route
    S_ENDPOINT = 0  # Двигаться в точку и остановиться (конечная точка пути)
    S_IGNOREOBSTACLES = 1  # Двигаться в точку игнорируя препятствия
    S_STOP = 2  # Literally top

    # waypoint.pos - скорость; waypoint.angle - угловая скорость
    S_VELOCITY = 3  # Разворачиваться с мячом

    S_BALL_GO = 5  # Двигаться с мячом
    S_BALL_KICK = 6  # Захватить мяч и мгновенно его пнуть
    S_BALL_KICK_UP = 7  # Захватить мяч и мгновенно его пнуть верхом
    S_BALL_GRAB = 8  # Захватить мяч не пиная
    S_BALL_PASS = 9  # Give pass to point

    # waypoint.pos - точка, в которую необходимо пнуть, waypoint.angle - угол из робота на мяч
    S_BALL_TWIST = 10  # Вращаться с мячом в захвате
    S_BALL_TWIST_PASS = 11  # Вращаться с мячом в захвате (ниже сила удара)

    R_PASSTHROUGH = 20  # Двигаться в точку не останавливаясь (промежуточная точка пути)
    R_BALL_ALIGN = 21  # Выровняться напротив мяча, приготовившись его захватить

    T_GRAVEYARD = 100


BALL_WP_TYPES = [
    WType.S_BALL_GO,
    WType.S_BALL_KICK,
    WType.S_BALL_GRAB,
    WType.S_BALL_KICK_UP,
    WType.S_BALL_PASS,
    WType.S_BALL_TWIST,
    WType.S_BALL_TWIST_PASS,
]


class Waypoint:
    """
    Описание путевой точки
    """

    def __init__(self, pos: aux.Point, angle: float, wp_type: WType) -> None:
        """
        Конструктор
        """
        self.pos = pos
        self.angle = angle
        self.type = wp_type

    def __str__(self) -> str:
        return f"WP:  {str(self.pos)}; angle = {self.angle:0.2f}; type =  {str(self.type)}"
