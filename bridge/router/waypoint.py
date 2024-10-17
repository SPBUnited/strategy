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
    S_STOP = 2  # Kostil for stopping
    S_VELOCITY = 3  # Разворачиваться с мячом
    # waypoint.pos - скорость; waypoint.angle - угловая скорость

    S_BALL_GO = 5  # Двигаться с мячом
    S_BALL_KICK = 6  # Захватить мяч и мгновенно его пнуть
    S_BALL_KICK_UP = 7  # Захватить мяч и мгновенно его пнуть верхом
    S_BALL_GRAB = 8  # Захватить мяч не пиная
    S_BALL_PASS = 9  # Give pass to point
    S_BALL_TWIST = 10  # Вращаться с мячом в захвате

    R_PASSTHROUGH = 20  # Двигаться в точку не останавливаясь (промежуточная точка пути)
    R_BALL_ALIGN = 21  # Выровняться напротив мяча, приготовившись его захватить

    T_GRAVEYARD = 100


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
