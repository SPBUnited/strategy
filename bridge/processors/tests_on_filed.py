"""
Модуль с тестами для отладки на полигоне
"""
from time import time

import bridge.processors.field as fld
import bridge.processors.waypoint as wp


def test_ping(field: fld.Field, waypoints: list[wp.Waypoint], id: int) -> None:
    """
    Измеряет и выводит в терминал задержку зрения + управления
    Измерят с точностью до одной итерации (результат больше настоящей задержки, но не более чем на const.Ts)
    """

    if timer not in globals():
        global timer, timer1
        timer = time()
    else:
        global timer, timer1

    if time() - timer > 5 and timer1 is None:
        field.allies[id].kick_forward()
        timer1 = time()
    elif timer1 is not None and field.is_ball_moves():
        print(time() - timer1)
    print(field.is_ball_moves())
    field.allies[id].set_dribbler_speed(15)
