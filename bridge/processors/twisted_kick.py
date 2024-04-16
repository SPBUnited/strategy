"""
Генерация объектов типа wp.Waypoint для роботов, движущихся с мячом
"""
import bridge.processors.auxiliary as aux
import bridge.processors.waypoint as wp


def spin_with_ball(w: float) -> wp.Waypoint:
    """
    Расчёт скорости робота для поворота с мячом с угловой скоростью w (рад/сек)
    """
    # delta_r = aux.rotate(aux.Point(200, 0), math.pi * 1.75)
    if w > 0:
        delta_r = aux.Point(-160, 0)
    else:
        delta_r = aux.Point(160, 0)
    vel = delta_r * w

    k_w = 1.65
    k_vel = 0.4 * k_w
    # print("target radius: ", delta_r.mag())
    return wp.Waypoint(vel * k_vel, w * k_w, wp.WType.S_VELOCITY)
