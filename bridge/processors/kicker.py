"""
Генерация объектов типа wp.Waypoint для роботов, движущихся с мячом
"""
from typing import Optional

import bridge.processors.auxiliary as aux
import bridge.processors.waypoint as wp
import bridge.processors.field as fld
from bridge.processors import const

class KickerAux:
    def __init__(self) -> None:
        self.twist_w: float = 0.5
        self.wait_kick_timer: Optional[float] = None

    def reset_kick_consts(self) -> None:
        "Обнулить константы для ударов"
        self.twist_w = 0.5
        self.wait_kick_timer = None

    def kick_to_point(self, field: fld.Field, kicker_id: int, kick_point: aux.Point) -> wp.Waypoint:
        "Удар в точку с выбором типа удара"
        if kicker_id < 9:
            angle  = aux.angle_to_point(field.ball.get_pos(), kick_point)
            self.reset_kick_consts()
            return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_KICK)
        
        if not field.is_ball_in(field.allies[kicker_id]):
            angle  =aux.angle_to_point(field.allies[kicker_id].get_pos(), field.ball.get_pos())
            self.reset_kick_consts()
            return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_GRAB)
        else:
            return self.twisted(field, kicker_id, kick_point)

    def twisted(
            self, field: fld.Field, kicker_id: int, kick_point: aux.Point
        ) -> wp.Waypoint:
        """
        Прицеливание и удар в точку при условии, что мяч находится в захвате у робота
        """

        kicker = field.allies[kicker_id]

        # signed_A = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - self.start_rotating_ang)
        # A = abs(signed_A)
        signed_x = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle())
        x = abs(signed_x)

        beta = 4
        a = beta / x - self.twist_w / (x**2)
        b = 2 * x * a - beta
        w = self.twist_w + const.Ts ** 2 * a + b * const.Ts
        self.twist_w = w
        if signed_x < 0:
            w *= -1
        # print(abs(w))
        waypoint = spin_with_ball(w)

        if x > const.KICK_ALIGN_ANGLE:
            field.allies[kicker_id].set_dribbler_speed(max(5, 15 - abs(w) * 5))
            self.wait_kick_timer = None
        else:
            # if self.wait_kick_timer is None:
            #     self.wait_kick_timer = time()
            # else:
            #     wt = 0.1
            # if time() - self.wait_kick_timer > wt:
            self.wait_kick_timer = None
            field.allies[kicker_id].kick_forward()
            # else:
            #     print(field.allies[kicker_id].dribbler_speed_)
            #     field.allies[kicker_id].set_dribbler_speed(max(6, 15 - (15 / wt) * (time() - self.wait_kick_timer)))
        return waypoint

def spin_with_ball(w: float) -> wp.Waypoint:
    """
    Расчёт скорости робота для поворота с мячом с угловой скоростью w (рад/сек)
    """
    if 0.01 < abs(w) < 0.3:
        w = 0.3 * aux.sign(w)

    if w > 0:
        delta_r = aux.Point(-160, 0)
    else:
        delta_r = aux.Point(160, 0)
    vel = delta_r * w

    k_w = 1.65  # костыль для приближения значений угловой скорости к рад/с
    k_vel = 0.4 * k_w
    # print("target radius: ", delta_r.mag())
    return wp.Waypoint(vel * k_vel, w * k_w, wp.WType.S_VELOCITY)
