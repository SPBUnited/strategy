"""
Генерация объектов типа wp.Waypoint для роботов, движущихся с мячом
"""
from typing import Optional

import bridge.processors.auxiliary as aux
import bridge.processors.waypoint as wp
import bridge.processors.field as fld
from bridge.processors import const
from time import time

class KickerAux:
    def __init__(self) -> None:
        self.twist_w: float = 0.5
        self.wait_kick_timer: Optional[float] = None

    def reset_kick_consts(self) -> None:
        "Обнулить константы для ударов"
        self.twist_w = 0.5
        self.wait_kick_timer = None

    def kick_to_point(self, field: fld.Field, kicker_id: int, kick_point: aux.Point, type: str = "SHOOT") -> wp.Waypoint:
        "Удар в точку с выбором типа удара (PASS/SHOOT)"
        if kicker_id < 90:
            angle  = aux.angle_to_point(field.ball.get_pos(), kick_point)
            self.reset_kick_consts()
            return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_KICK)

        angle_to_target = aux.angle_to_point(field.allies[kicker_id].get_pos(), kick_point)
        # if not (field.allies[kicker_id]) or abs(field.allies[kicker_id].get_angle() - angle_to_target) > 1:
        if aux.dist(field.allies[kicker_id].get_pos(), field.ball.get_pos()) < 500 or abs(field.allies[kicker_id].get_angle() - angle_to_target) > 1:
            field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_ZERO
        elif type == "PASS":
            field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_PASS
        elif type == "SHOOT":
            field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_SHOOT
        field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_SHOOT

        nearest_enemy = fld.find_nearest_robot(field.ball.get_pos(), field.enemies)
        if aux.dist(field.ball.get_pos(), nearest_enemy.get_pos()) < 500 and not field.is_ball_in(field.allies[kicker_id]):
            return self.twisted(field, kicker_id, kick_point)
        else:
            return self.twisted(field, kicker_id, kick_point)

    def safe(
            self, field: fld.Field, kicker_id: int, kick_point: aux.Point
    ) -> wp.Waypoint:
        kicker_pos = field.allies[kicker_id].get_pos()
        # angle  =aux.angle_to_point(kicker_pos, field.ball.get_pos())
        # if not field.is_ball_in(field.allies[kicker_id]):
        #     self.reset_kick_consts()
        #     target = field.ball.get_pos() + (field.ball.get_pos() - kick_point).unity() * const.ROBOT_R
        #     dist = aux.dist(field.ball.get_pos(), kicker_pos) 
        #     if dist > const.ROBOT_R + const.BALL_R:
        #         tangents = aux.get_tangent_points(field.ball.get_pos(), kicker_pos, const.ROBOT_R + const.BALL_R)
        #         passthrough = aux.find_nearest_point(target, tangents) 
        #         return wp.Waypoint(passthrough, angle, wp.WType.R_PASSTHROUGH)
        #     else:
        #         direction = aux.rotate(target - kicker_pos, field.allies[kicker_id].get_angle())
        #         w = 1
        #         if direction.x > 0:
        #             w *= -1
        #         return wp.Waypoint(aux.Point(0, dist), w, wp.WType.S_VELOCITY)
        angle = aux.angle_to_point(field.ball.get_pos(), kick_point)
        return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_KICK)
        

    def twisted(
            self, field: fld.Field, kicker_id: int, kick_point: aux.Point
        ) -> wp.Waypoint:
        """
        Прицеливание и удар в точку при условии, что мяч находится в захвате у робота
        """
        if not field.is_ball_in(field.allies[kicker_id]):
            angle  =aux.angle_to_point(field.allies[kicker_id].get_pos(), field.ball.get_pos())
            self.reset_kick_consts()
            return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_GRAB)

        kicker = field.allies[kicker_id]

        # signed_A = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - self.start_rotating_ang)
        # A = abs(signed_A)
        signed_x = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle())
        x = abs(signed_x)

        beta = 4
        a = beta / x - abs(self.twist_w) / (x**2)
        b = 2 * x * a - beta
        w = abs(self.twist_w) + b * const.Ts
        if signed_x < 0:
            w *= -1

        if x < 3.14 / 8:
            w = signed_x


        self.twist_w = w
        waypoint = spin_with_ball(w)

        if x > const.KICK_ALIGN_ANGLE:
            field.allies[kicker_id].set_dribbler_speed(max(5, 15 - abs(w) * 5))
            self.wait_kick_timer = None
        else:
            # if self.wait_kick_timer is None:
            #     self.wait_kick_timer = time()
            # wt = 0
            # if time() - self.wait_kick_timer > wt:
            self.wait_kick_timer = None
            field.allies[kicker_id].kick_forward()

            # else:
            #     field.allies[kicker_id].set_dribbler_speed(max(5, 15 - (15 / wt) * (time() - self.wait_kick_timer)))
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
    return wp.Waypoint(vel * k_vel, w * k_w, wp.WType.S_VELOCITY)
