"""
Генерация объектов типа wp.Waypoint для роботов, движущихся с мячом
"""

from typing import Optional

import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, fld, rbt


class KickerAux:
    def __init__(self) -> None:
        self.twist_w: float = 0.5
        self.wait_kick_timer: Optional[float] = None

        self.robot_voltage: list[int] = [15 for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]

    def reset_kick_consts(self) -> None:
        "Обнулить константы для ударов"
        self.twist_w = 0.5
        self.wait_kick_timer = None

    def pass_to_point(self, field: fld.Field, kicker: rbt.Robot, receive_pos: aux.Point) -> wp.Waypoint:
        "Пасс"
        if receive_pos.x * kicker.get_pos().x > 0:
            self.set_voltage(field, kicker.r_id, receive_pos, "PASS")
        else:
            self.set_voltage(field, kicker.r_id, receive_pos, "SHOOT")

        # if kicker.r_id < 9:  ####NOTE
        #     angle = aux.angle_to_point(field.ball.get_pos(), receive_pos)
        #     self.reset_kick_consts()
        #     return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_KICK)

        nearest_enemy = fld.find_nearest_robot(field.ball.get_pos(), field.enemies)
        if aux.dist(field.ball.get_pos(), nearest_enemy.get_pos()) < 500 or field.is_ball_in(kicker):
            return self.twisted(field, kicker, receive_pos)
        else:
            angle = aux.angle_to_point(field.ball.get_pos(), receive_pos)
            return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_KICK)

    def shoot_to_goal(self, field: fld.Field, kicker: rbt.Robot, shoot_point: aux.Point) -> wp.Waypoint:
        "Удар по воротам"
        self.set_voltage(field, kicker.r_id, shoot_point, "SHOOT")

        # if kicker.r_id < 9:  ####NOTE
        #     angle = aux.angle_to_point(field.ball.get_pos(), shoot_point)
        #     self.reset_kick_consts()
        #     return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_KICK)

        fld.find_nearest_robot(field.ball.get_pos(), field.enemies)
        # if aux.dist(field.ball.get_pos(), nearest_enemy.get_pos()) < 500:
        #     return self.twisted(field, kicker, shoot_point)
        if field.is_ball_in(kicker):
            if aux.dist(field.ball.get_pos(), field.enemy_goal.center) < 1000:
                return self.twisted(field, kicker, shoot_point, "ANGRY")
            return self.twisted(field, kicker, shoot_point)
        else:
            if const.IS_DRIBBLER_USED:
                target = aux.Point(shoot_point.x, 0)
            else:
                target = shoot_point
            angle = aux.angle_to_point(field.ball.get_pos(), target)
            return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_GRAB)

    def set_voltage(self, field: fld.Field, kicker_id: int, kick_point: aux.Point, style: str) -> None:
        "Set voltage for robot's kicker"
        angle_to_target = aux.angle_to_point(field.allies[kicker_id].get_pos(), kick_point)
        if (
            aux.dist(field.allies[kicker_id].get_pos(), field.ball.get_pos()) > 200
            or abs(aux.wind_down_angle(field.allies[kicker_id].get_angle() - angle_to_target)) > 1
        ):
            # print(aux.dist(field.allies[kicker_id].get_pos(), field.ball.get_pos()), aux.wind_down_angle(field.allies[kicker_id].get_angle() - angle_to_target))
            field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_ZERO
        elif style == "PASS":
            field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_PASS
        elif style == "SHOOT":
            field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_SHOOT
        elif style == "UP":
            field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_UP

        # print("voltage", self.robot_voltage[kicker_id], "->", field.allies[kicker_id].kicker_voltage_)
        if field.allies[kicker_id].kicker_voltage_ < self.robot_voltage[kicker_id] and not field.is_ball_in(
            field.allies[kicker_id]
        ):
            field.allies[kicker_id].kick_forward()
        self.robot_voltage[kicker_id] = field.allies[kicker_id].kicker_voltage_

    def twisted(
        self,
        field: fld.Field,
        kicker: rbt.Robot,
        kick_point: aux.Point,
        style: str = "SAFE",
    ) -> wp.Waypoint:
        """
        Прицеливание и удар в точку
        """
        if const.IS_SIMULATOR_USED:
            angle = aux.angle_to_point(field.ball.get_pos(), kick_point)
            return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_KICK)

        if not field.is_ball_in(kicker):
            angle = aux.angle_to_point(kicker.get_pos(), field.ball.get_pos())
            if kicker.is_kick_aligned_by_angle(angle):
                return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_KICK)
            self.reset_kick_consts()
            return wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_BALL_GRAB)

        signed_x = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle())
        x = abs(signed_x)

        if not const.IS_DRIBBLER_USED:
            waypoint = spin_around_ball(1 * aux.sign(signed_x))
        elif style != "SAFE":
            waypoint = spin_with_ball(1 * aux.sign(signed_x))
            kicker.set_dribbler_speed(12)
        else:

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
            self.wait_kick_timer = None
        else:
            # if self.wait_kick_timer is None:
            #     self.wait_kick_timer = time()
            # wt = 0
            # if time() - self.wait_kick_timer > wt:
            self.wait_kick_timer = None
            kicker.kick_forward()
            self.reset_kick_consts()

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


def spin_around_ball(w: float) -> wp.Waypoint:
    """
    Расчёт скорости робота для поворота с мячом с угловой скоростью w (рад/сек)
    """
    if 0.01 < abs(w) < 0.3:
        w = 0.3 * aux.sign(w)

    delta_r = aux.Point(0, 140)

    vel = delta_r * w

    k_w = 1.65  # костыль для приближения значений угловой скорости к рад/с
    k_vel = 0.4 * k_w
    return wp.Waypoint(vel * k_vel, w * k_w, wp.WType.S_VELOCITY)
