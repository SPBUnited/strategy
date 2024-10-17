"""
Класс, описывающий текущий маршрут робота
"""

import math
from time import time
from typing import Optional

import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, fld, rbt, tau


class Route:
    """
    Класс описание произвольного маршрута
    """

    def __init__(self, robot: rbt.Robot):
        """
        Конструктор
        """
        self._robot = [wp.Waypoint(robot.get_pos(), robot._angle, wp.WType.T_ROBOT)]
        self._destination = wp.Waypoint(aux.GRAVEYARD_POS, 0, wp.WType.T_GRAVEYARD)
        self._routewp: list[wp.Waypoint] = []

        self.ball_wp_types = [
            wp.WType.S_BALL_GO,
            wp.WType.S_BALL_KICK,
            wp.WType.S_BALL_GRAB,
            wp.WType.S_BALL_KICK_UP,
            wp.WType.S_BALL_PASS,
            wp.WType.S_BALL_TWIST,
        ]
        self.last_update = time()

    def update(self, robot: rbt.Robot) -> None:
        """
        Обновить маршрут

        Обновляет текущее положение робота в маршрутной карте
        """
        self._robot = [wp.Waypoint(robot.get_pos(), robot.get_angle(), wp.WType.T_ROBOT)]

    def clear(self) -> None:
        """
        Очистить промежуточные точки маршрута
        """
        self._routewp = []

    def __get_route(self) -> list[wp.Waypoint]:
        """
        Получить маршрут в виде списка путевых точек
        """
        return [*self._robot, *self._routewp, self._destination]

    def set_dest_wp(self, dest: wp.Waypoint) -> None:
        """
        Задать конечную точку
        """
        self.clear()
        self._destination = dest

    def get_dest_wp(self) -> wp.Waypoint:
        """
        Получить конечную точку
        """
        return self._destination

    def get_next_wp(self) -> wp.Waypoint:
        """
        Получить следующую путевую точку
        """
        return self.__get_route()[1]

    def get_next_segment(self) -> list[wp.Waypoint]:
        """
        Получить следующий сегмент маршрута в виде списка двух точек
        """
        return self.__get_route()[0:1]

    def get_next_vec(self) -> aux.Point:
        """
        Получить следующий сегмент маршрута в виде вектора
        """
        return self.__get_route()[1].pos - self.__get_route()[0].pos

    def get_next_angle(self) -> float:
        """
        Получить угол следующей путевой точки
        """
        return self.__get_route()[1].angle

    def get_next_type(self) -> wp.WType:
        """
        Получить тип следующей путевой точки
        """
        return self.__get_route()[1].type

    def insert_wp(self, wpt: wp.Waypoint) -> None:
        """
        Вставить промежуточную путевую точку в начало маршрута
        """
        self._routewp.insert(0, wpt)

    def is_used(self) -> bool:
        """
        Определить, используется ли маршрут
        """
        return self._destination.type != wp.WType.T_GRAVEYARD

    def get_length(self) -> float:
        """
        Получить длину маршрута
        """
        dist = 0.0
        last_wp_pos = self._robot[0].pos
        for wpt in self.__get_route():
            if wpt.type in self.ball_wp_types:
                break
            dist += (wpt.pos - last_wp_pos).mag()
            last_wp_pos = wpt.pos
        return dist

    def __str__(self) -> str:
        string = "ROUTE: "
        for wpt in self.__get_route():
            string += " ->\n" + str(wpt)
        return string

    def kicker_control(self, robot: rbt.Robot) -> None:
        """control voltage and autokick"""
        end_point = self.get_dest_wp()

        if end_point.type in self.ball_wp_types and self.get_length() < 300:
            robot.dribbler_enable_ = 1
            if robot.dribbler_speed_ == 0:
                robot.dribbler_speed_ = 15

            robot.kicker_charge_enable_ = 1
            if robot.kicker_voltage_ == 0:
                robot.kicker_voltage_ = const.VOLTAGE_SHOOT
                if end_point.type in [wp.WType.S_BALL_GRAB, wp.WType.S_BALL_GO]:
                    robot.kicker_voltage_ = const.VOLTAGE_ZERO
                elif end_point.type == wp.WType.S_BALL_PASS:
                    robot.kicker_voltage_ = const.VOLTAGE_PASS
                elif end_point.type == wp.WType.S_BALL_KICK_UP:
                    robot.kicker_voltage_ = const.VOLTAGE_UP

            is_aligned_by_angle = robot.is_kick_aligned_by_angle(end_point.angle)
            if end_point.type in [wp.WType.S_BALL_KICK, wp.WType.S_BALL_PASS] and is_aligned_by_angle:
                robot.auto_kick_ = 1
            elif end_point.type == wp.WType.S_BALL_KICK_UP and is_aligned_by_angle:
                robot.auto_kick_ = 2
            else:
                robot.auto_kick_ = 0

    def vel_control(self, robot: rbt.Robot, field: fld.Field) -> tuple[aux.Point, float]:
        """set vel using waypoint"""
        target_point = self.get_next_wp()
        end_point = self.get_dest_wp()

        cur_vel = robot.get_vel()
        vec_err = target_point.pos - robot.get_pos()
        dest_vec = end_point.pos - robot.get_pos()

        transl_vel: Optional[aux.Point] = None

        if end_point.type == wp.WType.S_STOP:
            transl_vel = aux.Point(0, 0)
            angle = robot.get_angle()
        elif target_point.type == wp.WType.R_PASSTHROUGH:  # TODO fix bad vel control
            robot.pos_reg_x.select_mode(tau.Mode.NORMAL)
            robot.pos_reg_y.select_mode(tau.Mode.NORMAL)

            u_x = robot.pos_reg_x.process(dest_vec.x, -cur_vel.x)
            u_y = robot.pos_reg_y.process(dest_vec.y, -cur_vel.y)

            transl_vel = aux.rotate(aux.Point(u_x, u_y), vec_err.arg() - dest_vec.arg())
            if transl_vel.mag() > const.MAX_SPEED:
                transl_vel = transl_vel.unity() * const.MAX_SPEED

            angle = end_point.angle
        elif end_point.type in self.ball_wp_types and robot.is_kick_aligned(end_point):
            transl_vel = aux.rotate(aux.RIGHT, target_point.angle) * 300
            angle = target_point.angle
        else:
            if end_point.type in self.ball_wp_types and self.get_length() < 500:
                robot.pos_reg_x.select_mode(tau.Mode.SOFT)
                robot.pos_reg_y.select_mode(tau.Mode.SOFT)
                transl_vel = field.ball.get_vel()
            else:
                robot.pos_reg_x.select_mode(tau.Mode.NORMAL)
                robot.pos_reg_y.select_mode(tau.Mode.NORMAL)
                transl_vel = aux.Point(0, 0)

            u_x = robot.pos_reg_x.process(vec_err.x, -cur_vel.x)
            u_y = robot.pos_reg_y.process(vec_err.y, -cur_vel.y)

            transl_vel += aux.Point(u_x, u_y)
            if transl_vel.mag() > const.MAX_SPEED:
                transl_vel = transl_vel.unity() * const.MAX_SPEED

            angle = end_point.angle

        return (transl_vel, angle)

    def go_route(self, robot: rbt.Robot, field: fld.Field) -> None:
        """
        Двигаться по маршруту route
        """
        dT = time() - self.last_update
        self.last_update = time()
        if self.get_next_type() == wp.WType.S_VELOCITY:
            waypoint = self.get_dest_wp()
            robot.kicker_charge_enable_ = 1
            robot.speed_x = -waypoint.pos.x / robot.k_xx
            robot.speed_y = waypoint.pos.y / robot.k_yy
            robot.delta_angle = (
                math.log(18 / math.pi * abs(waypoint.angle) + 1) * aux.sign(waypoint.angle) * (100 / math.log(18 + 1))
            )
            robot.beep = 1
            return

        robot.beep = 0

        self.kicker_control(robot)

        vel, angle = self.vel_control(robot, field)  # in global coordinate system
        robot.update_vel_xy_(vel, dT)

        aerr = aux.wind_down_angle(angle - robot.get_angle())
        if const.IS_SIMULATOR_USED:
            ang_vel = robot.angle_reg.process_(aerr, -robot.get_anglevel(), dT)
            robot.update_vel_w(ang_vel)
        else:
            robot.delta_angle = math.log(18 / math.pi * abs(aerr) + 1) * aux.sign(aerr) * (100 / math.log(18 + 1))

        reg_vel = aux.Point(robot.speed_x, -robot.speed_y)
        field.router_image.draw_line(
            robot.get_pos(),
            robot.get_pos() + aux.rotate(reg_vel, robot.get_angle()) * 10,
        )
