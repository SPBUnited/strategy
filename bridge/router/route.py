"""
Класс, описывающий текущий маршрут робота
"""

import math
from time import time
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt, tau
from bridge.router import kicker
from bridge.router import waypoint as wp


class Route:
    """
    Класс описание произвольного маршрута
    """

    def __init__(self, robot: rbt.Robot):
        """Конструктор"""
        self._robot = [wp.Waypoint(robot.get_pos(), robot._angle, wp.WType.T_ROBOT)]
        self._destination = wp.Waypoint(aux.GRAVEYARD_POS, 0, wp.WType.T_GRAVEYARD)
        self._routewp: list[wp.Waypoint] = []

        self.twisted_kicker = kicker.KickerAux()

        self.last_update = time()

    def update(self, robot: rbt.Robot) -> None:
        """
        Обновить маршрут

        Обновляет текущее положение робота в маршрутной карте
        """
        self._robot = [
            wp.Waypoint(robot.get_pos(), robot.get_angle(), wp.WType.T_ROBOT)
        ]

    def clear(self) -> None:
        """Очистить промежуточные точки маршрута"""
        self._routewp = []

    def __get_route(self) -> list[wp.Waypoint]:
        """Получить маршрут в виде списка путевых точек"""
        return [*self._robot, *self._routewp, self._destination]

    def set_dest_wp(self, dest: wp.Waypoint) -> None:
        """Задать конечную точку"""
        self.clear()
        self._destination = dest

    def get_dest_wp(self) -> wp.Waypoint:
        """Получить конечную точку"""
        return self._destination

    def get_next_wp(self) -> wp.Waypoint:
        """Получить следующую путевую точку"""
        return self.__get_route()[1]

    def get_next_segment(self) -> list[wp.Waypoint]:
        """Получить следующий сегмент маршрута в виде списка двух точек"""
        return self.__get_route()[0:1]

    def get_next_vec(self) -> aux.Point:
        """Получить следующий сегмент маршрута в виде вектора"""
        return self.__get_route()[1].pos - self.__get_route()[0].pos

    def get_next_angle(self) -> float:
        """Получить угол следующей путевой точки"""
        return self.__get_route()[1].angle

    def get_next_type(self) -> wp.WType:
        """Получить тип следующей путевой точки"""
        return self.__get_route()[1].type

    def insert_wp(self, wpt: wp.Waypoint) -> None:
        """Вставить промежуточную путевую точку в начало маршрута"""
        self._routewp.insert(0, wpt)

    def is_used(self) -> bool:
        """Определить, используется ли маршрут"""
        return self._destination.type != wp.WType.T_GRAVEYARD

    def get_length(self) -> float:
        """Получить длину маршрута"""
        dist = 0.0
        last_wp_pos = self._robot[0].pos
        for wpt in self.__get_route():
            if wpt.type in wp.BALL_WP_TYPES:
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
        if end_point.type == wp.WType.S_CATCH_BALL:
            robot.dribbler_speed_ = 15

        if end_point.type in wp.BALL_WP_TYPES and self.get_length() < 300:
            robot.dribbler_enable_ = 1
            if robot.dribbler_speed_ == 0:
                robot.dribbler_speed_ = 15

            robot.kicker_charge_enable_ = 1
            if robot.kicker_voltage_ == 0:
                robot.kicker_voltage_ = const.VOLTAGE_SHOOT
                if end_point.type in [wp.WType.S_BALL_GRAB, wp.WType.S_BALL_GO]:
                    robot.kicker_voltage_ = const.VOLTAGE_ZERO
                elif end_point.type in [
                    wp.WType.S_BALL_PASS,
                    wp.WType.S_BALL_TWIST_PASS,
                ]:
                    robot.kicker_voltage_ = const.VOLTAGE_PASS
                elif end_point.type == wp.WType.S_BALL_KICK_UP:
                    robot.kicker_voltage_ = const.VOLTAGE_UP

            is_aligned_by_angle = robot.is_kick_aligned_by_angle(end_point.angle)
            if (
                end_point.type in [wp.WType.S_BALL_KICK, wp.WType.S_BALL_PASS]
                and is_aligned_by_angle
            ):
                robot.auto_kick_ = 1
            elif end_point.type == wp.WType.S_BALL_KICK_UP and is_aligned_by_angle:
                robot.auto_kick_ = 2
            else:
                robot.auto_kick_ = 0

    def vel_control(
        self, robot: rbt.Robot, field: fld.Field
    ) -> tuple[aux.Point, float]:
        """set vel using waypoint"""
        target_point = self.get_next_wp()
        end_point = self.get_dest_wp()

        cur_vel = robot.get_vel()
        vec_err = target_point.pos - robot.get_pos()
        dest_vec = end_point.pos - robot.get_pos()

        transl_vel: Optional[aux.Point] = None

        self.twisted_kicker.reset_kick_consts()

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
        else:
            ball_escorting = (
                end_point.type in wp.BALL_WP_TYPES and self.get_length() < 500
            )
            if ball_escorting:
                robot.pos_reg_x.select_mode(tau.Mode.SOFT)
                robot.pos_reg_y.select_mode(tau.Mode.SOFT)
            else:
                robot.pos_reg_x.select_mode(tau.Mode.NORMAL)
                robot.pos_reg_y.select_mode(tau.Mode.NORMAL)

            u_x = robot.pos_reg_x.process(vec_err.x, -cur_vel.x)
            u_y = robot.pos_reg_y.process(vec_err.y, -cur_vel.y)

            transl_vel = aux.Point(u_x, u_y)
            if target_point.type == wp.WType.S_BALL_GRAB:
                transl_vel = get_grab_speed(
                    robot.get_pos(), transl_vel, field, target_point
                )
                if ball_escorting:
                    transl_vel += field.ball.get_vel()

            if transl_vel.mag() > const.MAX_SPEED:
                transl_vel = transl_vel.unity() * const.MAX_SPEED

            angle = end_point.angle

        return (transl_vel, angle)

    def go_route(self, robot: rbt.Robot, field: fld.Field) -> None:
        """
        Двигаться по маршруту route
        """
        self.last_update = time()

        if self.get_next_type() == wp.WType.S_VELOCITY:
            waypoint = self.get_dest_wp()
            robot.kicker_charge_enable_ = 1
            robot.speed_x = -waypoint.pos.x / robot.k_xx
            robot.speed_y = waypoint.pos.y / robot.k_yy
            robot.delta_angle = waypoint.angle
            robot.beep = 1
            return

        robot.beep = 0

        self.kicker_control(robot)

        vel, angle = self.vel_control(robot, field)  # in global coordinate system
        robot.update_vel_xy(vel)

        aerr = aux.wind_down_angle(angle - robot.get_angle())
        if const.IS_SIMULATOR_USED:
            ang_vel = robot.angle_reg.process(aerr, -robot.get_anglevel())
            robot.update_vel_w(ang_vel)
        else:
            robot.delta_angle = aerr

        reg_vel = aux.Point(robot.speed_x, -robot.speed_y)
        field.router_image.draw_line(
            robot.get_pos(),
            robot.get_pos() + aux.rotate(reg_vel, robot.get_angle()) * 10,
        )


def get_grab_speed(
    robot_pos: aux.Point, transl_vel: aux.Point, field: fld.Field, grab_wp: wp.Waypoint
) -> aux.Point:
    """Calculate speed for carefully grabbing a ball"""
    ball = field.ball.get_pos()
    grab_point = grab_wp.pos

    point_on_center_line = aux.closest_point_on_line(ball, grab_point, robot_pos, "S")
    dist_to_center_line = aux.dist(robot_pos, point_on_center_line)
    ball_dist_center_line = aux.dist(point_on_center_line, ball)
    if ball_dist_center_line == 0:
        offset_angle = 0.0
    else:
        offset_angle = math.atan(dist_to_center_line / ball_dist_center_line)

    dist_to_catch = (
        ball - aux.rotate(aux.RIGHT, grab_wp.angle) * const.GRAB_DIST
    ) - robot_pos

    vel_to_catch = dist_to_catch * const.GRAB_MULT

    vel_to_catch_r = aux.scal_mult(
        vel_to_catch,
        aux.rotate(aux.RIGHT, grab_wp.angle),
    )

    vel_to_align_r = aux.scal_mult(
        transl_vel,
        aux.rotate(aux.RIGHT, grab_wp.angle),
    )

    vel_to_align = transl_vel - aux.rotate(aux.RIGHT, grab_wp.angle) * vel_to_align_r

    board = min(
        offset_angle / const.GRAB_OFFSET_ANGLE, 1
    )  # 0 - go to ball; 1 - go to grab_point

    vel_r = vel_to_catch_r * (1 - board) + vel_to_align_r * board
    vel = vel_to_align + aux.rotate(aux.RIGHT, grab_wp.angle) * vel_r

    if aux.dist(robot_pos, grab_wp.pos) < 500:
        draw_grabbing_image(
            field,
            grab_wp,
            robot_pos,
            transl_vel,
            vel_to_catch,
            vel,
        )

    return vel


def draw_grabbing_image(
    field: fld.Field,
    grab_wp: wp.Waypoint,
    robot_pos: aux.Point,
    vel_to_align: aux.Point,
    vel_to_catch: aux.Point,
    vel: aux.Point,
) -> None:
    """Draw a screen easily debug grabbing a ball"""
    ball = field.ball.get_pos()
    grab_point = grab_wp.pos

    cord_scale = 0.8
    vel_scale = 0.4
    size = 200
    angle = -grab_wp.angle - math.pi / 2
    if ball.x > 0:
        middle = aux.Point(120, 780)
    else:
        middle = aux.Point(1080, 780)

    field.router_image.draw_rect(
        middle.x - size / 2, middle.y - size / 2, size, size, (200, 200, 200)
    )
    field.router_image.print(
        middle - aux.Point(0, size / 2 + 10),
        "GRABBING A BALL",
        need_to_scale=False,
    )

    ball_screen = middle - aux.Point(0, size // 2 - 30)
    center_boarder = convert_to_screen(ball_screen, cord_scale, angle, ball, grab_point)
    center_boarder += aux.RIGHT / 2  # чтобы не мерцало, хз
    field.router_image.draw_line(
        ball_screen, center_boarder, size_in_pixels=3, need_to_scale=False
    )

    right_boarder = convert_to_screen(
        ball_screen, 1, -const.GRAB_OFFSET_ANGLE, ball_screen, center_boarder
    )
    field.router_image.draw_line(
        ball_screen, right_boarder, size_in_pixels=3, need_to_scale=False
    )

    left_boarder = convert_to_screen(
        ball_screen, 1, const.GRAB_OFFSET_ANGLE, ball_screen, center_boarder
    )
    field.router_image.draw_line(
        ball_screen, left_boarder, size_in_pixels=3, need_to_scale=False
    )

    robot_screen = convert_to_screen(ball_screen, cord_scale, angle, ball, robot_pos)
    cropped_robot = aux.Point(
        aux.minmax(robot_screen.x, middle.x - size / 2, middle.x + size / 2),
        aux.minmax(robot_screen.y, middle.y - size / 2, middle.y + size / 2),
    )
    field.router_image.draw_dot(cropped_robot, (0, 0, 0), 80, False)

    if cropped_robot == robot_screen:
        vel_to_align_screen = convert_to_screen(
            robot_screen, vel_scale, angle, aux.Point(0, 0), vel_to_align
        )
        field.router_image.draw_line(
            robot_screen, vel_to_align_screen, (100, 100, 200), 2, need_to_scale=False
        )
        vel_to_catch_screen = convert_to_screen(
            robot_screen, vel_scale, angle, aux.Point(0, 0), vel_to_catch
        )
        field.router_image.draw_line(
            robot_screen, vel_to_catch_screen, (200, 100, 100), 2, need_to_scale=False
        )
        vel_screen = convert_to_screen(
            robot_screen, vel_scale, angle, aux.Point(0, 0), vel
        )
        field.router_image.draw_line(
            robot_screen, vel_screen, (200, 100, 200), 3, need_to_scale=False
        )

    field.router_image.draw_dot(ball_screen, (255, 100, 100), 50, False)


def convert_to_screen(
    ball_screen: aux.Point,
    scale: float,
    angle: float,
    ball: aux.Point,
    point: aux.Point,
) -> aux.Point:
    """Convert cord on field to cord on image"""
    vec_from_ball = point - ball
    scaled_vec = vec_from_ball * scale
    rotated_vec = aux.rotate(scaled_vec, angle)
    final_point = ball_screen + rotated_vec
    return final_point
