"""
Класс, описывающий текущий маршрут робота
"""

import math

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as field
import bridge.processors.robot as robot
import bridge.processors.tau as tau
import bridge.processors.waypoint as wp


class Route:
    """
    Класс описание произвольного маршрута
    """

    def __init__(self, rbt: robot.Robot):
        """
        Конструктор
        """
        self._robot = [wp.Waypoint(rbt.get_pos(), rbt._angle, wp.WType.T_ROBOT)]
        self._destination = [wp.Waypoint(aux.GRAVEYARD_POS, 0, wp.WType.T_GRAVEYARD)]
        self._routewp: list[wp.Waypoint] = []
        # self.__route = [*self.robot, *self.__routewp, *self.__destination]

    def update(self, rbt: robot.Robot) -> None:
        """
        Обновить маршрут

        Обновляет текущее положение робота в маршрутной карте
        """
        self._robot = [wp.Waypoint(rbt.get_pos(), rbt.get_angle(), wp.WType.T_ROBOT)]
        # if self.getNextVec().mag() < const.VANISH_DIST and \
        #     len(self._routewp) >= 1:
        #     del self._routewp[0]

    def clear(self) -> None:
        """
        Очистить промежуточные точки маршрута
        """
        self._routewp = []

    def __get_route(self) -> list[wp.Waypoint]:
        """
        Получить маршрут в виде списка путевых точек
        """
        return [*self._robot, *self._routewp, *self._destination]

    def set_dest_wp(self, dest: wp.Waypoint) -> None:
        """
        Задать конечную точку
        """
        self._destination = [dest]

    def get_dest_wp(self) -> wp.Waypoint:
        """
        Получить конечную точку
        """
        return self._destination[0]

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
        return self._destination[0].type != wp.WType.T_GRAVEYARD

    def get_length(self) -> float:
        """
        Получить длину маршрута
        """
        dist = 0.0
        last_wp_pos = self._robot[0].pos
        for wpt in self.__get_route():
            if wpt.type == wp.WType.S_BALL_GO or wpt.type == wp.WType.S_BALL_KICK or wpt.type == wp.WType.S_BALL_GRAB:
                break
            dist += (wpt.pos - last_wp_pos).mag()
            last_wp_pos = wpt.pos
        return dist

    def __str__(self) -> str:
        strin = "ROUTE: "
        for wpt in self.__get_route():
            strin += " ->\n" + str(wpt)
        # for wp in [*self.robot, *self.__routewp, *self.__destination]:
        #     strin += " -> " + str(wp)
        return strin

    def go_route(self, robot: robot.Robot, fld: field.Field) -> None:
        """
        Двигаться по маршруту route
        """
        cur_speed = robot.get_vel().mag()

        # print(self)

        dist = self.get_length()

        target_point = self.get_next_wp()
        end_point = self.get_dest_wp()

        # print(robot.getPos(), target_point, end_point)
        vel0 = (robot.get_pos() - target_point.pos).unity()

        dangle = (target_point.pos - robot.get_pos()).arg()
        rangle = aux.wind_down_angle(robot._angle - dangle)
        twpangle = aux.wind_down_angle(target_point.angle - dangle)

        angle60_abs = math.pi / 6 if abs(rangle) < math.pi / 2 else 2 * math.pi / 6
        # angle60_abs = 0
        angle60_sign = aux.sign(twpangle + rangle)

        angle60 = dangle + angle60_abs * angle60_sign

        lerp_angles = [target_point.angle, angle60]

        angle0 = aux.lerp(lerp_angles[0], lerp_angles[1], aux.minmax((dist - 100) / 1000, 0, 1))

        robot.pos_reg.select_mode(tau.Mode.NORMAL)

        if (
            end_point.type == wp.WType.S_BALL_KICK
            or end_point.type == wp.WType.S_BALL_GRAB
            or end_point.type == wp.WType.S_BALL_GO
        ) and dist < 1500:

            print("IS KICK ALIGNED: ", robot.is_kick_aligned(end_point), ",\tIS BALL GRABBED: ", fld.is_ball_in(robot))

            robot.pos_reg.select_mode(tau.Mode.SOFT)

            if end_point.type == wp.WType.S_BALL_GO:
                angle0 = end_point.angle

            robot.dribbler_enable_ = True
            robot.dribbler_speed_ = 15
            robot.kicker_voltage_ = 15
        else:
            robot.dribbler_enable_ = False

        if (end_point.type == wp.WType.S_BALL_KICK or end_point.type == wp.WType.S_BALL_GRAB) and (
            robot.is_kick_aligned(end_point) or fld.is_ball_in(robot)
        ):
            # vel0 = (robot.getPos() - end_point.pos).unity()
            vel0 = -aux.rotate(aux.RIGHT, robot._angle)
            # angle0 = end_point.angle
            angle0 = robot._angle

            if end_point.type == wp.WType.S_BALL_KICK:
                # robot.auto_kick_ = 2 if robot.r_id == const.GK else 1
                if robot._pos.x * const.POLARITY > 500:
                    robot.auto_kick_ = 2
                else:
                    robot.auto_kick_ = 1
            else:
                robot.auto_kick_ = 0

            transl_vel = vel0 * 300

        else:
            robot.auto_kick_ = 0

            err = dist
            u = robot.pos_reg.process(err, -cur_speed)
            transl_vel = vel0 * u

        angle0 = end_point.angle
        aerr = aux.wind_down_angle(angle0 - robot.get_angle())

        ang_vel = robot.angle_reg.process(aerr, -robot.get_anglevel())

        # if robot.r_id == const.GK and robot.color == 'b':
        #     print("is aligned: ", robot.is_kick_aligned(end_point), ",
        #               angle60: ", '%.2f'%angle60, ", end angle: ", '%.2f'%end_point.angle, \
        #               ", angle0: ", '%.2f'%angle0, ", aerr: ", '%.2f'%aerr, ",
        #               robot angle: ", '%.2f'%robot.getAngle())
        robot.update_vel_xyw(transl_vel, ang_vel)
