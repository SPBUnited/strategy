"""
Класс, описывающий текущий маршрут робота
"""

import math

import bridge.processors.auxiliary as aux
import bridge.processors.field as field
import bridge.processors.robot as robot
import bridge.processors.tau as tau
import bridge.processors.waypoint as wp
import bridge.processors.drawing as draw


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

    # def go_route(self, rbt: robot.Robot, fld: field.Field) -> None:
    #     """
    #     Двигаться по маршруту route
    #     """

    #     target_point = self.get_next_wp()
    #     end_point = self.get_dest_wp()

    #     vec_err = target_point.pos - rbt.get_pos()
    #     cur_vel = rbt.get_vel()

    #     vec_err_lcf = aux.rotate(vec_err, -rbt.get_angle())
    #     cur_vel_lcf = aux.rotate(cur_vel, -rbt.get_angle())

    #     dist = vec_err_lcf.mag()

    #     rbt.pos_reg_x.select_mode(tau.Mode.NORMAL)
    #     rbt.pos_reg_y.select_mode(tau.Mode.NORMAL)


    #     if (
    #         end_point.type == wp.WType.S_BALL_KICK
    #         or end_point.type == wp.WType.S_BALL_GRAB
    #         or end_point.type == wp.WType.S_BALL_GO
    #     ) and dist < 1500:
            
    #         rbt.pos_reg_x.select_mode(tau.Mode.SOFT)
    #         rbt.pos_reg_y.select_mode(tau.Mode.SOFT)

    #         if end_point.type == wp.WType.S_BALL_GO:
    #             angle0 = end_point.angle

    #         rbt.dribbler_enable_ = True
    #         rbt.dribbler_speed_ = 15
    #         rbt.kicker_voltage_ = 10
    #     else:
    #         rbt.dribbler_enable_ = False

    #     if (end_point.type == wp.WType.S_BALL_KICK or end_point.type == wp.WType.S_BALL_GRAB) and (
    #         rbt.is_kick_aligned(end_point) or fld.is_ball_in(rbt)
    #     ):
    #         angle0 = rbt.get_angle()

    #         if end_point.type == wp.WType.S_BALL_KICK:
    #             rbt.auto_kick_ = 1
    #         else:
    #             rbt.auto_kick_ = 0

    #         transl_vel = aux.Point(200, 0)

    #     else:
    #         rbt.auto_kick_ = 0

    #         u_x = rbt.pos_reg_x.process(vec_err.x, -cur_vel_lcf.x)
    #         u_y = -rbt.pos_reg_y.process(vec_err.y, -cur_vel_lcf.y)
            
    #         transl_vel = aux.Point(u_x, u_y)
    #         angle0 = end_point.angle
            
        
    #     aerr = aux.wind_down_angle(angle0 - rbt.get_angle())

    #     ang_vel = rbt.angle_reg.process(aerr, -rbt.get_anglevel())

    #     if end_point.type == wp.WType.S_STOP:
    #         transl_vel = aux.Point(0, 0)
    #         ang_vel = 0.0

    #     # transl_vel = aux.Point(100, 100)

    #     rbt.update_vel_xyw(transl_vel, ang_vel)

####################

    def go_route(self, rbt: robot.Robot, fld: field.Field) -> None:
        """
        Двигаться по маршруту route
        """
        cur_vel = rbt.get_vel()

      
        # print(self)

        dist = self.get_length()



        target_point = self.get_next_wp()

        # target_point.pos = aux.Point(0, 0)

        end_point = self.get_dest_wp()

        vec_err = target_point.pos - rbt.get_pos()


        # #   NOTE: kostil!!!!!!!
        # if (dist > 1500):
        #     end_point.angle = aux.angle_to_point(rbt.get_pos(), end_point.pos) 

        # print(rbt.get_pos(), target_point, end_point)
        vel0 = (rbt.get_pos() - target_point.pos).unity()

        dangle = (target_point.pos - rbt.get_pos()).arg()
        rangle = aux.wind_down_angle(rbt.get_angle() - dangle)
        twpangle = aux.wind_down_angle(target_point.angle - dangle)

        angle60_abs = math.pi / 6 if abs(rangle) < math.pi / 2 else 2 * math.pi / 6
        # angle60_abs = 0
        angle60_sign = aux.sign(twpangle + rangle)

        angle60 = dangle + angle60_abs * angle60_sign

        lerp_angles = [target_point.angle, angle60]

        angle0 = aux.lerp(lerp_angles[0], lerp_angles[1], aux.minmax((dist - 100) / 1000, 0, 1))

        rbt.pos_reg_x.select_mode(tau.Mode.NORMAL)
        rbt.pos_reg_y.select_mode(tau.Mode.NORMAL)

        if (
            end_point.type == wp.WType.S_BALL_KICK
            or end_point.type == wp.WType.S_BALL_GRAB
            or end_point.type == wp.WType.S_BALL_GO
        ) and dist < 500:

            # print("IS KICK ALIGNED: ", rbt.is_kick_aligned(end_point), ",\tIS BALL GRABBED: ", fld.is_ball_in(rbt))

            rbt.pos_reg_x.select_mode(tau.Mode.SOFT)
            rbt.pos_reg_y.select_mode(tau.Mode.SOFT)

            if end_point.type == wp.WType.S_BALL_GO:
                angle0 = end_point.angle

            rbt.dribbler_enable_ = True
            rbt.dribbler_speed_ = 15
            rbt.kicker_voltage_ = 10
        else:
            rbt.dribbler_enable_ = False

        if (end_point.type == wp.WType.S_BALL_KICK or end_point.type == wp.WType.S_BALL_GRAB) and (
            rbt.is_kick_aligned(end_point) or fld.is_ball_in(rbt)
        ):
            # vel0 = (rbt.get_pos() - end_point.pos).unity()
            vel0 = -aux.rotate(aux.RIGHT, rbt.get_angle())
            # angle0 = end_point.angle
            angle0 = rbt.get_angle()

            if end_point.type == wp.WType.S_BALL_KICK:
                # rbt.auto_kick_ = 2 if rbt.r_id == const.GK else 1
                # if rbt.get_pos().x * field.polarity > 500:
                #     rbt.auto_kick_ = 2
                # else:
                rbt.auto_kick_ = 1
            else:
                rbt.auto_kick_ = 0

            transl_vel = vel0 * 200

        else:
            rbt.auto_kick_ = 0

            u_x = -rbt.pos_reg_x.process(vec_err.x, -cur_vel.x)
            u_y = -rbt.pos_reg_y.process(vec_err.y, -cur_vel.y)
            # transl_vel = vel0 * u
            transl_vel = aux.Point(u_x, u_y)
            angle0 = end_point.angle

        aerr = aux.wind_down_angle(angle0 - rbt.get_angle())

        ang_vel = rbt.angle_reg.process(aerr, -rbt.get_anglevel())

        # if rbt.r_id == const.GK and rbt.color == 'b':
            # print("is aligned: ", rbt.is_kick_aligned(end_point), ",
            #           angle60: ", '%.2f'%angle60, ", end angle: ", '%.2f'%end_point.angle, \
            #           ", angle0: ", '%.2f'%angle0, ", aerr: ", '%.2f'%aerr, ",
            #           rbt angle: ", '%.2f'%rbt.getAngle())
        if end_point.type == wp.WType.S_STOP:
            transl_vel = aux.Point(0, 0)
            ang_vel = 0.0


        # vel0 = -aux.rotate(aux.RIGHT, rbt.get_angle())
        # transl_vel = vel0 * 200
        # ang_vel = 0.7

        rbt.update_vel_xyw(transl_vel, ang_vel)
