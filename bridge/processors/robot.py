"""
Описание полей и интерфейсов взаимодействия с роботом
"""
import math

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.entity as entity
import bridge.processors.field as field
import bridge.processors.tau as tau
import bridge.processors.waypoint as wp


class Robot(entity.Entity):
    """
    Описание робота
    """

    def __init__(self, pos, angle, R, color, r_id, ctrl_id):
        super().__init__(pos, angle, R)

        self.r_id = r_id
        self.ctrl_id = ctrl_id
        self.__is_used = 0
        self.color = color
        self.last_update_ = 0

        self.speed_x = 0
        self.speed_y = 0
        self.speed_r = 0
        self.kick_up_ = 0
        self.kick_forward_ = 0
        self.auto_kick_ = 0
        self.kicker_voltage_ = 0
        self.dribbler_enable_ = 0
        self.dribbler_speed_ = 0
        self.kicker_charge_enable_ = 1
        self.beep = 0
        self.role = None

        # v! SIM
        if const.IS_SIMULATOR_USED:
            self.k_xx = -833 / 20
            self.k_yy = 833 / 20
            self.k_ww = 1.25 / 20
            self.k_wy = -0.001
            self.t_wy = 0.15
            self.r_comp_f_dy = tau.FOD(self.t_wy, const.Ts)
            self.r_comp_f_fy = tau.FOLP(self.t_wy, const.Ts)

        # v! REAL
        else:
            self.k_xx = -250 / 20
            self.k_yy = 250 / 20
            self.k_ww = 6 / 20
            self.k_wy = 0
            self.t_wy = 0.15
            self.r_comp_f_dy = tau.FOD(self.t_wy, const.Ts)
            self.r_comp_f_fy = tau.FOLP(self.t_wy, const.Ts)

        # self.max_acc = 1000
        # self.xxRL = tau.RateLimiter(const.Ts, self.max_acc)
        # self.yyRL = tau.RateLimiter(const.Ts, self.max_acc)
        self.xx_t = 0.2
        self.xx_flp = tau.FOLP(self.xx_t, const.Ts)
        self.yy_t = 0.2
        self.yy_flp = tau.FOLP(self.yy_t, const.Ts)
        # self.a0TF = 0.5
        # self.a0Flp = tau.FOLP(self.a0TF, const.Ts)

        # !v REAL
        # if self.r_id != const.GK:
        gains_full = [6, 0.8, 0, const.MAX_SPEED]
        gains_soft = [8, 0.5, 0, const.SOFT_MAX_SPEED]
        # gains_soft = gains_full
        a_gains_full = [4, 0.1, 0.1, const.MAX_SPEED_R]
        a_gains_soft = [4, 0.07, 4, const.SOFT_MAX_SPEED_R]
        # a_gains_soft = a_gains_full
        # else:
        #     gains_full = [6, 0.8, 0, const.MAX_SPEED]
        #     gains_soft = [6, 1, 0.1, const.SOFT_MAX_SPEED]
        #     a_gains_full = [6, 0.1, 0, const.MAX_SPEED_R]
        #     a_gains_soft = [2, 0.07, 1, const.SOFT_MAX_SPEED_R]

        # !v SIM
        # gains_full = [2, 0.3, 0, const.MAX_SPEED]
        # gains_soft = [0.5, 0.1, 0, const.SOFT_MAX_SPEED]
        # a_gains_full = [2, 0.1, 0.1, const.MAX_SPEED_R]
        # a_gains_soft = [1, 0.07, 0, const.SOFT_MAX_SPEED_R]

        self.pos_reg = tau.PISD(
            const.Ts,
            [gains_full[0], gains_soft[0]],
            [gains_full[1], gains_soft[1]],
            [gains_full[2], gains_soft[2]],
            [gains_full[3], gains_soft[3]],
        )
        self.angle_reg = tau.PISD(
            const.Ts,
            [a_gains_full[0], a_gains_soft[0]],
            [a_gains_full[1], a_gains_soft[1]],
            [a_gains_full[2], a_gains_soft[2]],
            [a_gains_full[3], a_gains_soft[3]],
        )

        self.is_kick_commited = False

    def used(self, a) -> None:
        """
        Выставить флаг использования робота
        """
        self.__is_used = a

    def is_used(self):
        """
        Узнать, используется ли робот
        """
        return self.__is_used

    def last_update(self):
        """
        Получить время последнего обновления робота
        """
        return self.last_update_

    def update(self, pos, angle, t):
        """
        Обновить состояние робота согласно SSL Vision

        TODO добавить в Entity время последнего обновления
        """
        super().update(pos, angle)
        self.kick_forward_ = 0
        self.kick_up_ = 0
        self.last_update_ = t

    def kick_forward(self):
        """
        Ударить вперед
        """
        self.kick_forward_ = 1

    def kick_up(self):
        """
        Ударить вверх
        """
        self.kick_up_ = 1

    def copy_control_fields(self, robot):
        """
        Скопировать в данный робот поля управления робота robot
        """
        self.speed_x = robot.speed_x
        self.speed_y = robot.speed_y
        self.speed_r = robot.speed_r
        self.kick_up_ = robot.kick_up_
        self.kick_forward_ = robot.kick_forward_
        self.auto_kick_ = robot.auto_kick_
        self.kicker_voltage_ = robot.kicker_voltage_
        self.dribbler_enable_ = robot.dribbler_enable_
        self.dribbler_speed_ = robot.dribbler_speed_
        self.kicker_charge_enable_ = robot.kicker_charge_enable_
        self.beep = robot.beep

    def clear_fields(self):
        """
        Очистить поля управления
        """
        self.speed_x = 0
        self.speed_y = 0
        self.speed_r = 0
        self.kick_up_ = 0
        self.kick_forward_ = 0
        self.auto_kick_ = 0
        self.kicker_voltage_ = 0
        self.dribbler_enable_ = 0
        self.dribbler_speed_ = 0
        self.kicker_charge_enable_ = 0
        self.beep = 0

    def is_kick_aligned(self, target: wp.Waypoint):
        """
        Определить, выровнен ли робот относительно путевой точки target
        """
        # print(round((self.getPos() - target.pos).mag(), 2),
        #       const.KICK_ALIGN_DIST*const.KICK_ALIGN_DIST_MULT, \
        #       round(abs(aux.wind_down_angle(self._angle - target.angle)), 2),
        #       const.KICK_ALIGN_ANGLE, \
        #     # round(abs(aux.vect_mult(
        #           aux.rotate(aux.i, target.angle), target.pos - self._pos)), 2),
        #           const.KICK_ALIGN_OFFSET)
        #       round(aux.dist(aux.closest_point_on_line(
        #           target.pos,
        #           target.pos - aux.rotate(aux.i, target.angle)*const.KICK_ALIGN_DIST, self._pos),
        #           self._pos)),
        #           const.KICK_ALIGN_OFFSET)
        # print(aux.i, target.angle, aux.rotate(aux.i, target.angle), target.pos,
        #                                       self._pos, target.pos - self._pos)

        commit_scale = 1.2 if self.is_kick_commited else 1
        is_dist = (self.get_pos() - target.pos).mag() < const.KICK_ALIGN_DIST * const.KICK_ALIGN_DIST_MULT * commit_scale
        is_angle = abs(aux.wind_down_angle(self._angle - target.angle)) < const.KICK_ALIGN_ANGLE * commit_scale
        is_offset = (
            aux.dist(
                aux.closest_point_on_line(
                    target.pos, target.pos - aux.rotate(aux.RIGHT, target.angle) * const.KICK_ALIGN_DIST, self._pos
                ),
                self._pos,
            )
            < const.KICK_ALIGN_OFFSET * commit_scale
        )
        is_aligned = is_dist and is_angle and is_offset

        if is_aligned:
            self.is_kick_commited = True
        else:
            self.is_kick_commited = False
        print(is_dist, is_angle, is_offset)
        return is_aligned

    def is_ball_in(self, fld: field.Field):
        """
        Определить, находится ли мяч внутри дриблера
        """
        return (self.get_pos() - fld.ball.get_pos()).mag() < const.BALL_GRABBED_DIST and abs(
            aux.wind_down_angle((fld.ball.get_pos() - self.get_pos()).arg() - self._angle)
        ) < const.BALL_GRABBED_ANGLE

    def go_route(self, route, fld: field.Field):
        """
        Двигаться по маршруту route
        """
        cur_speed = self._vel.mag()

        # print(route)

        dist = route.get_length()

        target_point = route.get_next_wp()
        end_point = route.get_dest_wp()

        # print(self.getPos(), target_point, end_point)
        vel0 = (self.get_pos() - target_point.pos).unity()

        dangle = (target_point.pos - self.get_pos()).arg()
        rangle = aux.wind_down_angle(self._angle - dangle)
        twpangle = aux.wind_down_angle(target_point.angle - dangle)

        angle60_abs = math.pi / 6 if abs(rangle) < math.pi / 2 else 2 * math.pi / 6
        # angle60_abs = 0
        angle60_sign = aux.sign(twpangle + rangle)

        angle60 = dangle + angle60_abs * angle60_sign

        lerp_angles = [target_point.angle, angle60]

        angle0 = aux.lerp(lerp_angles[0], lerp_angles[1], aux.minmax((dist - 100) / 1000, 0, 1))

        self.pos_reg.select_mode(tau.Mode.NORMAL)

        if (
            end_point.type == wp.WType.S_BALL_KICK
            or end_point.type == wp.WType.S_BALL_GRAB
            or end_point.type == wp.WType.S_BALL_GO
        ) and dist < 1500:

            print("IS KICK ALIGNED: ", self.is_kick_aligned(end_point), ",\tIS BALL GRABBED: ", self.is_ball_in(fld))

            self.pos_reg.select_mode(tau.Mode.SOFT)

            if end_point.type == wp.WType.S_BALL_GO:
                angle0 = end_point.angle

            self.dribbler_enable_ = True
            self.dribbler_speed_ = 15
            self.kicker_voltage_ = 15
        else:
            self.dribbler_enable_ = False

        if (end_point.type == wp.WType.S_BALL_KICK or end_point.type == wp.WType.S_BALL_GRAB) and (
            self.is_kick_aligned(end_point) or self.is_ball_in(fld)
        ):
            # vel0 = (self.getPos() - end_point.pos).unity()
            vel0 = -aux.rotate(aux.RIGHT, self._angle)
            # angle0 = end_point.angle
            angle0 = self._angle

            if end_point.type == wp.WType.S_BALL_KICK:
                # self.auto_kick_ = 2 if self.r_id == const.GK else 1
                if self._pos.x * fld.side > 500:
                    self.auto_kick_ = 2
                else:
                    self.auto_kick_ = 1
            else:
                self.auto_kick_ = 0

            transl_vel = vel0 * 300

        else:
            self.auto_kick_ = 0

            err = dist
            u = self.pos_reg.process(err, -cur_speed)
            transl_vel = vel0 * u

        angle0 = end_point.angle
        aerr = aux.wind_down_angle(angle0 - self.get_angle())

        u_a = self.angle_reg.process(aerr, -self._anglevel)
        ang_vel = u_a

        # if self.r_id == const.GK and self.color == 'b':
        #     print("is aligned: ", self.is_kick_aligned(end_point), ",
        #               angle60: ", '%.2f'%angle60, ", end angle: ", '%.2f'%end_point.angle, \
        #               ", angle0: ", '%.2f'%angle0, ", aerr: ", '%.2f'%aerr, ",
        #               self angle: ", '%.2f'%self.getAngle())
        self.update_vel_xyw(transl_vel, ang_vel)

    def update_vel_xyw(self, vel: aux.Point, wvel: float):
        """
        Выполнить тик низкоуровневых регуляторов скорости робота

        vel - требуемый вектор скорости [мм/с] \\
        wvel - требуемая угловая скорость [рад/с]
        """
        self.speed_x = self.xx_flp.process(1 / self.k_xx * aux.rotate(vel, -self._angle).x)
        self.speed_y = self.yy_flp.process(1 / self.k_yy * aux.rotate(vel, -self._angle).y)

        # RcompY = self.Kwy * self.RcompFfy.process(self.RcompFdy.process(self.speed_y))
        # RcompY = self.Kwy * self.RcompFdy.process(abs(float(self.speed_y)**2))
        r_comp_y = 0
        self.speed_r = 1 / self.k_ww * (wvel - r_comp_y)
        if abs(self.speed_r) > const.MAX_SPEED_R:
            self.speed_r = const.MAX_SPEED_R * abs(self.speed_r) / self.speed_r

        vec_speed = math.sqrt(self.speed_x**2 + self.speed_y**2)
        r_speed = abs(self.speed_r)

        vec_speed *= ((const.MAX_SPEED_R - r_speed) / const.MAX_SPEED_R) ** 8
        ang = math.atan2(self.speed_y, self.speed_x)
        self.speed_x = vec_speed * math.cos(ang)
        self.speed_y = vec_speed * math.sin(ang)

    def clamp_motors(self):
        """
        Ограничить управляющее воздействие
        """
        # TODO

    def __str__(self) -> str:
        return (
            str(
                str(self.color)
                + " "
                + str(self.r_id)
                + " "
                + str(self.get_pos())
                + " "
                + str(self.speed_x)
                + " "
                + str(self.speed_y)
            )
            + " "
            + str(self.speed_r)
        )
