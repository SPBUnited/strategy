"""
Описание полей и интерфейсов взаимодействия с роботом
"""

import typing

import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, entity, tau


class Robot(entity.Entity):
    """
    Описание робота
    """

    def __init__(
        self,
        pos: aux.Point,
        angle: float,
        R: float,
        color: const.Color,
        r_id: int,
    ) -> None:
        super().__init__(pos, angle, R)

        self.r_id = r_id
        self._is_used = 0
        self.color = color
        self.last_update_ = 0.0

        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_r = 0.0
        self.delta_angle = 0.0
        self.kick_up_ = 0
        self.kick_forward_ = 0
        self.auto_kick_ = 0
        self.kicker_voltage_ = 0
        self.dribbler_enable_ = 0
        self.dribbler_speed_ = 0
        self.kicker_charge_enable_ = 0
        self.beep = 0

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
            self.k_xx = -1400 / 100
            self.k_yy = 1200 / 100
            self.k_ww = 6 / 20
            self.k_wy = 0
            self.t_wy = 0.15
            self.r_comp_f_dy = tau.FOD(self.t_wy, const.Ts)
            self.r_comp_f_fy = tau.FOLP(self.t_wy, const.Ts)

        # self.max_acc = 1000
        # self.xxRL = tau.RateLimiter(const.Ts, self.max_acc)
        # self.yyRL = tau.RateLimiter(const.Ts, self.max_acc)
        self.xx_t = 0.1
        self.xx_flp = tau.FOLP(self.xx_t, const.Ts)
        self.yy_t = 0.1
        self.yy_flp = tau.FOLP(self.yy_t, const.Ts)
        # self.a0TF = 0.5
        # self.a0Flp = tau.FOLP(self.a0TF, const.Ts)

        # !v REAL
        gains_full = [2.5, 0.15, 2.5, const.MAX_SPEED]
        gains_soft = [2, 0.1, 2, const.SOFT_MAX_SPEED]
        a_gains_full = [8, 0.1, 0, const.MAX_SPEED_R]
        # gains_soft = [10, 0.32, 0, const.SOFT_MAX_SPEED]
        # gains_soft = gains_full
        if const.IS_SIMULATOR_USED:
            # gains_full = [8, 0.35, 0, const.MAX_SPEED]
            gains_full = [2.5, 0.08, 0.4, const.MAX_SPEED]
            gains_soft = [2.5, 0.08, 0, const.SOFT_MAX_SPEED]
            a_gains_full = [2, 0.1, 0.1, const.MAX_SPEED_R]
        # a_gains_soft = [4, 0.07, 8, const.SOFT_MAX_SPEED_R]
        a_gains_soft = a_gains_full
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

        self.pos_reg_x = tau.PISD(
            const.Ts,
            [gains_full[0], gains_soft[0]],
            [gains_full[1], gains_soft[1]],
            [gains_full[2], gains_soft[2]],
            [gains_full[3], gains_soft[3]],
        )
        self.pos_reg_y = tau.PISD(
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

        self.is_kick_committed = False

    def __eq__(self, robo: typing.Any) -> bool:
        if not isinstance(robo, Robot):
            return False
        return self.r_id == robo.r_id and self.color == robo.color

    def to_entity(self) -> entity.Entity:
        ent = entity.Entity(self._pos, self._angle, self._radius)
        ent._vel = self._vel
        # ent._acc = self._acc
        return ent

    def used(self, a: int) -> None:
        """
        Выставить флаг использования робота
        """
        self._is_used = a

    def is_used(self) -> int:
        """
        Узнать, используется ли робот
        """
        return self._is_used

    def last_update(self) -> float:
        """
        Получить время последнего обновления робота
        """
        return self.last_update_

    def update(self, pos: aux.Point, angle: float, t: float) -> None:
        """
        Обновить состояние робота согласно SSL Vision
        """
        super().update(pos, angle, t)
        self.kick_forward_ = 0
        self.kick_up_ = 0
        self.last_update_ = t

    def update_(self, new_entity: "Robot") -> None:
        """
        Обновить состояние робота используя готовые данные
        """
        self._pos = new_entity.get_pos()
        self._vel = new_entity.get_vel()
        self._angle = new_entity.get_angle()
        self._anglevel = new_entity.get_anglevel()

        self._is_used = new_entity.is_used()
        self.last_update_ = new_entity.last_update()

    def kick_forward(self) -> None:
        """
        Ударить вперед
        """
        self.kick_forward_ = 1

    def kick_up(self) -> None:
        """
        Ударить вверх
        """
        self.kick_up_ = 1

    def set_dribbler_speed(self, speed: float) -> None:
        """
        Включить дриблер и задать его скорость
        """
        self.dribbler_enable_ = True
        self.dribbler_speed_ = round(aux.minmax(speed, 0.0, 15.0))

    def copy_control_fields(self, robot: "Robot") -> None:
        """
        Скопировать в данный "робот" поля управления робота robot
        """
        self.speed_x = robot.speed_x
        self.speed_y = robot.speed_y
        self.speed_r = robot.speed_r
        self.delta_angle = robot.delta_angle
        self.kick_up_ = robot.kick_up_
        self.kick_forward_ = robot.kick_forward_
        self.auto_kick_ = robot.auto_kick_
        self.kicker_voltage_ = robot.kicker_voltage_
        self.dribbler_enable_ = robot.dribbler_enable_
        self.dribbler_speed_ = robot.dribbler_speed_
        self.kicker_charge_enable_ = robot.kicker_charge_enable_
        self.beep = robot.beep
        self._is_used = robot.is_used()
        self.last_update_ = robot.last_update_

    def clear_fields(self) -> None:
        """
        Очистить поля управления
        """
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_r = 0.0
        self.delta_angle = 0.0
        self.kick_up_ = 0
        self.kick_forward_ = 0
        self.auto_kick_ = 0
        self.kicker_voltage_ = 0
        self.dribbler_enable_ = 0
        self.dribbler_speed_ = 0
        self.kicker_charge_enable_ = 0
        self.beep = 0

    def is_kick_aligned(self, target: wp.Waypoint) -> bool:
        """
        Определить, выровнен ли робот относительно путевой точки target
        """

        commit_scale = 1.2 if self.is_kick_committed else 1
        is_dist = (self.get_pos() - target.pos).mag() < const.KICK_ALIGN_DIST * const.KICK_ALIGN_DIST_MULT * commit_scale
        is_angle = self.is_kick_aligned_by_angle(target.angle)
        is_offset = (
            aux.dist(
                aux.closest_point_on_line(
                    target.pos,
                    target.pos - aux.rotate(aux.RIGHT, target.angle) * const.KICK_ALIGN_DIST,
                    self._pos,
                ),
                self._pos,
            )
            < const.KICK_ALIGN_OFFSET * commit_scale
        )
        is_aligned = is_dist and is_angle and is_offset

        if is_aligned:
            self.is_kick_committed = True
        else:
            self.is_kick_committed = False

        return is_aligned

    def is_kick_aligned_by_angle(self, angle: float) -> bool:
        """
        Определить, выровнен ли робот относительно путевой точки target
        """
        commit_scale = 1.2 if self.is_kick_committed else 1
        return abs(aux.wind_down_angle(self._angle - angle)) < const.KICK_ALIGN_ANGLE * commit_scale

    def update_vel_xy(self, vel: aux.Point) -> None:
        """
        Выполнить тик низкоуровневых регуляторов скорости робота

        vel - требуемый вектор скорости [мм/с]
        """
        global_speed_x = self.xx_flp.process(vel.x)
        global_speed_y = self.yy_flp.process(vel.y)

        global_speed = aux.Point(global_speed_x, global_speed_y)

        speed = -aux.rotate(global_speed, -self._angle)

        self.speed_x = 1 / self.k_xx * speed.x
        self.speed_y = 1 / self.k_yy * speed.y

        # if abs(self.speed_r) > const.MAX_SPEED_R:
        #     self.speed_r = math.copysign(const.MAX_SPEED_R, self.speed_r)

        # vec_speed = math.sqrt(self.speed_x**2 + self.speed_y**2)
        # r_speed = abs(self.speed_r)
        # if not const.IS_SIMULATOR_USED:
        #     vec_speed *= abs((const.MAX_SPEED_R - r_speed) / const.MAX_SPEED_R) ** 2
        # ang = math.atan2(self.speed_y, self.speed_x)

        # self.speed_x = vec_speed * math.cos(ang)
        # self.speed_y = vec_speed * math.sin(ang)

    def update_vel_w(self, wvel: float) -> None:
        """Update robot angle vel"""
        self.speed_r = 1 / self.k_ww * wvel

    def update_vel_xy_(self, vel: aux.Point, dT: float) -> None:
        """
        Выполнить тик низкоуровневых регуляторов скорости робота

        vel - требуемый вектор скорости [мм/с]
        """
        global_speed_x = self.xx_flp.process_(vel.x, dT)
        global_speed_y = self.yy_flp.process_(vel.y, dT)

        global_speed = aux.Point(global_speed_x, global_speed_y)

        speed = -aux.rotate(global_speed, -self._angle)

        self.speed_x = 1 / self.k_xx * speed.x
        self.speed_y = 1 / self.k_yy * speed.y

        # if abs(self.speed_r) > const.MAX_SPEED_R:
        #     self.speed_r = math.copysign(const.MAX_SPEED_R, self.speed_r)

        # vec_speed = math.sqrt(self.speed_x**2 + self.speed_y**2)
        # r_speed = abs(self.speed_r)
        # if not const.IS_SIMULATOR_USED:
        #     vec_speed *= abs((const.MAX_SPEED_R - r_speed) / const.MAX_SPEED_R) ** 2
        # ang = math.atan2(self.speed_y, self.speed_x)

        # self.speed_x = vec_speed * math.cos(ang)
        # self.speed_y = vec_speed * math.sin(ang)

    def clamp_motors(self) -> None:
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
