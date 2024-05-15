"""Верхнеуровневый код стратегии"""
# pylint: disable=redefined-outer-name

# @package Strategy
# Расчет требуемых положений роботов исходя из ситуации на поле

import math

# !v DEBUG ONLY
import typing
from enum import Enum
from time import time
from typing import Optional

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.drawing as draw
import bridge.processors.field as fld
import bridge.processors.robot as robot
import bridge.processors.signal as signal
import bridge.processors.waypoint as wp


class States(Enum):
    """Класс с глобальными состояниями игры"""

    DEBUG = 0
    DEFENSE = 1
    ATTACK = 2


class GameStates(Enum):
    """Класс с командами от судей"""

    HALT = 0
    STOP = 1
    RUN = 2
    TIMEOUT = 3
    PREPARE_KICKOFF = 5
    KICKOFF = 6
    PREPARE_PENALTY = 7
    PENALTY = 8
    FREE_KICK = 9
    BALL_PLACEMENT = 11


class ActiveTeam(Enum):
    """Класс с командами"""

    ALL = 0
    YELLOW = 1
    BLUE = 2


class Role(Enum):
    """Класс с ролями"""

    GOALKEEPER = 0
    ATTACKER = 1

    WALLLINER = 3
    FORWARD = 11

    UNAVAILABLE = 100


class Strategy:
    """Основной класс с кодом стратегии"""

    def __init__(self, dbg_game_status: GameStates = GameStates.RUN, dbg_state: States = States.ATTACK) -> None:

        self.game_status = dbg_game_status
        self.active_team: ActiveTeam = ActiveTeam.ALL
        self.state = dbg_state

        self.image = draw.Image()

    def change_game_state(self, new_state: GameStates, upd_active_team: int) -> None:
        """Изменение состояния игры и цвета команды"""
        self.game_status = new_state
        if upd_active_team == 0:
            self.active_team = ActiveTeam.ALL
        elif upd_active_team == 2:
            self.active_team = ActiveTeam.YELLOW
        elif upd_active_team == 1:
            self.active_team = ActiveTeam.BLUE

    def process(self, field: fld.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """
        # if      self.active_team == ActiveTeam.ALL or \
        #         field.ally_color == "b" and self.active_team == ActiveTeam.BLUE or \
        #         field.ally_color == "y" and self.active_team == ActiveTeam.YELLOW:
        #     self.we_active = 1

        waypoints: list[wp.Waypoint] = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoints.append(wp.Waypoint(field.allies[i].get_pos(), field.allies[i].get_angle(), wp.WType.S_STOP))

        if self.game_status == GameStates.RUN:
            self.run(field, waypoints)
        # else:
        #     if self.game_status == GameStates.TIMEOUT:
        #         self.refs.timeout(field, waypoints)
        #     elif self.game_status == GameStates.HALT:
        #         pass
        #         # self.halt(field, waypoints)
        #     elif self.game_status == GameStates.PREPARE_PENALTY:
        #         self.refs.prepare_penalty(field, waypoints)
        #     elif self.game_status == GameStates.PENALTY:
        #         if self.refs.we_kick or 1:
        #             self.refs.penalty_kick(field, waypoints)
        #         else:
        #             robot_with_ball = fld.find_nearest_robot(field.ball.get_pos(), field.enemies)
        #             waypoints[const.GK] = self.goalk(field, 0, robot_with_ball)[0]
        #     elif self.game_status == GameStates.BALL_PLACEMENT:
        #         self.refs.keep_distance(field, waypoints)
        #     elif self.game_status == GameStates.PREPARE_KICKOFF:
        #         self.refs.prepare_kickoff(field, waypoints)
        #     elif self.game_status == GameStates.KICKOFF:
        #         self.refs.kickoff(field, waypoints)
        #         robot_with_ball = fld.find_nearest_robot(field.ball.get_pos(), field.enemies)
        #         waypoints[const.GK] = self.goalk(field, 0, robot_with_ball)[0]
        #     # elif self.game_status == GameStates.FREE_KICK:
        #     #     self.free_kick(field, waypoints)
        #     elif self.game_status == GameStates.STOP:
        #         self.refs.keep_distance(field, waypoints)
        # # print(self.game_status, self.state)

        for rbt in field.allies:
            if rbt.is_used():
                self.image.draw_robot(rbt.get_pos(), rbt.get_angle())
        for rbt in field.enemies:
            if rbt.is_used():
                self.image.draw_robot(rbt.get_pos(), rbt.get_angle(), (255, 255, 0))
        self.image.draw_dot(field.ball.get_pos(), 5)
        self.image.draw_poly(field.ally_goal.hull)
        self.image.draw_poly(field.enemy_goal.hull)
        self.image.update_window()
        self.image.draw_field()

        return waypoints

    def run(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """
        Определение глобального состояния игры
        """
        x = 0

    def go_to_ball(self, waypoints: list[wp.Waypoint], field: fld.Field, robot: aux.Point, ball: aux.Point) -> None:
        robot_r = const.ROBOT_R
        lines = [(-400.0, 400.0)]
        for i in range(3):
            now_rb = field.enemies[i]
            if now_rb.is_used():
                angle1 = math.atan2(ball.y - now_rb.get_pos().y + robot_r, now_rb.get_pos().x - ball.x)
                angle2 = math.atan2(ball.y - now_rb.get_pos().y - robot_r, now_rb.get_pos().x - ball.x)
                if abs(angle1) > math.pi / 2 and abs(angle2) > math.pi / 2:
                    continue
                now_ln = (ball.y - (4500 - ball.x) * math.tan(angle1), ball.y - (4500 - ball.x) * math.tan(angle2))
                if now_ln[0] > now_ln[1]:
                    now_ln = (now_ln[1], now_ln[0])
                # myPrint.myPrint(str(now_ln[0]) + " " + str(now_ln[1]))
                # myPrint.myPrint(i)
                j = 0
                while j < len(lines):
                    lines_flag = 0
                    if now_ln[0] > lines[j + lines_flag][0] and now_ln[0] < lines[j + lines_flag][1]:
                        lines.insert(j + lines_flag, (lines[j + lines_flag][0], now_ln[0]))
                        lines_flag += 1
                    if now_ln[1] > lines[j + lines_flag][0] and now_ln[1] < lines[j + lines_flag][1]:
                        lines.insert(j + lines_flag, (now_ln[1], lines[j + lines_flag][1]))
                        lines_flag += 1
                    if lines_flag != 0:
                        lines.pop(j + lines_flag)
                        j += lines_flag - 1
                    if now_ln[0] < lines[j][0] and now_ln[1] > lines[j][1] and lines_flag == 0:
                        lines.pop(j)
                        j -= 1
                    j += 1
        max_line = -1
        for i in range(len(lines)):
            if max_line != -1:
                if lines[max_line][1] - lines[max_line][0] < lines[i][1] - lines[i][0]:
                    max_line = i
            else:
                max_line = i
        if max_line == -1:
            grades = aux.Point(4500, 0)
        else:
            grades = aux.Point(4500, (lines[max_line][0] + lines[max_line][1]) / 2)
        # myPrint.myPrint(grades.y)
        alphaTr = math.atan2(ball.y - grades.y, grades.x - ball.x)
        point = aux.Point(ball.x, ball.y)
        point.x -= robot_r * math.cos(alphaTr)
        point.y += robot_r * math.sin(alphaTr)
        aTr = aux.dist(robot, point)
        bTr = aux.dist(robot, ball)
        cTr = aux.dist(ball, point)
        pTr = (aTr + bTr + cTr) / 2
        STr = math.sqrt(pTr * (pTr - aTr) * (pTr - bTr) * (pTr - cTr))
        hTr = 2 * STr / aTr
        gamma = math.acos((bTr ** 2 + cTr ** 2 - aTr ** 2) / (2 * bTr * cTr))
        ###print(str(ball.x) + " " + str(ball.y))
        ###print(str(self.x) + " " + str(self.y))
        self.rotate_to_point(ball)
        if abs(ball.y) > 3000 or abs(ball.x) > 4500:
            self.go_to_point(ball)
            return
        if bTr > robot_r * 2:
            self.withBall = 0
        if hTr > robot_r or aTr < bTr or gamma < math.pi / 4:
            beta = math.atan2(robot.y - ball.y, ball.x - robot.x)
            #if abs(math.sin(beta - alphaTr) * bTr) < 15 and gamma < 0.1 and abs(abs(-self.orientation - alphaTr) - abs(gamma)) < 0.1:
            if self.rotated_to_point(ball) == 1 and self.gone_to_point(point):
                self.withBall = 1
            if self.withBall:
                self.go_to_point(ball)
            else:
                self.go_to_point(point)
            ###print(str(gamma) + " " + str(math.sin(beta - alphaTr) * bTr) + " " + str(abs(-self.orientation - alphaTr)))
        else:
            ###print(1)
            bTr = max(robot_r, bTr)
            aPr = math.sqrt(bTr ** 2 - robot_r ** 2)
            alphaPr = math.atan2(robot_r, aPr)
            beta = math.atan2(ball.y - robot.y, robot.x - ball.x)
            way1 = aux.Point(robot.x - (aPr + robot_r) * math.cos(beta - alphaPr), robot.y + (aPr + robot_r) * math.sin(beta - alphaPr))
            way2 = aux.Point(robot.x - (aPr + robot_r) * math.cos(beta + alphaPr), robot.y + (aPr + robot_r) * math.sin(beta + alphaPr))
            ###print(str(auxiliary.dist(way1, point)) + " " + str(auxiliary.dist(way2, point)))
            if aux.dist(way1, point) < aux.dist(way2, point):
                self.go_to_point(way1)
                ###print(str(way1.x) + " " + str(way1.y))
                #pass
            else:
                self.go_to_point(way2)
                ###print(str(way2.x) + " " + str(way2.y))
                #pass
