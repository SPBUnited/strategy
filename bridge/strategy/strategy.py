"""Верхнеуровневый код стратегии"""

# pylint: disable=redefined-outer-name

# @package Strategy
# Расчет требуемых положений роботов исходя из ситуации на поле


import math

# !v DEBUG ONLY
from enum import Enum
from time import time
from typing import Optional

import numpy as np

import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, fld, rbt
from bridge.processors.referee_state_processor import Color as ActiveTeam
from bridge.processors.referee_state_processor import State as GameStates
from bridge.strategy import attack_roles, defense_roles, kicker, ref_states
import bridge.strategy.accessories as acc


class Strategy:
    """Основной класс с кодом стратегии"""

    def __init__(
            self,
            dbg_game_status: GameStates = GameStates.RUN,
    ) -> None:
        self.game_status = dbg_game_status
        self.active_team: ActiveTeam = ActiveTeam.ALL
        self.timer = time()

    def change_game_state(
            self, new_state: GameStates, upd_active_team: ActiveTeam
    ) -> None:
        """Изменение состояния игры и цвета команды"""
        self.game_status = new_state
        self.active_team = upd_active_team

    def process(self, field: fld.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """
        waypoints: list[wp.Waypoint] = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoints.append(
                wp.Waypoint(
                    field.allies[i].get_pos(),
                    field.allies[i].get_angle(),
                    wp.WType.S_STOP,
                )
            )

        '''
        wp.Waypoint(
                    POINT,
                    ANGLE,
                    TYPE,
                )
        POINT - куда робот едет
        ANGLE - под каким углом
        TYPE - что робот будет там делать
        Например, 
        wp.Waypoint(
                    aux.Point(0, 0),
                    0,
                    wp.WType.S_ENDPOINT
                )
                
        Робот поедет в центр поля под углом 0
        Или,
        wp.Waypoint(
                    field.ball.get_pos(),
                    0,
                    wp.WType.S_BALL_KICK
                )
        
        Робот пнет мяч(находящийся в точке field.ball.get_pos()) под углом 0.
        
        Вероятно вам может захотеться использовать какую-то готовую математику, вы можете найти ее в bridge/auxiliary/aux.py
        
        В частности оттуда вам может пригодится:
        aux.Point(x, y) - наше представление вектора
        Перегружены операторы сложения/вычитания
        .mag() возвращает длину вектора, .arg() - угол
        
        Также вы, вероятно, хотели бы получить положения вратаря. 
        В вашем случае его стоит получать, как:
        field.allies[const.GK].get_pos()
        
        Ваша задача на сегодня:
        
        1) Забить в пустые ворота
        2) Забить не в пустые ворота :)
        
        По любым вопросам обращайтесь к нам
        Удачи!
        '''
        # CODE START

        # CODE FINISH
        return waypoints
