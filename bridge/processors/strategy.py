"""Верхнеуровневый код стратегии"""
# pylint: disable=redefined-outer-name

# @package Strategy
# Расчет требуемых положений роботов исходя из ситуации на поле

import math

# !v DEBUG ONLY
import time
import typing
from enum import Enum

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as field  # pylint: disable = unused-import
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


class Strategy:

    def __init__(self, dbg_game_status = GameStates.RUN, dbg_state = States.DEBUG) -> None:
        self.game_status = GameStates.RUN
        self.active_team = 0
        self.status = States.ATTACK
        self.n = 3
        self.ballRadius = 110
        self.robotRadius = 200
        self.goalUp = 500
        self.goalDown = -500

    def process(self, field: field.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """
        waypoints = [None]*const.TEAM_ROBOTS_MAX_COUNT
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoints.append(wp.Waypoint(field.allies[i].get_pos(), field.allies[i].get_angle(), wp.WType.S_ENDPOINT))

        if self.game_status != GameStates.PENALTY:
            self.is_started = 0

        if self.game_status == GameStates.RUN:
            self.run(field, waypoints)
        else:
            if self.game_status == GameStates.TIMEOUT:
                self.timeout(field, waypoints)
            elif self.game_status == GameStates.HALT:
                pass
                # self.halt(field, waypoints)
            elif self.game_status == GameStates.PREPARE_PENALTY:
                self.prepare_penalty(field, waypoints)
            elif self.game_status == GameStates.PENALTY:
                self.penalty(field, waypoints)
            elif self.game_status == GameStates.BALL_PLACEMENT:
                self.keep_distance(field, waypoints)
            elif self.game_status == GameStates.PREPARE_KICKOFF:
                self.prepare_kickoff(field, waypoints)
            elif self.game_status == GameStates.KICKOFF:
                self.kickoff(field, waypoints)
            elif self.game_status == GameStates.FREE_KICK:
                self.free_kick(field, waypoints)
            elif self.game_status == GameStates.STOP:
                self.keep_distance(field, waypoints)

        # print(self.game_status, self.state)
        return waypoints


    def distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def getIndexHolding(self, field: field.Field): # Возвращает индекс атакующего робота (для врагов индекс + 3), None -- в случае неопределенного статуса
        minDistEnemy = 4500
        iEnemy = -1
        for i in range(self.n):
            d = self.distance(field.ball.getPos(), field.enemies[i].getPos()) 
            if d < minDistEnemy:
                minDistEnemy = d
                iEnemy = i

        minDistAllies = 4500
        iAllies = -1
        for i in range(self.n):
            d = self.distance(field.ball.getPos(), field.allies[i].getPos()) 
            if d < minDistAllies:
                minDistAllies = d
                iAllies = i

        if minDistEnemy < minDistAllies and minDistAllies - minDistEnemy > 30:
            return 3 + iEnemy
        elif minDistAllies < minDistEnemy and minDistEnemy - minDistAllies > 30:
            return iAllies
        else: 
            return None

    def run(self, field: field.Field, waypoints):
        # field.ball.getPos() - координаты мяча
        # field.enemies[i].getPos() - координаты робота соперника с id i
        # field.allies[i].getPos() - координаты робота союзника с id i
        # waypoints[i] = wp.Waypoint(field.allies[i].getPos(), field.allies[i].getAngle(), wp.WType.S_ENDPOINT) - задать точку для езды. Куда, с каким углом, тип.

        goal_pos = aux.Point(-6000, 0)
        enemy_pos = field.enemies[0].getPos()
        self_pos = field.allies[0].getPos()

        # print(enemy_pos.x, enemy_pos.y)
       
        # print(self_pos.x, self_pos.y, sep = ", ", end = "\n")
        # print(len(waypoints))

        alpha = math.atan2(enemy_pos.y - goal_pos.y, enemy_pos.x - goal_pos.x)
        beta = math.atan2(self_pos.y - goal_pos.y, self_pos.x - goal_pos.x) - alpha
        dist_to_goal = math.sqrt((self_pos.x - goal_pos.x) ** 2 + (self_pos.y - goal_pos.y ** 2))

        length = dist_to_goal * math.cos(beta)
        #path_len = dist_to_goal * math.sin(beta)
        #path_angle = beta + (math.pi / 2)
        path_point = aux.Point(goal_pos.x + length * math.cos(alpha), goal_pos.y + length * math.sin(alpha))
        
        target_point = aux.Point(path_point.x + 100 * math.cos(alpha + (math.pi / 2)), path_point.y + 100 * math.sin(alpha + (math.pi / 2)))

        # waypoints[0] = wp.Waypoint(aux.Point(0, 0), field.allies[0].getAngle(), wp.WType.S_ENDPOINT)
        waypoints[0] = wp.Waypoint(path_point, alpha, wp.WType.S_ENDPOINT)
        

        print(path_point.x, path_point.y, self_pos.x, sep = ", ", end = "\n")

        # for i in range(len(waypoints)):
        #     waypoints[i] = wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_ENDPOINT)
            

        #if True: # Здесь будет условие смены атаки на защиту для данного робота
        #    self.chooseKick(field, 0)
        #else:
        #    pass

        pass

    def chooseKick(self, field: field.Field, robotInx):
        central = []
        
        myPos = field.allies[robotInx].getPos()
        ballPos = field.ball.getPos() 

        for i in range(self.n):
            dist = self.distance(myPos, field.enemies[i].getPos())
            D = dist * (4500 - ballPos.x) / (field.enemies[i].getPos().x - ballPos.x)
            if (self.robotRadius + self.ballRadius + 20) / dist > 1: alphaNew = math.asin(1)
            else: alphaNew = math.asin((self.robotRadius + self.ballRadius + 20) / dist)
            
            gamma = math.acos((field.enemies[i].getPos()- ballPos.x) / dist) - alphaNew
            
            downDist = math.sqrt(D**2 - (4500 - ballPos.x)**2) - (4500 - ballPos.x) * math.tan(gamma) #HASHUV
            
            if abs(D * math.sin(alphaNew) / downDist) <= 1:  bettaNew = math.asin(D * math.sin(alphaNew) / downDist) 
            elif math.sin(alphaNew) > 0: bettaNew = math.asin(1)
            else: bettaNew = math.asin(-1)
            
            upDist = D * math.sin(alphaNew) / math.sin(math.pi - 2 * alphaNew - bettaNew) #HASHUV

            if ballPos.y > field.enemies[i]: 
                (downDist, upDist) = (upDist, downDist)
                ycc = ballPos.y - D * math.sin(alphaNew + gamma)
            else:
                ycc = ballPos.y + D * math.sin(alphaNew + gamma)


            if ycc < self.goalDown or ycc > self.goalUp:
                central.append([ycc, ycc + upDist, ycc - downDist])
        
        central = sorted(central, key=lambda x: x[0])

        maxiAngle = 0
        rememberI = -1
        for i in range(len(central) - 1):
            lookUp = central[i + 1][2]
            lookDown = central[i][1]

            bokDown = math.sqrt((myPos.x - 4500)**2 + (myPos.y - lookDown)**2)
            bokUp = math.sqrt((myPos.x - 4500)**2 + (myPos.y - lookUp)**2)
            v1 = (4500 - myPos.x, lookDown - myPos.y)
            v2 = (4500 - myPos.x, lookUp - myPos.y)
            
            if (v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp) > 1: angleBetweenVectors = math.acos(1)
            elif (v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp) < -1: angleBetweenVectors = math.acos(-1)
            else: angleBetweenVectors = math.acos((v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp))

            if angleBetweenVectors > maxiAngle:
                maxiAngle = angleBetweenVectors
                rememberI = i
            
        if rememberI != -1:
            lookUp = central[rememberI + 1][2]
            lookDown = central[rememberI][1]
            bokDown = math.sqrt((myPos.x - 4500)**2 + (myPos.y - lookDown)**2)
            bokUp = math.sqrt((myPos.x - 4500)**2 + (myPos.y - lookUp)**2)
            
            osn = lookUp - lookDown
            distUp = osn * bokUp / (bokUp + bokDown)
            
            self.xR = 4500
            self.yR = lookUp - distUp
        else:
            self.xR = 4500
            self.yR = 0
        #pass
