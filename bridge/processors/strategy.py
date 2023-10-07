## @package Stratery
# 
# Расчет требуемых положений роботов исходя из ситуации на поле

import bridge.processors.field as field
import bridge.processors.waypoint as wp
import bridge.processors.const as const
import bridge.processors.auxiliary as aux
import bridge.processors.signal as signal
import math
from enum import Enum

#!v DEBUG ONLY
import time

popusk_positions = [aux.Point(0, 0), aux.Point(0, 2000), aux.Point(0, -2000), aux.Point(2000, 2000), aux.Point(2000, -2000)]
used_pop_pos = [False, False, False, False, False]

class States(Enum):
    DEBUG = 0
    DEFENCE = 1
    ATTACK = 2

class GameStates(Enum):
    HALT = 0
    STOP = 1
    RUN = 2
    TIMEOUT = 3
    PREPARE_KICKOFF = 5
    KICKOFF = 6
    PREPARE_PENALTY = 7
    PENALTY = 8
    FREE_KICK = 9
    BALL_PLACMENT = 11

class ActiveTeam(Enum):
    ALL = 0
    YELLOW = 1
    BLUE = 2

class Strategy:
    def __init__(self, dbg_game_status = GameStates.RUN, dbg_state = States.DEBUG) -> None:
        self.game_status = dbg_game_status
        self.active_team = 0
        self.state = dbg_state
        self.weActive = 1

        #DEFENCE
        self.old_def_helper = -1
        self.old_def = -1
        self.steal_flag = 0

        #ATTACK
        self.robot_with_ball = 3
        self.connector = []
        self.popusk = [4, 1, 2]
        self.attack_state = "TO_BALL"
        self.attack_pos = aux.Point(0,0)

    def changeGameState(self, newState, activeTeam):
        self.game_status = newState
        if activeTeam == 0:
            self.active_team = ActiveTeam.ALL
        elif activeTeam == 2:
            self.active_team = ActiveTeam.YELLOW
        elif activeTeam == 1:
            self.active_team = ActiveTeam.BLUE

    """
    Рассчитать конечные точки для каждого робота
    """
    def process(self, field: field.Field):
        if field.ally_color == 'b':
            if self.active_team == ActiveTeam.BLUE:
                self.weActive = 1
            else:
                self.weActive = 0
        else:
            if self.active_team == ActiveTeam.YELLOW:
                self.weActive = 1
            else:
                self.weActive = 0
        if self.active_team == ActiveTeam.ALL:
            self.weActive = 1

        print(self.weActive)
        waypoints = [None]*const.TEAM_ROBOTS_MAX_COUNT
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoint = wp.Waypoint(field.allies[i].getPos(), field.allies[i].getAngle(), wp.WType.ENDPOINT)
            waypoints[i] = waypoint

        if self.game_status == GameStates.RUN:
            '''if field.ball.vel.mag() < 100:
                rivals_robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
                allies_robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.allies)

                if aux.dist(rivals_robot_with_ball.getPos(), field.ball.getPos()) < aux.dist(allies_robot_with_ball.getPos(), field.ball.getPos()):
                    self.state = States.DEFENCE
                else:
                    self.state = States.ATTACK'''


            if self.state == States.DEBUG:
                self.debug(field, waypoints)
            elif self.state == States.DEFENCE:
                self.defence(field, waypoints)
            elif self.state == States.ATTACK:
                self.preAttack(field)
                self.attack(field, waypoints)
            
            if self.state != States.DEFENCE:
                self.old_def_helper = -1
                self.old_def = -1
                self.steal_flag = 0

            robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
            self.gk_go(field, waypoints, [const.GK], robot_with_ball)
        else:
            if self.game_status == GameStates.TIMEOUT:
                rC = 0
                for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                    if field.allies[i].is_used():
                        waypoint = wp.Waypoint(aux.Point(2000 * field.side, 1250 - 500 * rC), field.allies[i].getAngle(), wp.WType.ENDPOINT)
                        waypoints[i] = waypoint
                        rC+=1
            elif self.game_status == GameStates.HALT:
                for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                    if field.allies[i].is_used():
                        waypoint = wp.Waypoint(field.allies[i].getPos(), field.allies[i].getAngle(), wp.WType.STOP)
                        waypoints[i] = waypoint
            elif self.game_status == GameStates.PREPARE_PENALTY:
                for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                    if field.allies[i].is_used():
                        waypoint = wp.Waypoint(field.allies[i].getPos(), field.allies[i].getAngle(), wp.WType.STOP)
                        waypoints[i] = waypoint

        return waypoints


    square = signal.Signal(8, 'SQUARE', lohi=(-1000, 1000))
    square_ang = signal.Signal(8, 'SQUARE', lohi=(0, 3.14))
    def debug(self, field: field.Field, waypoints):
        for i in range(0, 6):
            # pos = aux.point_on_line(bbotpos, -aux.Point(const.GOAL_DX, 0), 300)
            pos = -field.ally_goal.eye_forw*500*i -field.ally_goal.eye_up*3000
            # pos = aux.Point(1000 + self.square.get(), -1000)
            
            # dpos = bbotpos - ybotpos
            # angle = math.atan2(dpos.y, dpos.x)
            # angle = self.square_ang.get()
            angle = math.pi/3 * 0

            waypoint = wp.Waypoint(pos, angle, wp.WType.ENDPOINT)
            waypoints[i] = waypoint
        
        waypoints[1].pos = field.ally_goal.eye_forw*1000 + field.ally_goal.eye_up * (self.square.get() - 1000)

        waypoints[2] = wp.Waypoint(field.ball.getPos(), (field.enemy_goal.center - field.ball.getPos()).arg(), wp.WType.KICK_IMMEDIATE)

        return waypoints

    def gk_go(self, field: field.Field, waypoints, gk_wall_idx_list, robot_with_ball):
        """
        Расчет требуемых положений вратаря и стенки
        
        [in] field - объект Field(), ситуация на поле
        [out] waypoints - ссылка на массив путевых точек, который будет изменен!
        [in] gk_wall_idx_list - массив индексов вратаря и стенки. Нулевой элемент всегда индекс вратаря
        Дальше индексы роботов в стенке, кол-во от 0 до 3
        [in] robot_with_ball - текущий робот с мячом
        """
        try:
            gk_pos = aux.LERP(aux.point_on_line(field.ally_goal.center, field.ball.pos, const.GK_FORW),
                          aux.get_line_intersection(robot_with_ball.pos, robot_with_ball.pos + aux.rotate(aux.Point(1, 0), robot_with_ball.angle),
                                                    field.ally_goal.down, field.ally_goal.up, 'RS') + const.GK_FORW*field.ally_goal.eye_forw,
                          0.5)
            # print(robot_with_ball.angle)
        except:
            gk_pos = aux.point_on_line(field.ally_goal.center, field.ball.pos, const.GK_FORW)

        # print(field.ball.vel.mag())
        if field.ball.vel.mag() > 100 and \
            aux.get_line_intersection(field.ally_goal.down,
                                      field.ally_goal.up,
                                      field.ball.getPos(),
                                      field.ball.getPos() + field.ball.getVel(),
                                      'SR'
                                      ) is not None:
            gk_pos = aux.closest_point_on_line(field.ball.pos, field.ball.vel.unity()*1000000, field.allies[gk_wall_idx_list[0]].pos)
            # print("GK INTERCEPT", time.time())

        gk_angle = math.pi/2
        waypoints[gk_wall_idx_list[0]] = wp.Waypoint(gk_pos, gk_angle, wp.WType.ENDPOINT)

        # print(field.isBallInGoalSq())
        if field.isBallInGoalSq() and field.ball.vel.mag() < 200:
            waypoints[gk_wall_idx_list[0]] = wp.Waypoint(field.ball.pos, field.ally_goal.eye_forw.arg(), wp.WType.KICK_IMMEDIATE)

    def defence(self, field: field.Field, waypoints):
        dist_between = 200

        worksRobots = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                field.allies[i].dribblerEnable = 0
                field.allies[i].autoKick = 0
                worksRobots.append(field.allies[i])
        totalRobots = len(worksRobots)

        used_robots_id = []
        if field.allies[const.GK].is_used():
            used_robots_id = [const.GK]
        robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
        

        def1 = aux.find_nearest_robot(robot_with_ball.getPos(), field.allies, used_robots_id)

        # FOR BALL STEALING
        '''if def1.rId == self.old_def_helper:
            def1 = allies[self.old_def]'''

        targetPoint = aux.point_on_line(robot_with_ball.getPos(), aux.Point(field.side * const.GOAL_DX, 0), dist_between)
        waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, robot_with_ball.getPos()), wp.WType.ENDPOINT)
        waypoints[def1.rId] = waypoint
        self.old_def = def1.rId


        used_robots_id.append(def1.rId)
        rbs = sorted(field.enemies, reverse=True, key=lambda x: x.getPos().x)
        rbs_rIds = []
        xAttack = 3500

        for r in rbs:
            if ((field.side * r.getPos().x > (field.side * robot_with_ball.getPos().x - 150)) or (field.side * r.getPos().x > 0)) and r.rId != robot_with_ball.rId and r.is_used() and r.rId != const.ENEMY_GK:  
                rbs_rIds.append(r.rId)

        for i in range(len(rbs_rIds)):
            defender = aux.find_nearest_robot(field.enemies[rbs_rIds[i]].getPos(), field.allies, used_robots_id)
            used_robots_id.append(defender.rId)

            targetPoint = aux.point_on_line(field.enemies[rbs_rIds[i]].getPos(), field.ball.getPos(), dist_between)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
            waypoints[defender.rId] = waypoint

        # FOR BALL STEALING
        '''if totalRobots - len(used_robots_id) > 0:
            def1Helper = aux.find_nearest_robot(robot_with_ball.getPos(), allies, used_robots_id)
            #targetPoint = aux.point_on_line(field.ball.getPos(), robot_with_ball.getPos(), dist_between)
            targetPoint = aux.point_on_line(field.ball.getPos(), robot_with_ball.getPos(), -dist_between - 200)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(def1Helper.getPos(), field.ball.getPos()), wp.WType.ENDPOINT)
            used_robots_id.append(def1Helper.rId)
            self.old_def_helper = def1Helper.rId
            if aux.dist(def1Helper.getPos(), targetPoint) > 500:
                self.steal_flag = 0
            if aux.dist(def1Helper.getPos(), targetPoint) < 50 or self.steal_flag:
                self.steal_flag = 1
                def1Helper.dribblerEnable = 1
                def1Helper.speedDribbler = 15
                if aux.dist(def1Helper.getPos(), field.ball.getPos()) < 130: # IF BALL IS CATCHED
                    waypoint = wp.Waypoint(aux.Point(0, 0), aux.angle_to_point(def1Helper.getPos(), field.ball.getPos()), wp.WType.IGNOREOBSTACLES)   
                else:
                    waypoint = wp.Waypoint(field.ball.getPos(), aux.angle_to_point(def1Helper.getPos(), field.ball.getPos()), wp.WType.IGNOREOBSTACLES)   
            waypoints[def1Helper.rId] = waypoint
            if aux.dist(def1Helper.getPos(), targetPoint) < 1500:
                targetPoint = aux.point_on_line(robot_with_ball.getPos(), aux.Point(field.side * const.GOAL_X, 0), dist_between + 500)
                waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, robot_with_ball.getPos()), wp.WType.ENDPOINT)
                waypoints[def1.rId] = waypoint'''



        if totalRobots - len(used_robots_id) == 1:
             for r in worksRobots:
                i = r.rId
                if i not in used_robots_id:
                    targetPoint = aux.Point(field.side * -xAttack, 1500)
                    waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
                    waypoints[i] = waypoint
                    used_robots_id.append(i)
        elif totalRobots - len(used_robots_id) == 2:
            def2 = aux.find_nearest_robot(aux.Point(field.side * -xAttack, 1500), field.allies, used_robots_id)
            used_robots_id.append(def2.rId)
            targetPoint = aux.Point(field.side * -xAttack, 1500)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
            waypoints[def2.rId] = waypoint


            for r in worksRobots:
                i = r.rId
                if field.allies[i].rId not in used_robots_id:
                    used_robots_id.append(field.allies[i].rId)
                    targetPoint = aux.Point(field.side * -xAttack, -1500)
                    waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
                    waypoints[field.allies[i].rId] = waypoint

        else:
            def2 = aux.find_nearest_robot(aux.Point(field.side * -xAttack, 1500), field.allies, used_robots_id)
            used_robots_id.append(def2.rId)
            targetPoint = aux.Point(field.side * -xAttack, 1500)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
            waypoints[def2.rId] = waypoint
            
            def2 = aux.find_nearest_robot(aux.Point(field.side * -xAttack, -1500), field.allies, used_robots_id)
            used_robots_id.append(def2.rId)
            targetPoint = aux.Point(field.side * -xAttack, -1500)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), wp.WType.ENDPOINT)
            waypoints[def2.rId] = waypoint

            # Some math for wall. Should be remade with gk.
            k = totalRobots - len(used_robots_id)
            c = 0
            for r in worksRobots:
                i = r.rId
                if i not in used_robots_id:
                    if abs(field.ball.getPos().y) > 800:
                        bY =  800 * aux.sign(field.ball.getPos().y)
                        if abs(field.ball.getPos().x) < 3500:
                            bX = 3200
                        else:
                            bX = field.ball.getPos().x                            
                    else:
                        bY = field.ball.getPos().y
                        bX = 3400
                    if k % 2 == 0:
                        if bX == field.ball.getPos().x:
                            used_robots_id.append(field.allies[i].rId)
                            targetPoint = aux.Point(field.side * bX - 200 * k / 2 + 200 * c, 1200 * aux.sign(field.ball.getPos().y))
                            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(field.allies[i].getPos(), aux.Point(field.allies[i].getPos().x, 5000 * aux.sign(field.ball.getPos().y))), wp.WType.ENDPOINT)
                            waypoints[field.allies[i].rId] = waypoint                       
                        else:
                            used_robots_id.append(field.allies[i].rId)
                            targetPoint = aux.Point(field.side * bX, (bY - 200 * k / 2 + 200 * c) * field.side)
                            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(field.allies[i].getPos(), aux.Point(0, field.allies[i].getPos().y)), wp.WType.ENDPOINT)
                            waypoints[field.allies[i].rId] = waypoint                          
                    else:
                        if bX == field.ball.getPos().x:
                            used_robots_id.append(field.allies[i].rId)
                            targetPoint = aux.Point(field.side * bX - 200 * (k - 1) + 200 * c, 1200 * aux.sign(field.ball.getPos().y))
                            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(field.allies[i].getPos(), aux.Point(field.allies[i].getPos().x, 5000 * aux.sign(field.ball.getPos().y))), wp.WType.ENDPOINT)
                            waypoints[field.allies[i].rId] = waypoint          
                        else:
                            used_robots_id.append(field.allies[i].rId)
                            targetPoint = aux.Point(field.side * bX, (bY - 200 * (k - 1) + 200 * c) * field.side)
                            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(field.allies[i].getPos(), aux.Point(0, field.allies[i].getPos().y)), wp.WType.ENDPOINT)
                            waypoints[field.allies[i].rId] = waypoint  
                    c += 1

    def preAttack(self, field: field.Field):
            allies = field.allies
            used_robots_id = []
            mn = 1e10
            role_with_ball = 0
            for robot in allies:
                ball_dist = aux.dist(field.ball.pos, robot.getPos())
                if mn > ball_dist:
                    mn = ball_dist
                    self.robot_with_ball = robot.rId
            mn = 1e10
            for pointIndex in range(len(popusk_positions)):
                pop_pos_dist = aux.dist(allies[self.robot_with_ball].getPos(), popusk_positions[pointIndex])
                if mn > pop_pos_dist:
                    mn = pop_pos_dist
                    allies[self.robot_with_ball].role = pointIndex
            used_pop_pos[allies[self.robot_with_ball].role] = True
            for pointIndex in range(len(popusk_positions)):
                if used_pop_pos[pointIndex] == False:
                    mn = 1e10
                    save_robot = 0
                    for robot in allies:
                        pop_pos_dist = aux.dist(robot.getPos(), popusk_positions[pointIndex])
                        if mn > pop_pos_dist:
                            mn = pop_pos_dist
                            save_robot = robot.rId
                    allies[save_robot].role = pointIndex
                    used_pop_pos[pointIndex] = True
            # for role in AttackRoles:
            #     point = AttackPoints[role]
            #     robot = aux.find_nearest_robot(point, allies, used_robots_id)
            #     robot.is_used()
            #     role.id = robot
        

    def attack(self, field: field.Field, waypoints):
        for i in range(6):
            waypoint = wp.Waypoint(field.allies[i].getPos(), field.allies[i].getAngle(), wp.WType.ENDPOINT)
            waypoints[i] = waypoint

        goal_points = [aux.Point(const.GOAL_DX * -field.side, i) for i in range(-500, 501)]
        bro_points = [field.allies[i].getPos() for i in range(len(field.allies))]
        # print(bro_points)
        if self.robot_with_ball != None:
            if self.attack_state == "TO_BALL":
                self.attack_pos = field.ball.pos
                if aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].getPos(), 500):
                    self.attack_state = "CALCULATING"
            elif self.attack_state == "CALCULATING":
                shot_pos, shot_prob = aux.shotDecision(field.ball.pos, goal_points, field.enemies[:2])
                # shot_pos = aux.Point(field.ball.getPos().x - 1000 * const.ROBOT_R, field.ball.getPos().y)
                # if shot_prob > const.KOEFF_NAGLO:
                # print(shot_pos)
                self.attack_pos = shot_pos
                # print(shot_pos)
                if aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].getPos(), 50):
                    self.attack_state = "GO_TO_SHOOTING_POSITION"
                # else:
                #     pass_pos, pass_prob = aux.shotDecision(field.ball.getPos(), bro_points, rivals)
                #     attack_pos = pass_pos
            elif self.attack_state == "GO_TO_SHOOTING_POSITION":
                # shot_pos = aux.Point(field.ball.getPos().x - 1000 * const.ROBOT_R, field.ball.getPos().y)
                # if shot_prob > const.KOEFF_NAGLO:
                # self.attack_pos = shot_pos
                field.allies[self.robot_with_ball].kickerChargeEnable = 1
                field.allies[self.robot_with_ball].kickerVoltage = 15
                if aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].getPos(), 50):
                    self.attack_state = "SHOOT"
            elif self.attack_state == "SHOOT":
                self.attack_pos = field.ball.pos
                field.allies[self.robot_with_ball].kickerChargeEnable = 1
                field.allies[self.robot_with_ball].kickerVoltage = 15
                field.allies[self.robot_with_ball].kick_up()
                if not(aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].getPos(), 500)):
                    self.attack_state = "TO_BALL"
                #ехать к нужной точке
            if self.attack_state == "SHOOT":
                waypoints[self.robot_with_ball] = wp.Waypoint(self.attack_pos, aux.angle_to_point(field.allies[self.robot_with_ball].getPos(), field.ball.getPos()), wp.WType.IGNOREOBSTACLES)
            else:
                waypoints[self.robot_with_ball] = wp.Waypoint(self.attack_pos, aux.angle_to_point(field.allies[self.robot_with_ball].getPos(), field.ball.getPos()), wp.WType.ENDPOINT)
            print(self.attack_state)

        # if connector != None:
        #     ball_line = aux.getBallLine()
        #     pos = aux.find_nearest_point_on_the_line(ball_line, connector.pos)
        #     if not aux.in_place(pos, connector.pos, 10):

        #     else:
                
            
        #     connector =
        for robot in self.popusk:
            pop_pos = popusk_positions[robot]
            waypoints[robot] = wp.Waypoint(pop_pos, 0, wp.WType.ENDPOINT)