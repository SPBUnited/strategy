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
calc = False

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
        self.game_status = GameStates.RUN
        self.active_team = 0
        self.state = States.ATTACK
        self.weActive = 1
        self.timer = 0
        self.is_ball_moved = 0

        #DEFENCE
        self.old_def_helper = -1
        self.old_def = -1
        self.steal_flag = 0

        #ATTACK
        self.robot_with_ball = None
        self.connector = []
        self.popusk = []
        self.attack_state = "TO_BALL"
        self.attack_pos = aux.Point(0,0)
        self.calc = False

        #PENALTY
        self.we_kick = 0
        self.is_started = 0
        self.ball_inside = 0
        self.last_seen = 0
        self.ball_history = 0
        self.ticks_without_ball = 0
        self.penalty_timer = 0


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
        waypoints = [None]*const.TEAM_ROBOTS_MAX_COUNT
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoint = wp.Waypoint(field.allies[i].getPos(), field.allies[i].getAngle(), wp.WType.S_ENDPOINT)
            waypoints[i] = waypoint

        if self.game_status != GameStates.PENALTY:
            self.is_started = 0

        if self.game_status == GameStates.RUN:
            self.run(field, waypoints)
        else:
            if self.game_status == GameStates.TIMEOUT:
                self.timeout(field, waypoints)
            elif self.game_status == GameStates.HALT:
                pass
                #self.halt(field, waypoints)
            elif self.game_status == GameStates.PREPARE_PENALTY:
                self.prepare_penalty(field, waypoints)
            elif self.game_status == GameStates.PENALTY:
                self.penalty(field, waypoints) 
            elif self.game_status == GameStates.BALL_PLACMENT:
                self.keep_distance(field, waypoints)
            elif self.game_status == GameStates.PREPARE_KICKOFF:
                self.prepare_kickof(field, waypoints)
            elif self.game_status == GameStates.KICKOFF:
                self.kickoff(field, waypoints)
            elif self.game_status == GameStates.FREE_KICK:
                self.free_kick(field, waypoints)
            elif self.game_status == GameStates.STOP:
                self.keep_distance(field, waypoints)

        return waypoints


    def run(self, field: field.Field, waypoints):
        if field.ball.getVel().mag() < 100:
            if self.is_ball_moved or time.time() - self.timer > 7:
                rivals_robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies, [const.ENEMY_GK])
                allies_robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.allies, [const.GK])
                self.timer = time.time()
                if aux.dist(rivals_robot_with_ball.getPos(), field.ball.getPos()) < aux.dist(allies_robot_with_ball.getPos(), field.ball.getPos()):
                    self.state = States.DEFENCE
                    print("defence")
                else:
                    self.state = States.ATTACK
                    print("attack")
            self.is_ball_moved = 0
            if abs(const.GOAL_DX - abs(field.ball.getPos().x)) < const.GOAL_DY and abs(field.ball.getPos().y) < const.GOAL_DY:
                if abs(field.ball.getPos().x - field.enemy_goal.center.x) < abs(field.ball.getPos().x - field.ally_goal.center.x): 
                    self.state = States.DEFENCE
                else:
                    self.state = States.ATTACK
        else:
            self.is_ball_moved = 1
        wall = []
<<<<<<< HEAD
        
=======

        self.state = States.DEBUG

>>>>>>> a543cef (pisd regulator)
        if self.state == States.DEBUG:
            self.debug(field, waypoints)
        elif self.state == States.DEFENCE:
            wall = self.defence(field, waypoints)
        elif self.state == States.ATTACK:
            self.decide_popusk_position(field)
                # if not self.calc:
            self.preAttack(field)
                #     if len(self.popusk) != 0:
                #         self.calc = True
            self.attack(field, waypoints)
        
        if self.state != States.DEFENCE:
            self.old_def_helper = -1
            self.old_def = -1
            self.steal_flag = 0

        robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
        self.gk_go(field, waypoints, [const.GK] + wall, robot_with_ball)

    square = signal.Signal(8, 'SQUARE', ampoffset=(400, 0))
    square_ang = signal.Signal(8, 'SQUARE', lohi=(0, 3.14))
    def debug(self, field: field.Field, waypoints):

        # waypoints[const.DEBUG_ID].pos = aux.Point(0, 0)        
        # waypoints[const.DEBUG_ID].angle = 3.14
        # waypoints[const.DEBUG_ID].type = wp.WType.S_ENDPOINT        
        
        # robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
        self.gk_go(field, waypoints, [const.GK], None)


        # for i in range(0, 6):
        #     # pos = aux.point_on_line(bbotpos, -aux.Point(const.GOAL_DX, 0), 300)
        #     pos = -field.ally_goal.eye_forw*500*i -field.ally_goal.eye_up*3000
        #     # pos = aux.Point(1000 + self.square.get(), -1000)
            
        #     # dpos = bbotpos - ybotpos
        #     # angle = math.atan2(dpos.y, dpos.x)
        #     # angle = self.square_ang.get()
        #     angle = math.pi/3 * 0

        #     waypoint = wp.Waypoint(pos, angle, wp.WType.S_ENDPOINT)
        #     waypoints[i] = waypoint
        
        # waypoints[1].pos = field.ally_goal.eye_forw*4000 + field.ally_goal.eye_up * (self.square.get() - 2000)


        # egf = field.enemy_goal.forwdown + field.enemy_goal.eye_forw*300
        # if not field.allies[2].is_ball_in(field):
        #     waypoints[2] = wp.Waypoint(field.ball.getPos(), (field.enemy_goal.center - field.ball.getPos()).arg(), wp.WType.S_BALL_GRAB)
        # elif (egf - field.allies[2].getPos()).mag() > 300:
        #     waypoints[2] = wp.Waypoint(egf, math.pi/3, wp.WType.S_BALL_GO)
        # else:
        #     waypoints[2] = wp.Waypoint(egf, math.pi/3, wp.WType.S_BALL_KICK)

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
            gk_pos = aux.LERP(aux.point_on_line(field.ally_goal.center, field.ball.getPos(), const.GK_FORW),
                          aux.get_line_intersection(robot_with_ball.getPos(), robot_with_ball.getPos() + aux.rotate(aux.Point(1, 0), robot_with_ball.getAngle()),
                                                    field.ally_goal.down, field.ally_goal.up, 'RS') + const.GK_FORW*field.ally_goal.eye_forw,
                          0.5)
            # print(robot_with_ball.angle)
        except:
            gk_pos = aux.point_on_line(field.ally_goal.center, field.ball.getPos(), const.GK_FORW)

        # print(field.ball.vel.mag())
        if field.ball._vel.mag() > const.GK_INTERCEPT_SPEED and \
            aux.get_line_intersection(field.ally_goal.down,
                                      field.ally_goal.up,
                                      field.ball.getPos(),
                                      field.ball.getPos() + field.ball.getVel(),
                                      'SR'
                                      ) is not None:
            gk_pos = aux.closest_point_on_line(field.ball.getPos(), field.ball._vel.unity()*1000000, field.allies[gk_wall_idx_list[0]].getPos())
            # print("GK INTERCEPT", time.time())

        gk_angle = math.pi/2
        waypoints[gk_wall_idx_list[0]] = wp.Waypoint(gk_pos, gk_angle, wp.WType.S_ENDPOINT)

        # print(field.isBallInGoalSq(), field.ball.getPos())
        if field.isBallInGoalSq() and field.ball._vel.mag() < const.GK_PEN_KICKOUT_SPEED:
            waypoints[gk_wall_idx_list[0]] = wp.Waypoint(field.ball.getPos(), field.ally_goal.eye_forw.arg(), wp.WType.S_BALL_KICK)

        # wallline = [field.ally_goal.forw + field.ally_goal.eye_forw * const.GOAL_WALLLINE_OFFSET]
        # wallline.append(wallline[0] + field.ally_goal.eye_up)

        walline = aux.point_on_line(field.ally_goal.center, field.ball.getPos(), const.GOAL_WALLLINE_OFFSET)
        walldir = aux.rotate((field.ally_goal.center - field.ball.getPos()).unity(), math.pi/2)
        dirsign = -aux.sign(aux.vect_mult(field.ally_goal.center, field.ball.getPos()))

        wall = []
        for i in range(len(gk_wall_idx_list)-1):
            wall.append(walline - walldir * (i + 1) * dirsign * (1 + (i % 2)*-2) * const.GOAL_WALL_ROBOT_SEPARATION)
            waypoints[gk_wall_idx_list[i + 1]] = wp.Waypoint(wall[i], walldir.arg(), wp.WType.S_ENDPOINT)

    def defence(self, field: field.Field, waypoints, ENDPOINT_TYPE = wp.WType.S_ENDPOINT):
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
        

        def1 = aux.find_nearest_robot(field.ball.getPos(), field.allies, used_robots_id)

        # FOR BALL STEALING
        '''if def1.rId == self.old_def_helper:
            def1 = allies[self.old_def]'''
        targetPoint = aux.point_on_line(field.ball.getPos(), aux.Point(field.side * const.GOAL_DX, 0), dist_between)
        if ENDPOINT_TYPE == wp.WType.S_KEEP_BALL_DISTANCE:
            if aux.dist(targetPoint, field.ball.getPos()) < const.KEEP_BALL_DIST:
                targetPoint = aux.point_on_line(aux.point_on_line(targetPoint, field.ball.getPos(), -const.KEEP_BALL_DIST), aux.Point(field.side * const.GOAL_DX, 0), dist_between)
        waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), ENDPOINT_TYPE)
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
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), ENDPOINT_TYPE)
            waypoints[defender.rId] = waypoint

        # FOR BALL STEALING
        '''if totalRobots - len(used_robots_id) > 0:
            def1Helper = aux.find_nearest_robot(robot_with_ball.getPos(), allies, used_robots_id)
            #targetPoint = aux.point_on_line(field.ball.getPos(), robot_with_ball.getPos(), dist_between)
            targetPoint = aux.point_on_line(field.ball.getPos(), robot_with_ball.getPos(), -dist_between - 200)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(def1Helper.getPos(), field.ball.getPos()), wp.WType.S_ENDPOINT)
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
                waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, robot_with_ball.getPos()), wp.WType.S_ENDPOINT)
                waypoints[def1.rId] = waypoint'''

        wall_bots = []
        for _ in range(3):
            if totalRobots - len(used_robots_id) == 0:
                break
            wall_bot = aux.find_nearest_robot(field.ally_goal.center, field.allies, used_robots_id)
            wall_bots.append(wall_bot.rId)
            used_robots_id.append(wall_bot.rId)

        if totalRobots - len(used_robots_id) == 1:
             for r in worksRobots:
                i = r.rId
                if i not in used_robots_id:
                    targetPoint = aux.Point(field.side * -xAttack, 1500)
                    waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), ENDPOINT_TYPE)
                    waypoints[i] = waypoint
                    used_robots_id.append(i)
        elif totalRobots - len(used_robots_id) >= 2:
            def2 = aux.find_nearest_robot(aux.Point(field.side * -xAttack, 1500), field.allies, used_robots_id)
            used_robots_id.append(def2.rId)
            targetPoint = aux.Point(field.side * -xAttack, 1500)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), ENDPOINT_TYPE)
            waypoints[def2.rId] = waypoint


            for r in worksRobots:
                i = r.rId
                if field.allies[i].rId not in used_robots_id:
                    used_robots_id.append(field.allies[i].rId)
                    targetPoint = aux.Point(field.side * -xAttack, -1500)
                    waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), ENDPOINT_TYPE)
                    waypoints[field.allies[i].rId] = waypoint
                    break

        for r in worksRobots:
            i = r.rId
            if field.allies[i].rId not in used_robots_id:
                wall_bots.append(field.allies[i].rId)
                used_robots_id.append(field.allies[i].rId)
        return sorted(wall_bots)

    def decide_popusk_position(self, field: field.Field):
        for pointIndex in range(len(popusk_positions)):
            save_robot = -1
            if used_pop_pos[pointIndex] == False:
                mn = 1e10
                for robot in field.allies:
                    if robot.is_used() and robot.rId != const.GK and robot.rId != self.robot_with_ball and not aux.is_in_list(self.popusk, robot.rId):
                        pop_pos_dist = aux.dist(robot.getPos(), popusk_positions[pointIndex])
                        if mn > pop_pos_dist:
                            mn = pop_pos_dist
                            save_robot = robot.rId
                    elif not robot.is_used() and field.allies[robot.rId].role != None:
                        used_pop_pos[robot.role] = False
                        field.allies[robot.rId].role = None
                        self.popusk.pop(self.popusk.index(robot.rId))
                    # print([robot.rId, field.allies[robot.rId].role])
            if save_robot != -1:
                field.allies[save_robot].role = pointIndex
                self.popusk.append(save_robot)
                used_pop_pos[pointIndex] = True
                
        # print(used_pop_pos)
        # print(self.popusk)

    def preAttack(self, field: field.Field):
            # used_pop_pos = [False, False, False, False, False]
            # self.popusk = []
        if self.robot_with_ball == None and field.ball.getVel().mag() < 800:
            mn = 1e10
            for robot in field.allies:
                if robot.is_used() and robot.rId != const.GK:
                    ball_dist = aux.dist(field.ball.getPos(), robot.getPos())
                    if mn > ball_dist:
                        mn = ball_dist
                        self.robot_with_ball = robot.rId
            if aux.is_in_list(self.popusk, self.robot_with_ball):
                self.popusk.pop(self.popusk.index(self.robot_with_ball))
            # mn = 1e10
            # for pointIndex in range(len(popusk_positions)):
            #     pop_pos_dist = aux.dist(field.allies[self.robot_with_ball].getPos(), popusk_positions[pointIndex])
            #     if mn > pop_pos_dist:
            #         mn = pop_pos_dist
            #         field.allies[self.robot_with_ball].role = pointIndex
            # used_pop_pos[field.allies[self.robot_with_ball].role] = True
            # print(len(field.allies))
            # print(used_pop_pos)
            
                    
        
            # print(self.popusk)
            
            # for role in AttackRoles:
            #     point = AttackPoints[role]
            #     robot = aux.find_nearest_robot(point, allies, used_robots_id)
            #     robot.is_used()
            #     role.id = robot
        
    def attack(self, field: field.Field, waypoints):
        goal_points = [aux.Point(field.enemy_goal.center.x, i) for i in range(-400, 401)]
        bro_points = [field.allies[i].getPos() for i in range(len(field.allies))]
        # print(bro_points)
        if self.robot_with_ball != None:
            if self.attack_state == "TO_BALL":
                self.attack_pos = field.ball.getPos()
                if aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].getPos(), 500):
                    self.attack_state = "CALCULATING"
            elif self.attack_state == "CALCULATING":
                shot_pos, shot_prob = aux.shotDecision(field.ball.getPos(), goal_points, field.enemies)
                # shot_pos = aux.Point(field.ball.getPos().x - 1000 * const.ROBOT_R, field.ball.getPos().y)
                # if shot_prob > const.KOEFF_NAGLO:
                #     used_bots = []
                #     for bot in field.allies:
                #         if bot.used():
                #             used_bots.append(bot)

                #     attack_graph = aux.Graph(len(used_bots))
                #     for bot in used_bots:
                #         attack_graph.add_edge(self.robot_with_ball, bot.rId, aux.dist(field.allies[self.robot_with_ball].getPos(), field.allies[bot.rId].getPos()))
                #         bot_pass_pos, bot_pass_prob = aux.shotDecision(bot.getPos(), goal_points, field.enemies[:2])
                #         attack_graph.add_edge(bot.rId, const.GOAL_ID, bot_pass_prob)
                # print(shot_pos)
                self.attack_pos = shot_pos
                # print(shot_pos)
                # if aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].getPos(), 50):
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
                self.attack_pos = field.ball.getPos()
                field.allies[self.robot_with_ball].kickerChargeEnable = 1
                field.allies[self.robot_with_ball].kickerVoltage = 15
                field.allies[self.robot_with_ball].kick_up()
                if not(aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].getPos(), 500)):
                    # for index in range(len(used_pop_pos)):
                        # if used_pop_pos[index] == False:
                        #     waypoints[self.robot_with_ball] = wp.Waypoint(popusk_positions[index], 0, wp.WType.S_ENDPOINT)
                        #     waypoints[self.robot_with_ball] = wp.Waypoint(popusk_positions[index], 0, wp.WType.S_ENDPOINT)
                            # used_pop_pos[index] = True
                    self.popusk.append(self.robot_with_ball)
                    self.attack_state = "TO_BALL"
                    # field.allies[self.robot_with_ball].role = index
                    self.robot_with_ball = None
                            # break

                #ехать к нужной точке
            if self.robot_with_ball != None:
                if self.attack_state == "SHOOT":
                    waypoints[self.robot_with_ball] = wp.Waypoint(self.attack_pos, aux.angle_to_point(field.allies[self.robot_with_ball].getPos(), field.ball.getPos()), wp.WType.S_BALL_KICK)
                else:
                    waypoints[self.robot_with_ball] = wp.Waypoint(self.attack_pos, aux.angle_to_point(field.allies[self.robot_with_ball].getPos(), field.ball.getPos()), wp.WType.S_ENDPOINT)
                # print(self.attack_state)
        '''if self.connector != None:
            ball_line = aux.getBallLine()
            connect_pos = aux.find_nearest_point_on_the_line(ball_line, self.connector.pos)
            # if not aux.in_place(pos, connector.pos, 10):

            # else:
            
            # connector =
            waypoints[robot] = wp.Waypoint(connect_pos, field.ball.getPos().arg(), wp.WType.S_BALL_GRAB)'''
        for robot in self.popusk:
            pop_pos = field.allies[robot].role
            waypoints[robot] = wp.Waypoint(popusk_positions[pop_pos], 0, wp.WType.S_ENDPOINT)
        
    def prepare_penalty(self, field: field.Field, waypoints):
        if self.weActive:
            self.we_kick = 1
        else:
            self.we_kick = 0 
        if self.we_kick:
            rC = 0
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].rId != const.PENALTY_KICKER and field.allies[i].rId != const.GK:
                    waypoint = wp.Waypoint(aux.Point(3000 * field.side, 2500 - 1000 * rC), field.allies[i].getAngle(), wp.WType.S_ENDPOINT)
                    waypoints[i] = waypoint
                    rC+=1
            waypoint = wp.Waypoint(aux.Point(1700 * field.side, 0), aux.angle_to_point(aux.Point(1700 * field.side, 0), field.ball.getPos()), wp.WType.S_ENDPOINT)
            waypoints[field.allies[const.PENALTY_KICKER].rId] = waypoint
            waypoint = wp.Waypoint(field.ally_goal.center, aux.angle_to_point(field.ally_goal.center, field.ball.getPos()), wp.WType.S_ENDPOINT)
            waypoints[field.allies[const.GK].rId] = waypoint
        else:
            rC = 0
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].rId != const.GK:
                    waypoint = wp.Waypoint(aux.Point(3000 * -field.side, 2500 - 1000 * rC), 0, wp.WType.S_ENDPOINT)
                    waypoints[i] = waypoint
                    rC+=1
            waypoint = wp.Waypoint(field.ally_goal.center, aux.angle_to_point(field.ally_goal.center, field.ball.getPos()), wp.WType.S_ENDPOINT)
            waypoints[field.allies[const.GK].rId] = waypoint

    def halt(self, field: field.Field, waypoints):
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                waypoint = wp.Waypoint(field.allies[i].getPos(), field.allies[i].getAngle(), wp.WType.STOP)
                waypoints[i] = waypoint

    def timeout(self, field: field.Field, waypoints):
        rC = 0
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                waypoint = wp.Waypoint(aux.Point(2000 * field.side, 1250 - 500 * rC), field.allies[i].getAngle(), wp.WType.S_ENDPOINT)
                waypoints[i] = waypoint
                rC+=1
    
    def keep_distance(self, field: field.Field, waypoints):
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                if aux.dist(field.allies[i].getPos(), field.ball.getPos()) < const.KEEP_BALL_DIST:
                    waypoint = wp.Waypoint(aux.point_on_line(field.allies[i].getPos(), field.ball.getPos(), -const.KEEP_BALL_DIST), field.allies[i].getAngle(), wp.WType.S_ENDPOINT)
                    waypoints[i] = waypoint
    
    def penalty(self, field: field.Field, waypoints):
        if self.we_kick:
            self.penalty_kick(field, waypoints)
        else:
            robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
            self.gk_go(field, waypoints, [const.GK], robot_with_ball)

    def penalty_kick(self, field: field.Field, waypoints):
        if not self.is_started:
            self.is_started = 1
            self.ball_inside = 0
            self.last_seen = time.time() - 10
            self.ball_history = 0
            self.ticks_without_ball = 5
            self.penalty_timer = time.time()
        
        if self.ball_history != field.ball.getPos().x:
            self.ball_history = field.ball.getPos().x
            ball_visible = 1
            self.ticks_without_ball = 0
        elif self.ticks_without_ball < 30:
            self.ticks_without_ball+=1
            ball_visible = 1
        else:
            ball_visible = 0

        if ball_visible == 0 : print("can't see a ball")
        #print(self.ticks_without_ball)

        ball_visible = 1 #!!!!!!!!!!!!!DEBUG!!!!!!!!!!!!!!!!!!!!
        
        player_ID = const.PENALTY_KICKER
        enemy_keeper_ID = const.ENEMY_GK

        #for i in range(16): if allies[i].getPos().y != 0: print(i)
        
        field.allies[player_ID].autoKick = 0
        field.allies[player_ID].kickerChargeEnable = 1
        field.allies[player_ID].kickerVoltage = 50
        field.allies[player_ID].maxSpeed = 500
        kick_speed = 300
        kick_dist = 2000
        kick_delta = const.GOAL_DY / 2 - 50 ###const
        
        field.allies[player_ID].dribblerEnable = 1
        field.allies[player_ID].speedDribbler = 10

        #goal = aux.Point(1250, 0)  #add const field_size (preferably import from LARCmaCS)
        
        angle_to_goal=math.atan2(0 - field.allies[player_ID].getPos().y, field.enemy_goal.center.x - field.allies[player_ID].getPos().x)
        angle_to_ball=math.atan2(field.ball.getPos().y - field.allies[player_ID].getPos().y, field.ball.getPos().x - field.allies[player_ID].getPos().x) - angle_to_goal
        if angle_to_ball > math.pi: angle_to_ball-=2*math.pi
        if angle_to_ball < -math.pi: angle_to_ball+=2*math.pi
        #print(angle_to_ball)
        angle_ball_to_goal=math.atan2(0 - field.ball.getPos().y, field.enemy_goal.center.x - field.ball.getPos().x)
        delta = (field.allies[player_ID].getPos().x-field.ball.getPos().x)*math.sin(angle_ball_to_goal) - (field.allies[player_ID].getPos().y-field.ball.getPos().y)*math.cos(angle_ball_to_goal)
        #print("delta:   ", delta)
        
        if field.allies[player_ID].is_ball_in(field) or self.ball_inside == 1 and ball_visible == 0:
            self.last_seen = time.time()
        print("self.last_seen: ", self.ball_inside, - self.last_seen + time.time())

        if  time.time() - self.last_seen > 0.2 and ball_visible == 1 or ball_visible == 0 and self.ball_inside == 0:  
            waypoint = wp.Waypoint(field.ball.getPos(), (field.enemy_goal.center - field.allies[player_ID].getPos()).arg(), wp.WType.S_BALL_GRAB)
            self.ball_inside = 0
            print('going to ball')
        elif abs(field.enemy_goal.center.x - field.allies[player_ID].getPos().x) > kick_dist and abs(field.enemies[enemy_keeper_ID].getPos().x - field.allies[player_ID].getPos().x) > kick_dist and (time.time() - self.penalty_timer) < 8.5:
            field.allies[player_ID].maxSpeed -= max((field.allies[player_ID].maxSpeed - kick_speed) * kick_dist / abs(field.enemy_goal.center.x - field.allies[player_ID].getPos().x), 0)
            waypoint = wp.Waypoint(field.enemy_goal.center, (field.enemy_goal.center - field.allies[player_ID].getPos()).arg(), wp.WType.S_BALL_GO)
            self.ball_inside = 1
            print('too far')
        else:
            self.ball_inside = 1
            if field.enemies[enemy_keeper_ID].is_used():
                angle_to_keeper=math.atan2(field.enemies[enemy_keeper_ID].getPos().y - field.allies[player_ID].getPos().y, field.enemies[enemy_keeper_ID].getPos().x - field.allies[player_ID].getPos().x)
                angle_to_right_corner=math.atan2(kick_delta - field.allies[player_ID].getPos().y, field.enemy_goal.center.x - field.allies[player_ID].getPos().x)
                angle_to_left_corner=math.atan2(-kick_delta - field.allies[player_ID].getPos().y, field.enemy_goal.center.x - field.allies[player_ID].getPos().x)
            
                if abs(angle_to_keeper - angle_to_right_corner) > abs(angle_to_keeper - angle_to_left_corner):
                    target = aux.Point(field.enemy_goal.center.x, kick_delta)
                else:
                    target = aux.Point(field.enemy_goal.center.x, -kick_delta) 
            else:
                target = aux.Point(field.enemy_goal.center.x, 0) 
            angle_to_target=math.atan2(target.y - field.allies[player_ID].getPos().y, target.x - field.allies[player_ID].getPos().x)
            field.allies[player_ID].maxSpeed = kick_speed
            if abs(angle_to_target-field.allies[player_ID].angle)<math.pi/25:    #need to check speed of rotating(or increase angle's gate)
                waypoint = wp.Waypoint(field.enemy_goal.center, (target-field.allies[player_ID].getPos()).arg(), wp.WType.S_BALL_KICK)
                print('!!!KICK!!!')
            else:    
                waypoint = wp.Waypoint(field.enemy_goal.center, (target-field.allies[player_ID].getPos()).arg(), wp.WType.S_BALL_GO)
                print('aiming')
            
        #waypoint = wp.Waypoint(aux.Point(0, 0), aux.angle_to_point(allies[player_ID].getPos(), aux.Point(0, 0)), wp.WType.S_ENDPOINT)
        #allies[10].is_used = 1
        waypoints[player_ID] = waypoint
        #waypoints[player_ID] = wp.Waypoint(field.allies[player_ID].getPos(), (goal - field.allies[player_ID].getPos()).arg(), wp.WType.S_BALL_GO)

        #print(self.b_team.robot(1).x)
        
        #print(allies[player_ID].maxSpeed)

    def prepare_kickof(self, field, waypoints):
        if self.weActive:
            self.we_kick = 1
        else:
            self.we_kick = 0
        self.put_kickoff_waypoints(field, waypoints) 

    def put_kickoff_waypoints(self, field, waypoints):
        rC = 0
        if self.we_kick:
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].rId != const.GK:
                    if rC < 3:
                        if rC == 1:
                            waypoint = wp.Waypoint(aux.Point(700 * field.side, 0), aux.angle_to_point(field.allies[i].getPos(), aux.Point(0, 0)), wp.WType.S_ENDPOINT)
                        else:
                            waypoint = wp.Waypoint(aux.Point(700 * field.side, 2000 - 2000 * rC), aux.angle_to_point(field.allies[i].getPos(), aux.Point(0, 0)), wp.WType.S_ENDPOINT)
                        waypoints[i] = waypoint
                    else:
                        waypoint = wp.Waypoint(aux.Point(200 * field.side, 1500 - 3000 * (rC - 3)), aux.angle_to_point(field.allies[i].getPos(), aux.Point(0, 0)), wp.WType.S_ENDPOINT)
                        waypoints[i] = waypoint
                    rC+=1
        else:
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].rId != const.GK:
                    if rC == 0:
                        waypoint = wp.Waypoint(aux.Point(700 * field.side, 0), aux.angle_to_point(field.allies[i].getPos(), aux.Point(0, 0)), wp.WType.S_ENDPOINT)
                    elif rC < 3:
                        waypoint = wp.Waypoint(aux.Point(200 * field.side, 1000 - 2000 * (rC - 1)), aux.angle_to_point(field.allies[i].getPos(), aux.Point(0, 0)), wp.WType.S_ENDPOINT)
                    else:
                        waypoint = wp.Waypoint(aux.Point(200 * field.side, 2000 + 4000 * (rC - 4)), aux.angle_to_point(field.allies[i].getPos(), aux.Point(0, 0)), wp.WType.S_ENDPOINT)
                    waypoints[i] = waypoint
                    rC+=1
        waypoint = wp.Waypoint(field.ally_goal.center, aux.angle_to_point(field.ally_goal.center, field.ball.getPos()), wp.WType.S_ENDPOINT)
        waypoints[field.allies[const.GK].rId] = waypoint

    def kickoff(self, field, waypoints):
        self.put_kickoff_waypoints(field, waypoints) 
        if self.we_kick:
            go_kick = aux.find_nearest_robot(field.ball.getPos(), field.allies)
            target = aux.Point(700 * field.side, 2000)
            field.allies[go_kick.rId].kickerVoltage = 10
            waypoint = wp.Waypoint(field.ball.getPos(), (target - field.allies[go_kick.rId].getPos()).arg(), wp.WType.S_BALL_KICK)
            waypoints[go_kick.rId] = waypoint
        else:
            go_kick = aux.find_nearest_robot(field.ball.getPos(), field.allies)
            target = aux.point_on_line(field.ball.getPos(), aux.Point(field.side * const.GOAL_DX, 0), 200)
            waypoint = wp.Waypoint(target, (field.ball.getPos() - field.allies[go_kick.rId].getPos()).arg(), wp.WType.S_IGNOREOBSTACLES)
            waypoints[go_kick.rId] = waypoint

        robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
        self.gk_go(field, waypoints, [const.GK], robot_with_ball)

    def free_kick(self, field, waypoints):
        wall = []
        if not self.weActive:
            wall = self.defence(field, waypoints, wp.WType.S_KEEP_BALL_DISTANCE)
            self.keep_distance(field, waypoints) 
        else:
            self.attack(field, waypoints) 
        robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
        self.gk_go(field, waypoints, [const.GK] + wall, robot_with_ball)