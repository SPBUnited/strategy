## @package Stratery
# 
# Расчет требуемых положений роботов исходя из ситуации на поле

import bridge.processors.field as field
import bridge.processors.waypoint as wp
import bridge.processors.const as const
import bridge.processors.auxiliary as aux
import bridge.processors.signal as signal
import bridge.processors.robot as robot
import math
from enum import Enum

#!v DEBUG ONLY
import time


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
        self.PointRes = aux.Point(0,0)
        self.used_pop_pos = [False, False, False, False, False]

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

    def process(self, field: field.Field):
        """
        Рассчитать конечные точки для каждого робота
        """
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
        if field.ball.getVel().mag() < 300:
            if self.is_ball_moved or time.time() - self.timer > 7:
                rivals_robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies, [const.ENEMY_GK])
                allies_robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.allies, [const.GK])
                self.timer = time.time()
                if aux.dist(rivals_robot_with_ball.getPos(), field.ball.getPos()) < aux.dist(allies_robot_with_ball.getPos(), field.ball.getPos()):
                    self.state = States.DEFENCE
                    print("defence")
                else:
                    self.state = States.ATTACK
                    self.reset_all_attack_var()
                    print("attack")
            self.is_ball_moved = 0
            if abs(const.GOAL_DX - abs(field.ball.getPos().x)) < const.GOAL_DY and abs(field.ball.getPos().y) < const.GOAL_DY:
                if abs(field.ball.getPos().x - field.enemy_goal.center.x) < abs(field.ball.getPos().x - field.ally_goal.center.x): 
                    self.state = States.DEFENCE
                else:
                    self.reset_all_attack_var()
                    self.state = States.ATTACK
        else:
            self.is_ball_moved = 1
        wall = []

        # self.state = States.ATTACK

        if self.state == States.DEBUG:
            self.debug(field, waypoints)
        elif self.state == States.DEFENCE:
            wall = self.defence(field, waypoints)
        elif self.state == States.ATTACK:
            self.decide_popusk_position(field)
            self.preAttack(field)
            
                # if not self.calc:
            
                #     if len(self.popusk) != 0:
                #         self.calc = True
            self.attack(field, waypoints)
        
        if self.state != States.DEFENCE:
            self.old_def_helper = -1
            self.old_def = -1
            self.steal_flag = 0

        robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
        self.gk_go(field, waypoints, [const.GK] + wall, robot_with_ball)

    square = signal.Signal(8, 'SQUARE', ampoffset=(300, 0))
    square_ang = signal.Signal(8, 'SQUARE', lohi=(0, 3.14))

    def debug(self, field: field.Field, waypoints):

        # waypoints[const.DEBUG_ID].pos = aux.Point(0, 0)        
        # waypoints[const.DEBUG_ID].angle = 3.14
        # waypoints[const.DEBUG_ID].type = wp.WType.S_ENDPOINT        
        
        robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.allies)
        self.gk_go(field, waypoints, [const.GK], robot_with_ball)

        # for i in range(0, 6):
        #     # pos = aux.point_on_line(bbotpos, -aux.Point(const.GOAL_DX, 0), 300)
        #     pos = -field.ally_goal.eye_forw*500*(i-0) -field.ally_goal.eye_up*1000*self.square.get()*0
        #     # pos = aux.Point(1000 + self.square.get(), -1000)
            
        #     # dpos = bbotpos - ybotpos
        #     # angle = math.atan2(dpos.y, dpos.x)
        #     # angle = self.square_ang.get()
        #     angle = math.pi/2 * 1

        #     waypoint = wp.Waypoint(pos, angle, wp.WType.S_ENDPOINT)
        #     waypoints[i] = waypoint

        # robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
        # self.gk_go(field, waypoints, [const.GK], None)
        # waypoints[const.DEBUG_ID].pos = aux.Point(-1000, self.square.get())
        # # waypoints[const.DEBUG_ID].angle = self.square_ang.get()
        # waypoints[const.DEBUG_ID].angle = math.pi/4*0
        # waypoints[const.DEBUG_ID].type = wp.WType.S_ENDPOINT

        waypoints[const.DEBUG_ID].pos = field.ball.getPos()
        waypoints[const.DEBUG_ID].angle = (field.ally_goal.center - field.ball.getPos() + aux.j*self.square.get()).arg()
        # waypoints[const.DEBUG_ID].angle = math.pi/2
        waypoints[const.DEBUG_ID].type = wp.WType.S_BALL_KICK

        # print(field.ball.getPos(), waypoints[const.DEBUG_ID])

        
        # waypoints[const.GK].pos = -field.ally_goal.eye_forw*2000 + field.ally_goal.eye_up * (self.square.get() - 000)
        # waypoints[const.GK].angle = self.square_ang.get()


        RID = 9
        # egf = field.ally_goal.forwdown + field.ally_goal.eye_forw*300
        # if not field.allies[RID].is_ball_in(field):
        #     waypoints[RID] = wp.Waypoint(field.ball.getPos(), (field.ally_goal.center - field.ball.getPos()).arg(), wp.WType.S_BALL_GRAB)
        # elif (egf - field.allies[RID].getPos()).mag() > 300:
        #     waypoints[RID] = wp.Waypoint(egf, math.pi/3, wp.WType.S_BALL_GO)
        # else:
        # waypoints[RID] = wp.Waypoint(field.ball.getPos(), 0, wp.WType.S_BALL_KICK)

        return waypoints

    def gk_go(self, field: field.Field, waypoints, gk_wall_idx_list, robot_with_ball: robot.Robot):
        """
        Расчет требуемых положений вратаря и стенки
        
        [in] field - объект Field(), ситуация на поле
        [out] waypoints - ссылка на массив путевых точек, который будет изменен!
        [in] gk_wall_idx_list - массив индексов вратаря и стенки. Нулевой элемент всегда индекс вратаря
        Дальше индексы роботов в стенке, кол-во от 0 до 3
        [in] robot_with_ball - текущий робот с мячом
        """
        if robot_with_ball is not None:
            predict = aux.get_line_intersection(robot_with_ball.getPos(), robot_with_ball.getPos() + aux.rotate(aux.i, robot_with_ball.getAngle()),
                                                    field.ally_goal.down, field.ally_goal.up, 'RS')
            if predict is not None:
                p_ball = (field.ball.getPos() - predict).unity()
                gk_pos = aux.LERP(aux.point_on_line(field.ally_goal.center, field.ball.getPos(), const.GK_FORW),
                                aux.get_line_intersection(robot_with_ball.getPos(), robot_with_ball.getPos() + aux.rotate(aux.i, robot_with_ball.getAngle()),
                                                        field.ally_goal.down, field.ally_goal.up, 'RS') + p_ball*const.GK_FORW,
                                0.5)
                # print("PREDICTION: ", robot_with_ball.getAngle())
            else:
                # print("NO PREDICTION1")
                gk_pos = aux.point_on_line(field.ally_goal.center, field.ball.getPos(), const.GK_FORW)
        else:
            # print("NO PREDICTION2")
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
            if ((field.side * r.getPos().x > 0)) and r.rId != robot_with_ball.rId and r.is_used() and r.rId != const.ENEMY_GK:  
                rbs_rIds.append(r.rId)


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

        for i in range(len(rbs_rIds)):
            defender = aux.find_nearest_robot(field.enemies[rbs_rIds[i]].getPos(), field.allies, used_robots_id)
            used_robots_id.append(defender.rId)

            targetPoint = aux.point_on_line(field.enemies[rbs_rIds[i]].getPos(), field.ball.getPos(), dist_between)
            waypoint = wp.Waypoint(targetPoint, aux.angle_to_point(targetPoint, field.ball.getPos()), ENDPOINT_TYPE)
            waypoints[defender.rId] = waypoint

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

    def reset_all_attack_var(self):
        #return
        self.used_pop_pos = [False, False, False, False, False]
        self.robot_with_ball = None
        self.connector = []
        self.popusk = []
        self.attack_state = "TO_BALL"
        self.attack_pos = aux.Point(0,0)
        self.calc = False
        self.PointRes = (0,0)
    
    def decide_popusk_position(self, field: field.Field):
        for pointIndex in range(len(field.enemy_goal.popusk_positions)):
            save_robot = -1
            if self.used_pop_pos[pointIndex] == False:
                mn = 1e10
                for robot in field.allies:
                    if robot.is_used() and robot.rId != const.GK and robot.rId != self.robot_with_ball and not aux.is_in_list(self.popusk, robot.rId):
                        pop_pos_dist = aux.dist(robot.getPos(), field.enemy_goal.popusk_positions[pointIndex])
                        if mn > pop_pos_dist:
                            mn = pop_pos_dist
                            save_robot = robot.rId
                    elif not robot.is_used() and field.allies[robot.rId].role != None:
                        self.used_pop_pos[robot.role] = False
                        field.allies[robot.rId].role = None
                        if aux.is_in_list(self.popusk, robot.rId):
                            self.popusk.pop(self.popusk.index(robot.rId))
            if save_robot != -1:
                field.allies[save_robot].role = pointIndex
                self.popusk.append(save_robot)
                self.used_pop_pos[pointIndex] = True
                
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
        elif self.robot_with_ball != None and not field.allies[self.robot_with_ball].is_used():
            field.allies[self.robot_with_ball].role = None
            self.robot_with_ball = None
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
        goal_points = [aux.Point(field.enemy_goal.center.x, i) for i in range(-200, 201, 40)]
        bro_points = [field.allies[i].getPos() for i in range(len(field.allies))]
        for robot in self.popusk:
            pop_pos = field.allies[robot].role
            waypoints[robot] = wp.Waypoint(field.enemy_goal.popusk_positions[pop_pos], 0, wp.WType.S_ENDPOINT)
        print(self.PointRes)
        if self.robot_with_ball != None:
            if self.attack_state == "TO_BALL":
                self.attack_pos = field.ball.getPos()
                self.PointRes = field.enemy_goal.center
                if aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].getPos(), 1000):
                    self.attack_state = "CALCULATING"
            elif self.attack_state == "CALCULATING":
                shot_pos, shot_prob, self.PointRes = aux.shotDecision(field.ball.getPos(), goal_points, field.enemies)
                # self.PointRes.y *= -1
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
                # field.allies[self.robot_with_ball].kickerChargeEnable = 1
                # field.allies[self.robot_with_ball].kickerVoltage = 15
                if aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].getPos(), 50):
                    self.attack_state = "SHOOT"
            elif self.attack_state == "SHOOT":
                self.attack_pos = field.ball.getPos()
                # field.allies[self.robot_with_ball].kickerChargeEnable = 1
                # field.allies[self.robot_with_ball].kickerVoltage = 15
                # field.allies[self.robot_with_ball].kick_up()
                if not(aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].getPos(), 3000)):
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
                # if self.attack_state == "SHOOT":
                  waypoints[self.robot_with_ball] = wp.Waypoint(field.ball.getPos(), aux.angle_to_point(field.ball.getPos(), self.PointRes), wp.WType.S_BALL_KICK)
                # else:
                #     waypoints[self.robot_with_ball] = wp.Waypoint(self.attack_pos, aux.angle_to_point(field.ball.getPos(), field.enemy_goal.center), wp.WType.S_BALL_KICK)
                # print(self.attack_state)
        '''if self.connector != None:
            ball_line = aux.getBallLine()
            connect_pos = aux.find_nearest_point_on_the_line(ball_line, self.connector.pos)
            # if not aux.in_place(pos, connector.pos, 10):

            # else:
            
            # connector =
            waypoints[robot] = wp.Waypoint(connect_pos, field.ball.getPos().arg(), wp.WType.S_BALL_GRAB)'''
              
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
                waypoint = wp.Waypoint(aux.Point(2000 * field.side, 1250 - 500 * rC), 0, wp.WType.S_ENDPOINT)
                waypoints[i] = waypoint
                rC+=1
    
    def keep_distance(self, field: field.Field, waypoints):
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                if aux.dist(field.allies[i].getPos(), field.ball.getPos()) < const.KEEP_BALL_DIST:
                    waypoint = wp.Waypoint(aux.point_on_line(field.ball.getPos(), aux.Point(0,0), const.KEEP_BALL_DIST), field.allies[i].getAngle(), wp.WType.S_ENDPOINT)
                    waypoints[i] = waypoint
    
    def penalty(self, field: field.Field, waypoints):
        if self.we_kick or 1:
            self.penalty_kick(field, waypoints)
        else:
            robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
            self.gk_go(field, waypoints, [const.GK], robot_with_ball)

    def penalty_kick(self, field: field.Field, waypoints):
        self.is_started+=1
        if self.is_started < 5:
            field.allies[const.PENALTY_KICKER].kickUp = 1
        
        field.allies[const.PENALTY_KICKER].dribblerEnable = 1
        field.allies[const.PENALTY_KICKER].speedDribbler = 10
        field.allies[const.PENALTY_KICKER].kickerChargeEnable = 2
        field.allies[const.PENALTY_KICKER].autoKick = 2 
        kick_delta = 400

        angle_to_keeper=math.atan2(field.enemies[const.ENEMY_GK].getPos().y - field.allies[const.PENALTY_KICKER].getPos().y, field.enemies[const.ENEMY_GK].getPos().x - field.allies[const.PENALTY_KICKER].getPos().x)
        angle_to_right_corner=math.atan2(kick_delta - field.allies[const.PENALTY_KICKER].getPos().y, field.enemy_goal.center.x - field.allies[const.PENALTY_KICKER].getPos().x)
        angle_to_left_corner=math.atan2(-kick_delta - field.allies[const.PENALTY_KICKER].getPos().y, field.enemy_goal.center.x - field.allies[const.PENALTY_KICKER].getPos().x)
    
        if abs(angle_to_keeper - angle_to_right_corner) > abs(angle_to_keeper - angle_to_left_corner):
            target = aux.Point(field.enemy_goal.center.x, kick_delta)
        else:
            target = aux.Point(field.enemy_goal.center.x, -kick_delta)

        # print(field.ball.getPos().x)
        if abs(field.enemy_goal.center.x - field.ball.getPos().x) > 1000:
            field.allies[const.PENALTY_KICKER].kickerVoltage = 5
            waypoint = wp.Waypoint(field.ball.getPos(), (target - field.allies[const.PENALTY_KICKER].getPos()).arg(), wp.WType.S_BALL_KICK)
        else:
            field.allies[const.PENALTY_KICKER].kickerVoltage = 15
            waypoint = wp.Waypoint(field.ball.getPos(), (target - field.allies[const.PENALTY_KICKER].getPos()).arg(), wp.WType.S_BALL_KICK)
                
        waypoints[const.PENALTY_KICKER] = waypoint

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
        # self.we_kick = 0
        if self.we_kick:
            go_kick = aux.find_nearest_robot(field.ball.getPos(), field.allies)
            target = field.enemy_goal.center
            target.y = 300
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
            self.state = States.ATTACK
            self.decide_popusk_position(field)
            self.preAttack(field)
            self.attack(field, waypoints)
        robot_with_ball = aux.find_nearest_robot(field.ball.getPos(), field.enemies)
        self.gk_go(field, waypoints, [const.GK] + wall, robot_with_ball)
