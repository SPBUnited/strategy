import bridge.processors.const as const
import bridge.processors.auxiliary as aux

class Team:
    def __init__(self, gk_id):
        self._robots = []
        self.gk_id = gk_id

    def play(self, enemy, ball):
        self.defence(enemy, ball)
        self.gk(enemy)


    def defence(self, enemy, ball):
        used_robots_id = [self.gk_id]
        robot_with_ball = self.find_nearest_robot(ball, enemy)

        def1 = self.find_nearest_robot(robot_with_ball, self, used_robots_id)

        used_robots_id.append(def1.rId)
        rbs = sorted(enemy.used_robots(), reverse=True, key=lambda x: x.x)
        rbs_rIds = []
        for r in rbs:
            if ((const.SIDE * r.x > (const.SIDE * robot_with_ball.x - 150)) or (const.SIDE * r.x > 0)) and r.rId != robot_with_ball.rId:  
                rbs_rIds.append(r.rId)

        for i in range(len(rbs_rIds)):
            def2 = self.find_nearest_robot(enemy.robot(rbs_rIds[i]), self, used_robots_id)
            used_robots_id.append(def2.rId)
            self.robot(def2.rId).go_to_point_with_detour(aux.point_on_line(enemy.robot(rbs_rIds[i]) , ball, 300), enemy, self)
            self.robot(def2.rId).rotate_to_point(enemy.robot(rbs_rIds[i]))


        self.robot(def1.rId).go_to_point_with_detour(aux.point_on_line(robot_with_ball, aux.Point(const.SIDE * 4500, 0), 300), enemy, self)
        self.robot(def1.rId).rotate_to_point(robot_with_ball)
        if len(self.used_robots()) - len(used_robots_id) == 1:
            for i in range(len(self.used_robots())):
                if self.robot(i).rId not in used_robots_id:
                    self.robot(i).go_to_point_with_detour(aux.Point(const.SIDE * 4000, 2000), enemy, self)
                    self.robot(i).rotate_to_point(ball)
        elif len(self.used_robots()) - len(used_robots_id) == 2:
            def2 = self.find_nearest_robot(aux.Point(const.SIDE * -4000, 1500), self, used_robots_id)
            used_robots_id.append(def2.rId)
            self.robot(def2.rId).go_to_point_with_detour(aux.Point(const.SIDE * -4000, 1500), enemy, self)
            self.robot(def2.rId).rotate_to_point(ball)

            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if self.robot(i).rId not in used_robots_id:
                    self.robot(i).go_to_point_with_detour(aux.Point(const.SIDE * -4000, -1500), enemy, self)
                    self.robot(i).rotate_to_point(ball)
        else:
            def2 = self.find_nearest_robot(aux.Point(const.SIDE * -4000, 1500), self, used_robots_id)
            used_robots_id.append(def2.rId)
            self.robot(def2.rId).go_to_point_with_detour(aux.Point(const.SIDE * -4000, 1500), enemy, self)
            self.robot(def2.rId).rotate_to_point(ball)
            
            def2 = self.find_nearest_robot(aux.Point(const.SIDE * -4000, -1500), self, used_robots_id)
            used_robots_id.append(def2.rId)
            self.robot(def2.rId).go_to_point_with_detour(aux.Point(const.SIDE * -4000, -1500), enemy, self)
            self.robot(def2.rId).rotate_to_point(ball)

            k = len(self.used_robots()) - len(used_robots_id)
            c = 0
            for i in range(len(self.used_robots())):
                if self.robot(i).rId not in used_robots_id:
                    if abs(ball.y) > 800:
                        bY =  800 * ball.y / abs(ball.y)
                        print(ball.x)
                        if abs(ball.x) < 3500:
                            bX = 3200
                        else:
                            bX = ball.x
                            
                    else:
                        bY = ball.y
                        bX = 3400
                    if k % 2 == 0:
                        if bX == ball.x:
                            self.robot(i).go_to_point(aux.Point(const.SIDE * bX - 200 * k / 2 + 200 * c, 1200 * ball.y / abs(ball.y)))
                            self.robot(i).rotate_to_point(aux.Point(self.robot(i).x, 5000 * ball.y / abs(ball.y)))
                            
                        else:
                            self.robot(i).go_to_point(aux.Point(const.SIDE * bX, bY - 200 * k / 2 + 200 * c))
                            self.robot(i).rotate_to_point(aux.Point(0, self.robot(i).y))
                            
                    else:
                        if bX == ball.x:
                            self.robot(i).go_to_point(aux.Point(const.SIDE * bX - 200 * (k - 1) + 200 * c, 1200 * ball.y / abs(ball.y)))
                            self.robot(i).rotate_to_point(aux.Point(self.robot(i).x, 5000 * ball.y / abs(ball.y)))
                            
                        else:
                            self.robot(i).go_to_point(aux.Point(const.SIDE * bX, bY - 200 * (k - 1) + 200 * c))
                            self.robot(i).rotate_to_point(aux.Point(0, self.robot(i).y))
                    c += 1

    def gk(self, enemy):
        self.robot(self.gk_id).go_to_point_with_detour(aux.Point(const.SIDE * 4300, 0), enemy, self)
        self.robot(self.gk_id).rotate_to_point(aux.Point(self.robot(self.gk_id).x, -3000))
        
    def add_robot(self, robot):
        self._robots.append(robot)

    def robots_amount(self):
        return len(self._robots)
    
    def robot(self, id):
        return self._robots[id]
    
    def robots(self):
        return self._robots.copy()
    
    def used_robots(self):
        rs = []
        for i in range(self.robots_amount()):
            if self.robot(i).isUsed:
                rs.append(self.robot(i))
        return rs
    
    def find_nearest_robot(self, robot, team, avoid = []):
        id = self.gk_id
        dist = 10e10
        for i in range(0, const.TEAM_ROBOTS_MAX_COUNT):
            if team.robot(i).rId in avoid:
                continue
            if aux.dist(robot, team.robot(i)) < dist:
                dist = aux.dist(robot, team.robot(i))
                id = i
        return team.robot(id)
