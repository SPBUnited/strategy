import bridge.processors.const as const
import bridge.processors.auxiliary as auxiliary

class Team:
    def __init__(self, gk_id):
        self._robots = []
        self.gk_id = gk_id

    def play(self, enemy, ball):
        self.gk(enemy)
        self.defence(enemy, ball)


    def defence(self, enemy, ball):
        used_robots_id = [self.gk_id]
        robot_with_ball = self.find_nearest_robot(ball, enemy)

        def1 = self.find_nearest_robot(robot_with_ball, self, used_robots_id)
        self.robot(def1.rId).go_to_point_with_detour(auxiliary.point_on_line(robot_with_ball, auxiliary.Point(4500, 0), 300), enemy, self)
        self.robot(def1.rId).rotate_to_point(robot_with_ball)

        used_robots_id.append(robot_with_ball.rId)
        rbs = sorted(enemy.robots(), reverse=True, key=lambda x: x.x)
        rbs_rIds = []
        for r in rbs:
            if r.x > robot_with_ball.x:
                rbs_rIds.append(r.rId)

        for i in range(len(rbs_rIds)):
            def1 = self.find_nearest_robot(enemy.robot(rbs_rIds[i]), self, used_robots_id)
            used_robots_id.append(def1.rId)
            self.robot(def1.rId).go_to_point_with_detour(auxiliary.point_on_line(enemy.robot(rbs_rIds[i]) , auxiliary.Point(4500, 0), 300), enemy, self)
            self.robot(def1.rId).rotate_to_point(enemy.robot(rbs_rIds[i]))

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if self.robot(i).rId not in used_robots_id:
                self.robot(i).go_to_point_with_detour(auxiliary.Point(4500, 3000), enemy, self)


            


    def gk(self, enemy):
        self.robot(self.gk_id).go_to_point_with_detour(auxiliary.Point(4300, 0), enemy, self)
        self.robot(self.gk_id).rotate_to_point(auxiliary.Point(self.robot(self.gk_id).x, -3000))
        
    def add_robot(self, robot):
        self._robots.append(robot)

    def robots_amount(self):
        return len(self._robots)
    
    def robot(self, id):
        return self._robots[id]
    
    def robots(self):
        return self._robots.copy()
    
    def find_nearest_robot(self, robot, team, avoid = []):
        id = self.gk_id
        dist = 10e10
        for i in range(0, const.TEAM_ROBOTS_MAX_COUNT):
            if team.robot(i).rId in avoid:
                continue
            if auxiliary.dist(robot, team.robot(i)) < dist:
                dist = auxiliary.dist(robot, team.robot(i))
                id = i
        return team.robot(id)
