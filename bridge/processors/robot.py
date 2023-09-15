import math
import bridge.processors.auxiliary as auxiliary
import bridge.processors.const as const

class Robot:
    def __init__(self, color, r_id, x, y, orientation):
        self.rId = r_id
        self.isUsed = 1
        self.x = x
        self.y = y
        self.orientation = orientation
        self.maxSpeed = const.MAX_SPEED
        self.maxSpeedR = const.MAX_SPEED_R
        self.color = color

        # Changeable params
        self.speedX = 0.0
        self.speedY = 0.0
        self.speedR = 0.0
        self.kickUp = 0.0
        self.kickForward = 0.0
        self.autoKick = 0.0
        self.kickerVoltage = const.BASE_KICKER_VOLTAGE
        self.dribblerEnable = 0.0
        self.speedDribbler = 0.0
        self.kickerChargeEnable = 0.0
        self.beep = 0.0
        self.acc = const.ACCELERATION

    def update(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.kickForward = 0
        self.kickUp = 0

    def kick_forward(self):
        self.kickForward = 1

    def kick_up(self):
        self.kickUp = 1

    def go_to_point_with_detour(self, target_point, y_team, b_team):
        # Calculate the angle to the target point
        angle_to_point = math.atan2(target_point.y - self.y, target_point.x - self.x)

        # Calculate the distance to the target point
        distance_to_point = math.dist((self.x, self.y), (target_point.x, target_point.y))

        # Check if there are any robots in the way
        obstacles = []

        obstacle_distance_threshold = 700  # Adjust this threshold
        obstacle_distance_to_line_threshold = 300

        for i in range(y_team.robots_amount()):
            r = y_team.robot(i)
            if r != self:
                distance_to_robot = auxiliary.dist(self, r)
                if distance_to_robot < obstacle_distance_threshold and auxiliary.dist(self, target_point) != 0:
                    angle_to_robot = math.atan2(r.y - self.y, r.x - self.x)
                    angle_difference = abs(angle_to_robot - angle_to_point)
                    if angle_difference < math.pi / 4:  # Adjust this threshold for the angle
                        # Find the closest point on the line between self and target_point to y_robot
                        closest_point = auxiliary.closest_point_on_line(self, target_point, r)
                        
                        # Calculate the distance from y_robot to the closest point on the line
                        distance_to_line = auxiliary.dist(r, closest_point)
                        
                        if distance_to_line < obstacle_distance_to_line_threshold:
                            obstacles.append(r)

        for i in range(b_team.robots_amount()):
            r = b_team.robot(i)
            if r != self:
                distance_to_robot = auxiliary.dist(self, r)
                if distance_to_robot < obstacle_distance_threshold and auxiliary.dist(self, target_point) != 0:
                    angle_to_robot = math.atan2(r.y - self.y, r.x - self.x)
                    angle_difference = abs(angle_to_robot - angle_to_point)
                    if angle_difference < math.pi / 4:  # Adjust this threshold for the angle
                        # Find the closest point on the line between self and target_point to y_robot
                        closest_point = auxiliary.closest_point_on_line(self, target_point, r)
                        
                        # Calculate the distance from y_robot to the closest point on the line
                        distance_to_line = auxiliary.dist(r, closest_point)
                        
                        if distance_to_line < obstacle_distance_to_line_threshold:
                            obstacles.append(r)

        if len(obstacles) == 0:
            # No obstacles, go directly to the target point
            self.go_to_point(target_point, 1)
        else:
           # Find the closest obstacle
            closest_obstacle = min(obstacles, key=lambda robot: math.dist((self.x, self.y), (robot.x, robot.y)))

            # Calculate the angle to the obstacle
            angle_to_obstacle = math.atan2(closest_obstacle.y - self.y, closest_obstacle.x - self.x)

            # Calculate the distances to the tangent points
            tangent_distance = 100
            angle_offset = math.asin(100 / tangent_distance)
            tangent_point1 = auxiliary.Point(
                self.x + tangent_distance * math.cos(angle_to_obstacle + angle_offset),
                self.y + tangent_distance * math.sin(angle_to_obstacle + angle_offset)
            )
            tangent_point2 = auxiliary.Point(
                self.x + tangent_distance * math.cos(angle_to_obstacle - angle_offset),
                self.y + tangent_distance * math.sin(angle_to_obstacle - angle_offset)
            )

            # Check which tangent point is closer to the target point
            dist_to_tangent1 = auxiliary.dist(tangent_point1, target_point)
            dist_to_tangent2 = auxiliary.dist(tangent_point2, target_point)

            if dist_to_tangent1 < dist_to_tangent2:
                detour_point = tangent_point1
            else:
                detour_point = tangent_point2

            # Go to the detour point
            self.go_to_point(detour_point, 0)


    def go_to_point(self, point, is_final_point):
        # Calculate the angle to the ball
        angle_to_point = math.atan2(point.y - self.y, point.x - self.x)

        # Calculate the distance to the ball
        distance_to_point = math.dist((self.x, self.y), (point.x, point.y))

        if is_final_point:
            newSpeed = min(self.maxSpeed, distance_to_point * 0.07)
        else:
            newSpeed = self.maxSpeed

        self.speedX = newSpeed * math.cos(angle_to_point - self.orientation)
        self.speedY = newSpeed * math.sin(angle_to_point - self.orientation)

        if const.IS_SIMULATOR_USED:
            self.speedY *= -1

    def get_speed(self, distance):
        pass

    def rotate_to_angle(self, angle):
        err = angle - self.orientation
        err = err % (2*math.pi)
        if err > math.pi:
            err -= 2*math.pi
        
        err *= -1

        if const.IS_SIMULATOR_USED:
            err *= -1

        if abs(err) > 0.001:
            self.speedR = err * 30
        else:
            self.speedR = 0


    def rotate_to_point(self, point):
        vx = self.x - point.x
        vy = self.y - point.y
        ux = -math.cos(self.orientation)
        uy = -math.sin(self.orientation)
        dif = math.atan2(auxiliary.scal_mult(auxiliary.Point(vx, vy), auxiliary.Point(ux, uy)),
                            auxiliary.vect_mult(auxiliary.Point(vx, vy), auxiliary.Point(ux, uy)))
        if const.IS_SIMULATOR_USED:
            dif *= -1

        if abs(dif) > 0.001:
            self.speedR = dif * 30
        else:
            self.speedR = 0
