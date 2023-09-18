import math
import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as field

class Robot:
    def __init__(self, color, r_id, x, y, orientation):
        self.rId = r_id
        self.isUsed = 0
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

    """
    Двигаться в целевую точку согласно векторному полю
    """
    def go_to_point_vector_field(self, target_point, field: field.Field):

        self_pos = aux.Point(self.x, self.y)

        sep_dist = 400

        closest_robot = None
        closest_dist = math.inf
        closest_separation = 0

        # Расчет теней роботов для векторного поля
        for r in field.b_team:
            robot_separation = aux.dist(aux.closest_point_on_line(self_pos, target_point, r.getPos()), r.getPos())
            robot_dist = aux.dist(self_pos, r.getPos())
            if robot_separation < sep_dist and robot_dist < closest_dist:
                closest_robot = r
                closest_dist = robot_dist
                closest_separation = robot_separation

        vel_vec = target_point - self_pos
        if closest_robot != None:
            side = aux.vect_mult(closest_robot.getPos() - self_pos, target_point - self_pos)
            # offset_angle_val = 200*math.pi/6 * closest_dist**(-2)
            offset_angle_val = -2 * math.atan((sep_dist - closest_separation)/(2*(closest_dist)))
            offset_angle = offset_angle_val if side < 0 else -offset_angle_val
            vel_vec = aux.rotate(target_point - self_pos, offset_angle)

        vel_vec = vel_vec.unity()

        Fsum = aux.Point(0,0)

        # Расчет силы взаимодействия роботов
        for r in field.all_bots:
            delta_pos = self_pos - r.getPos()
            if delta_pos == aux.Point(0,0):
                continue
            F = delta_pos * 40 * (1 / delta_pos.mag()**3)
            Fsum = Fsum + F

        vel_vec = vel_vec + Fsum

        # Calculate the angle to the ball
        angle_to_point = math.atan2(vel_vec.y, vel_vec.x)

        # Calculate the distance to the ball
        distance_to_point = aux.dist(self_pos, target_point)

        newSpeed = min(self.maxSpeed, distance_to_point * 0.07)

        self.speedX = newSpeed * math.cos(angle_to_point - self.orientation)
        self.speedY = newSpeed * math.sin(angle_to_point - self.orientation)

        if const.IS_SIMULATOR_USED:
            self.speedY *= -1
        pass

    def go_to_point_with_detour(self, target_point, y_team, b_team):
        if aux.dist(self, target_point) < 500:
            self.go_to_point(target_point, 1)
        # Calculate the angle to the target point
        angle_to_point = math.atan2(target_point.y - self.y, target_point.x - self.x)

        # Calculate the distance to the target point
        distance_to_point = math.dist((self.x, self.y), (target_point.x, target_point.y))

        # Check if there are any robots in the way
        obstacles = []

        obstacle_distance_threshold = 600 # Adjust this threshold
        obstacle_distance_to_line_threshold = 300

        for i in range(y_team.robots_amount()):
            r = y_team.robot(i)
            if r != self:
                distance_to_robot = aux.dist(self, r)
                if distance_to_robot < obstacle_distance_threshold and aux.dist(self, target_point) != 0:
                    angle_to_robot = math.atan2(r.y - self.y, r.x - self.x)
                    angle_difference = abs(angle_to_robot - angle_to_point)
                    if angle_difference < math.pi / 4:  # Adjust this threshold for the angle
                        # Find the closest point on the line between self and target_point to y_robot
                        closest_point = aux.closest_point_on_line(self, target_point, r)
                        
                        # Calculate the distance from y_robot to the closest point on the line
                        distance_to_line = aux.dist(r, closest_point)
                        
                        if distance_to_line < obstacle_distance_to_line_threshold:
                            obstacles.append(r)

        for i in range(b_team.robots_amount()):
            r = b_team.robot(i)
            if r != self:
                distance_to_robot = aux.dist(self, r)
                if distance_to_robot < obstacle_distance_threshold and aux.dist(self, target_point) != 0:
                    angle_to_robot = math.atan2(r.y - self.y, r.x - self.x)
                    angle_difference = abs(angle_to_robot - angle_to_point)
                    if angle_difference < math.pi / 4:  # Adjust this threshold for the angle
                        # Find the closest point on the line between self and target_point to y_robot
                        closest_point = aux.closest_point_on_line(self, target_point, r)
                        
                        # Calculate the distance from y_robot to the closest point on the line
                        distance_to_line = aux.dist(r, closest_point)
                        
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
            tangent_point1 = aux.Point(
                self.x + tangent_distance * math.cos(angle_to_obstacle + angle_offset),
                self.y + tangent_distance * math.sin(angle_to_obstacle + angle_offset)
            )
            tangent_point2 = aux.Point(
                self.x + tangent_distance * math.cos(angle_to_obstacle - angle_offset),
                self.y + tangent_distance * math.sin(angle_to_obstacle - angle_offset)
            )

            # Check which tangent point is closer to the target point
            dist_to_tangent1 = aux.dist(tangent_point1, target_point)
            dist_to_tangent2 = aux.dist(tangent_point2, target_point)

            if dist_to_tangent1 < dist_to_tangent2:
                detour_point = tangent_point1
            else:
                detour_point = tangent_point2

            # Go to the detour point
            self.go_to_point(detour_point, 0)


    def go_to_point(self, point, is_final_point = 1):
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
        dif = math.atan2(aux.scal_mult(aux.Point(vx, vy), aux.Point(ux, uy)),
                            aux.vect_mult(aux.Point(vx, vy), aux.Point(ux, uy)))
        if const.IS_SIMULATOR_USED:
            dif *= -1

        if abs(dif) > 0.001:
            self.speedR = dif * 30
        else:
            self.speedR = 0
