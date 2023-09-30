import math
import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as field
import bridge.processors.entity as entity

class Robot(entity.Entity):
    def __init__(self, pos, angle, R, color, r_id):
        super().__init__(pos, angle, R)

        self.rId = r_id
        self.isUsed = 0
        self.color = color
        self.lastUpdate = 0
        self.errOld = 0

        self.speedX = 0
        self.speedY = 0
        self.speedR = 0
        self.kickUp = 0
        self.kickForward = 0
        self.autoKick = 0
        self.kickerVoltage = 15
        self.dribblerEnable = 0
        self.speedDribbler = 0
        self.kickerChargeEnable = 1
        self.beep = 0   

    def used(self, a):
        self.isUsed = a

    def is_used(self):
        return self.isUsed
    
    def last_update(self):
        return self.lastUpdate

    def update(self, pos, angle, t):
        super().update(pos, angle)
        self.kickForward = 0
        self.kickUp = 0
        self.lastUpdate = t

    def kick_forward(self):
        self.kickForward = 1

    def kick_up(self):
        self.kickUp = 1

    def copy_control_fields(self, robot):
        self.speedX = robot.speedX
        self.speedY = robot.speedY
        self.speedR = robot.speedR
        self.kickUp = robot.kickUp
        self.kickForward = robot.kickForward
        self.autoKick = robot.autoKick
        self.kickerVoltage = robot.kickerVoltage
        self.dribblerEnable = robot.dribblerEnable
        self.speedDribbler = robot.speedDribbler
        self.kickerChargeEnable = robot.kickerChargeEnable
        self.beep = robot.beep

    def clear_fields(self):
        self.speedX = 0
        self.speedY = 0
        self.speedR = 0
        self.kickUp = 0
        self.kickForward = 0
        self.autoKick = 0
        self.kickerVoltage = 0
        self.dribblerEnable = 0
        self.speedDribbler = 0
        self.kickerChargeEnable = 0
        self.beep = 0 


    """
    Двигаться в целевую точку согласно векторному полю
    """
    def go_to_point_vector_field(self, target_point, field: field.Field):

        enemy = field.b_team
        self_pos = self.getPos()

        if (self_pos - target_point).mag() == 0:
            self.speedX = 0
            self.speedY = 0
            return



        sep_dist = 500

        closest_robot = None
        closest_dist = math.inf
        closest_separation = 0
        angle_to_point = math.atan2(target_point.y - self.getPos().y, target_point.x - self.getPos().x)

        # Расчет теней роботов для векторного поля
        for r in field.all_bots:
            if r == self or r.is_used() == 0:
                continue
            robot_separation = aux.dist(aux.closest_point_on_line(self_pos, target_point, r.getPos()), r.getPos())
            robot_dist = aux.dist(self_pos, r.getPos())
            if robot_dist == 0:
                continue
            if robot_separation < sep_dist and robot_dist < closest_dist:
                closest_robot = r
                closest_dist = robot_dist
                closest_separation = robot_separation

        vel_vec = target_point - self_pos
        angle_cos = aux.scal_mult((r.getPos() - self_pos).unity(), (target_point - self_pos).unity())
        if closest_robot != None and closest_dist != 0:
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

        angle_to_point = math.atan2(vel_vec.y, vel_vec.x)

        distance_to_point = aux.dist(self_pos, target_point)
        

        distance_to_point = aux.dist(self_pos, target_point)

        newSpeed = min(const.MAX_SPEED, distance_to_point * const.KP)

        self.speedX = newSpeed * math.cos(angle_to_point - self.getAngle())
        self.speedY = -newSpeed * math.sin(angle_to_point - self.getAngle())

    def remake_speed(self):
        vecSpeed = math.sqrt(self.speedX ** 2 + self.speedY ** 2)
        rSpeed = abs(self.speedR)

        vecSpeed *= ((const.MAX_SPEED_R - rSpeed) / const.MAX_SPEED_R) ** 5
        ang = math.atan2(self.speedY, self.speedX)
        self.speedX = vecSpeed * math.cos(ang)
        self.speedY = vecSpeed * math.sin(ang)

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
        angle_to_point = math.atan2(point.y - self.getPos().y, point.x - self.getPos().x)

        # Calculate the distance to the ball
        distance_to_point = math.dist((self.getPos().x, self.getPos().y), (point.x, point.y))

        if is_final_point:
            newSpeed = min(const.MAX_SPEED,  distance_to_point * const.KP)
        else:
            newSpeed = const.MAX_SPEED
        self.speedX = newSpeed * math.cos(angle_to_point - self.getAngle())
        self.speedY = newSpeed * math.sin(angle_to_point - self.getAngle())

        if const.IS_SIMULATOR_USED:
            self.speedY *= -1

    def rotate_to_angle(self, angle):
        err = angle - self.getAngle()
        err = err % (2*math.pi)
        if err > math.pi:
            err -= 2*math.pi
        
        if abs(err) > 0.001:
            self.speedR = min(err * const.R_KP + (err - self.errOld) * const.R_KD, const.MAX_SPEED_R)
        else:
            self.speedR = 0
        self.errOld = err


    def rotate_to_point(self, point):
        vx = self.getPos().x - point.x
        vy = self.getPos().y - point.y
        ux = -math.cos(self.getAngle())
        uy = -math.sin(self.getAngle())
        err = -math.atan2(aux.vect_mult(aux.Point(vx, vy), aux.Point(ux, uy)),
                            aux.scal_mult(aux.Point(vx, vy), aux.Point(ux, uy)))

        if abs(err) > 0.001:
            self.speedR = min(err * const.R_KP + (err - self.errOld) * const.R_KD, const.MAX_SPEED_R)
        else:
            self.speedR = 0
        self.errOld = err


    def __str__(self) -> str:
        return str(str(self.color) + " " + str(self.rId) + " " + str(self.pos))

