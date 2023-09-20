import math
import bridge.processors.const as const

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, p):
        return Point(self.x + p.x, self.y + p.y)

    def __neg__(self):
        return Point(-self.x, -self.y)

    def __sub__(self, p):
        return self + -p

    def __mul__(self, a: float):
        return Point(self.x * a, self.y * a)

    def __truediv__(self, a: float):
        return self * (1/a)

    def __pow__(self, a: float):
        return Point(self.x ** a, self.y ** a)

    def __eq__(self, p):
        return self.x == p.x and self.y == p.y

    def __str__(self):
        # return "x = " + str(self.x) + ", y = " + str(self.y)
        return f'x = {self.x:.4f}, y = {self.y:.4f}'

    def mag(self):
        return math.hypot(self.x, self.y)

    def unity(self):
        if self.mag() == 0:
            #print("БАГА, .unity от нулевого вектора")
            return Point(1, 0)
        return self/self.mag()


class Line:
    def __init__(self, start_x, start_y, end_x, end_y):
        self.startX = start_x
        self.startY = start_y
        self.endX = end_x
        self.endY = end_y


class Circle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


def dist(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def construct_tangents(robot, obstacle):
    tangents = []

    robot_to_obstacle = math.hypot(obstacle.x - robot.x, obstacle.y - robot.y)
    robot_radius = robot.size

    if robot_to_obstacle <= 2 * robot_radius:
        return tangents

    # Calculate the angle between the robot and the obstacle
    angle_to_obstacle = math.atan2(obstacle.y - robot.y, obstacle.x - robot.x)

    # Calculate the tangent angles
    tangent_angle_1 = angle_to_obstacle + math.asin(2 * robot_radius / robot_to_obstacle)
    tangent_angle_2 = angle_to_obstacle - math.asin(2 * robot_radius / robot_to_obstacle)

    # Calculate the tangent points
    tangent_point_1 = Point(robot.x + robot_radius * math.cos(tangent_angle_1),
                            robot.y + robot_radius * math.sin(tangent_angle_1))

    tangent_point_2 = Point(robot.x + robot_radius * math.cos(tangent_angle_2),
                            robot.y + robot_radius * math.sin(tangent_angle_2))

    # Create tangent lines
    tangent_line_1 = Line(robot.x, robot.y, tangent_point_1.x, tangent_point_1.y)
    tangent_line_2 = Line(robot.x, robot.y, tangent_point_2.x, tangent_point_2.y)

    tangents.append(tangent_line_1)
    tangents.append(tangent_line_2)

    return tangents


def calculate_path_length(start_point, end_point, obstacles):
    path_length = math.hypot(end_point.x - start_point.x, end_point.y - start_point.y)

    for obstacle in obstacles:
        tangents = construct_tangents(start_point, obstacle)

        for tangent in tangents:
            intersection_point = get_line_intersection(start_point, end_point, Point(tangent.startX, tangent.startY),
                                                       Point(tangent.endX, tangent.endY))

            if intersection_point:
                partial_path_length = math.hypot(intersection_point.x - start_point.x,
                                                 intersection_point.y - start_point.y)

                if partial_path_length < path_length:
                    path_length = partial_path_length

    return path_length


def get_line_intersection(line1_start, line1_end, line2_start, line2_end):
    # Calculate the differences
    delta_x1 = line1_end.x - line1_start.x
    delta_y1 = line1_end.y - line1_start.y
    delta_x2 = line2_end.x - line2_start.x
    delta_y2 = line2_end.y - line2_start.y

    # Calculate the determinants
    determinant = delta_x1 * delta_y2 - delta_x2 * delta_y1

    if determinant == 0:
        # The lines are parallel or coincident
        return None

    # Calculate the differences between the start points
    delta_x_start = line1_start.x - line2_start.x
    delta_y_start = line1_start.y - line2_start.y

    # Calculate the t parameters
    t1 = (delta_x_start * delta_y2 - delta_x2 * delta_y_start) / determinant
    t2 = (delta_x_start * delta_y1 - delta_x1 * delta_y_start) / determinant

    if 0 <= t1 <= 1 and 0 <= t2 <= 1:
        # The lines intersect within their segments
        intersection_x = line1_start.x + t1 * delta_x1
        intersection_y = line1_start.y + t1 * delta_y1
        return Point(intersection_x, intersection_y)

    # The lines do not intersect within their segments
    return None


def vect_mult(v, u):
    return v.x * u.y - v.y * u.x

def scal_mult(v, u):
    return v.x * u.x + v.x * u.x

def rotate(p: Point, angle: float):
    return Point(p.x * math.cos(angle) - p.y * math.sin(angle),
                 p.y * math.cos(angle) + p.x * math.sin(angle))

def find_nearest_robot(robot, team, avoid = []):
    id = -1
    minDist = 10e10
    for i in range(0, const.TEAM_ROBOTS_MAX_COUNT):
        if i in avoid or not team[i].isUsed:
            continue
        if dist(robot, team[i].getPos()) < minDist:
            minDist = dist(robot, team[i].getPos())
            id = i
    return team[id]


def format_angle(ang):
    while ang > math.pi:
        ang -= 2 * math.pi
    while ang < -math.pi:
        ang += 2 * math.pi
    return ang

def closest_point_on_line(point1, point2, point):
    line_vector = (point2.x - point1.x, point2.y - point1.y)
    line_length = dist(point1, point2)
    
    if line_length == 0:
        return point1
    
    line_direction = (line_vector[0] / line_length, line_vector[1] / line_length)
    
    point_vector = (point.x - point1.x, point.y - point1.y)
    dot_product = point_vector[0] * line_direction[0] + point_vector[1] * line_direction[1]
    
    if dot_product <= 0:
        return point1
    elif dot_product >= line_length:
        return point2
    
    closest_point = Point(
        point1.x + line_direction[0] * dot_product,
        point1.y + line_direction[1] * dot_product
    )
    
    return closest_point

def point_on_line(robot, point, distance):
        angle_to_point = math.atan2(point.y - robot.y, point.x - robot.x)

        # Calculate the new point on the line at the specified distance from the robot
        new_x = robot.x + distance * math.cos(angle_to_point)
        new_y = robot.y + distance * math.sin(angle_to_point)
        return Point(new_x, new_y)

def angle_to_point(point1, point2):
    dpos = -point1 + point2
    angle = math.atan2(dpos.y, dpos.x)

    return angle

def sign(num):
    if num == 0:
        return 0
    return num / abs(num)