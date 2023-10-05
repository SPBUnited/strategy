import math
import bridge.processors.const as const
import matplotlib.pyplot as plt
import numpy as np

class bobLine:
    def __init__(self, A, B, C):
        self.A = A
        self.B = B
        self.C = C

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
        return f'x = {self.x:.2f}, y = {self.y:.2f}'

    def mag(self):
        return math.hypot(self.x, self.y)

    def unity(self):
        if self.mag() == 0:
            #raise ValueError("БАГА, .unity от нулевого вектора")
            return self
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


def get_line_intersection(line1_start, line1_end, line2_start, line2_end, is_inf = 'SS'):
    """
    Получить точку пересечения отрезков или прямых

    is_inf задает ограничения на точку пересечения. Имеет вид 'AB', параметр A
    задает параметры первой прямой, B - второй.

    S(egment) - задан отрезок
    R(ay) - задан луч (начало - _start, направление - _end), точка пересечения валидна только
    если находится на луче _start-_end
    L(ine) - задана прямая
    """
    # Calculate the differences
    delta_x1 = line1_end.x - line1_start.x
    delta_y1 = line1_end.y - line1_start.y
    delta_x2 = line2_end.x - line2_start.x
    delta_y2 = line2_end.y - line2_start.y

    # Calculate the determinants
    determinant = delta_y1 * delta_x2 - delta_y2 * delta_x1

    if determinant == 0:
        # The lines are parallel or coincident
        return None

    # Calculate the differences between the start points
    delta_x_start = line1_start.x - line2_start.x
    delta_y_start = line1_start.y - line2_start.y

    # Calculate the t parameters
    t1 = (delta_x_start * delta_y2 - delta_x2 * delta_y_start) / determinant
    t2 = (delta_x_start * delta_y1 - delta_x1 * delta_y_start) / determinant

    intersection_x = line1_start.x + t1 * delta_x1
    intersection_y = line1_start.y + t1 * delta_y1
    p = Point(intersection_x, intersection_y)

    first_valid = False
    second_valid = False
    if is_inf[0] == 'S' and 0 <= t1 <= 1 or \
        is_inf[0] == 'R' and t1 >= 0 or \
        is_inf[0] == 'L':
        first_valid = True
    if is_inf[1] == 'S' and 0 <= t2 <= 1 or \
        is_inf[1] == 'R' and t2 >= 0 or \
        is_inf[1] == 'L':
        second_valid = True

    if first_valid and second_valid:
        return p

    return None



def vect_mult(v, u):
    return v.x * u.y - v.y * u.x

def scal_mult(v, u):
    return v.x * u.x + v.y * u.y

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

def peresek(mainLine, obj):
    res = []
    for point in obj:
        tmpLine = Line(point.getPos().x, point.getPos().y, point.getPos().x + (mainLine.endY - mainLine.startY), point.getPos().y - (mainLine.endX - mainLine.startX))
        print(mainLine.startX, mainLine.startY, mainLine.endX, mainLine.endY)
        print(tmpLine.startX, tmpLine.startY, tmpLine.endX, tmpLine.endY)
        res.append(get_line_intersection(Point(mainLine.startX, mainLine.startY), Point(mainLine.endX, mainLine.endY), Point(tmpLine.startX, tmpLine.startY), Point(tmpLine.endX, tmpLine.endY), "LL"))
    return res

def det (a,b,c,d):
	return a * d - b * c

def intersect (m, bots):
	result = []
	for n in bots:
		zn = det (m.A, m.B, n.A, n.B)
		res = Point(0, 0)
		if (abs (zn) < 1e-9):
			return None	
		res.x = - det (m.C, m.B, n.C, n.B) / zn
		res.y = - det (m.A, m.C, n.A, n.C) / zn
		result.append(res)
	return result

def probability(inter, bots):
    res = 1
    # print(len(inter), end = ' ')
    for i in range(len(inter)):
        koef = 1
        # print([inter[i].x, inter[i].y, bots[i].getPos().x, bots[i].getPos().y])
        tmpRes = ((inter[i].x - bots[i].getPos().x)**2 + (inter[i].y - bots[i].getPos().y)**2) ** (0.5) - const.ROBOT_R * 100 * 2
        # print(tmpRes)
        if tmpRes < 0: 
            koef = 0
        elif tmpRes > const.ROBOT_R * 100 * 15:
            koef = 1
        else:
            koef = tmpRes / (const.ROBOT_R * 100 * 15)
        res *= koef
    return res

# def probability(mainLine, obj):
#     inter = peresek(mainLine, obj)
#     # print(inter)
#     res = 1
#     for i in range(len(inter)):
#         koef = 1
#         tmpRes = (inter[i].x - obj[i].getPos().x)**2 + (inter[i].y - obj[i].getPos().y)**2 - const.ROBOT_R * 2
#         if tmpRes < 0: 
#             koef = 0
#         elif tmpRes > const.ROBOT_R * 4:
#             koef = 1
#         else:
#             koef = tmpRes / const.ROBOT_R * 4
#         res *= koef
#     return res

def botPosition(st, vecx, vecy):
    modul = (vecx**2 + vecy**2)**(0.5)
    vecx = (vecx / modul) * const.ROBOT_R * 1000 * 2
    vecy = (vecy / modul) * const.ROBOT_R * 1000 * 2
    return Point(st.x - vecx, st.y - vecy)

def shotDecision(st, end, obj):
    mx_shot_prob = 0
    shot_point = st
    mx = 0
    # print(st)
    # for bot in obj:
    #     # print([bot.getPos().x, bot.getPos().y], end = " ")
    #     plt.plot(bot.getPos().x, bot.getPos().y, 'bo')
    # t = np.arange(-4500*1.0, 1000*1.0, 10)
    for point in end:        
        A = -(point.y - st.y)
        B = (point.x - st.x)
        C = st.x * (point.y - st.y) - st.y * (point.x - st.x)
        tmpLine = bobLine(A, B, C)
        # plt.plot(t, (tmpLine.A*t + tmpLine.C)/tmpLine.B, 'g--')
        Lines = []
        for bot in obj:
            tmpC = -(B*bot.getPos().x - A*bot.getPos().y)
            L2 = bobLine(B, -A, tmpC)
            Lines.append(L2)
        inter = intersect(tmpLine, Lines)
        # plt.plot(inter[0].x, inter[0].y, 'bx')
        # plt.plot(inter[1].x, inter[1].y, 'gx')
        tmp_prob = probability(inter, obj)
        # print(tmp_prob, end = " ")
        if tmp_prob > mx:
            mx = tmp_prob
            shot_point = botPosition(st, point.x - st.x, point.y - st.y)
            Lres = point
    # plt.plot(t, -(Lres.A*t + Lres.C)/Lres.B, 'r-')
    # plt.plot(shot_point.x, shot_point.y, 'r^')
    # plt.axis('equal')
    # plt.grid(True)
    # plt.show()
    print(Lres)
    return shot_point, mx_shot_prob
    
def in_place(st, end, epsilon):
    if ((st - end).mag() < epsilon):
        return True
    else:
        return False