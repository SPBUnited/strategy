"""
Модуль вспомогательной математики и утилит
"""

import math
import typing
from typing import Optional, Sequence

from bridge import const


class Graph:
    """
    Класс для работы с графами
    """

    def __init__(self, num_vertices: int) -> None:
        """
        Конструктор

        Аллоцирует память под граф с num_vertices вершинами
        """
        self.num_vertices = num_vertices
        self.graph = [[0] * num_vertices for _ in range(num_vertices)]

    def add_edge(self, from_vertex: int, to_vertex: int, weight: int) -> None:
        """
        Добавить ребро графу
        """
        self.graph[from_vertex][to_vertex] = weight
        self.graph[to_vertex][from_vertex] = weight

    def dijkstra(self, start_vertex: int) -> list[float]:
        """
        Найти кратчайший путь в графе используя алгоритм Дейкстры
        """
        distances = [float("inf")] * self.num_vertices
        distances[start_vertex] = 0
        visited = [False] * self.num_vertices

        for _ in range(self.num_vertices):
            min_distance = float("inf")
            min_vertex = -1

            for v in range(self.num_vertices):
                if not visited[v] and distances[v] < min_distance:
                    min_distance = distances[v]
                    min_vertex = v

            visited[min_vertex] = True

            for v in range(self.num_vertices):
                if (
                    not visited[v]
                    and self.graph[min_vertex][v]
                    and distances[min_vertex] != float("inf")
                    and distances[min_vertex] + self.graph[min_vertex][v] < distances[v]
                ):
                    distances[v] = distances[min_vertex] + self.graph[min_vertex][v]

        return distances


class Point:
    """
    Класс, описывающий точку (вектор)
    """

    def __init__(self, x: float = 0, y: float = 0):
        self.x = x
        self.y = y

    def __add__(self, p: typing.Optional["Point"]) -> "Point":
        if p is None:
            return self
        return Point(self.x + p.x, self.y + p.y)

    def __neg__(self) -> "Point":
        return Point(-self.x, -self.y)

    def __sub__(self, p: "Point") -> "Point":
        return self + -p

    def __mul__(self, a: float) -> "Point":
        return Point(self.x * a, self.y * a)

    def __truediv__(self, a: float) -> "Point":
        return self * (1 / a)

    def __pow__(self, a: float) -> "Point":
        return Point(self.x**a, self.y**a)

    def __eq__(self, p: typing.Any) -> bool:
        try:
            return abs(self.x - p.x) < 0.1 and abs(self.y - p.y) < 0.1
        except TypeError:
            return False

    def __str__(self) -> str:
        return f"x = {self.x:.2f}, y = {self.y:.2f}"

    def mag(self) -> float:
        """
        Получить модуль вектора
        """
        return math.hypot(self.x, self.y)

    def arg(self) -> float:
        """
        Получить аргумент вектора (угол относительно оси OX)
        """
        return math.atan2(self.y, self.x)

    def unity(self) -> "Point":
        """
        Получить единичный вектор, коллинеарный данному
        """
        if self.mag() == 0:
            # raise ValueError("БАГА, .unity от нулевого вектора")
            return self
        return self / self.mag()


RIGHT = Point(1, 0)
UP = Point(0, 1)
GRAVEYARD_POS = Point(0, const.GRAVEYARD_POS_X)
FIELD_INF = Point(const.GRAVEYARD_POS_X, 0)


def dist2line(p1: Point, p2: Point, p: Point) -> float:
    """
    Рассчитать расстояние от точки p до прямой, образованной точками p1 и p2
    """
    return abs(vec_mult((p2 - p1).unity(), p - p1))


def segment_poly_intersect(p1: Point, p2: Point, points: list[Point]) -> typing.Optional[Point]:
    """
    Определить, пересекает ли отрезок p1-p2 полигон points
    Если да - возвращает одну из двух точек пересечения
    Если нет - возвращает None
    """
    for i in range(-1, len(points) - 1):
        p = get_line_intersection(p1, p2, points[i], points[i + 1], "SS")
        if p is not None:
            return p
    return None


def is_point_inside_poly(p: Point, points: list[Point]) -> bool:
    """
    Определить, лежит ли точка внутри выпуклого полигона
    """
    old_sign = sign(vec_mult(p - points[-1], points[0] - points[-1]))
    for i in range(len(points) - 1):
        if old_sign != sign(vec_mult(p - points[i], points[i + 1] - points[i])):
            return False
    return True


def dist(a: Point, b: Point) -> float:
    """
    Определить расстояние между двумя точками
    """
    return math.hypot(a.x - b.x, a.y - b.y)


def average_point(points: Sequence[Point]) -> Point:
    """
    Возвращает точку с усредненными координатами
    """
    point = Point(0, 0)
    for p in points:
        point += p
    return point / len(points)


def average_angle(angles: list[float]) -> float:
    """
    Возвращает средний угол
    """
    delta_angle = 0.0
    angle_zero = angles[0]
    for ang in angles:
        delta_angle += wind_down_angle(ang - angle_zero)
    return delta_angle / len(angles) + angle_zero


def get_line_intersection(
    line1_start: Point,
    line1_end: Point,
    line2_start: Point,
    line2_end: Point,
    is_inf: str = "SS",
) -> typing.Optional[Point]:
    """
    Получить точку пересечения отрезков или прямых

    is_inf задает ограничения на точку пересечения. Имеет вид 'AB', параметр A
    задает параметры первой прямой, B - второй.

    S(segment) - задан отрезок
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
    if is_inf[0] == "S" and 0 <= t1 <= 1 or is_inf[0] == "R" and t1 >= 0 or is_inf[0] == "L":
        first_valid = True
    if is_inf[1] == "S" and 0 <= t2 <= 1 or is_inf[1] == "R" and t2 >= 0 or is_inf[1] == "L":
        second_valid = True

    if first_valid and second_valid:
        return p

    return None


def vec_mult(v: Point, u: Point) -> float:
    """
    Посчитать модуль векторного произведения векторов v и u
    """
    return v.x * u.y - v.y * u.x


def scal_mult(v: Point, u: Point) -> float:
    """
    Посчитать скалярное произведение векторов v и u
    """
    return v.x * u.x + v.y * u.y


def rotate(p: Point, angle: float) -> Point:
    """
    Повернуть вектор p на угол angle
    """
    return Point(
        p.x * math.cos(angle) - p.y * math.sin(angle),
        p.y * math.cos(angle) + p.x * math.sin(angle),
    )


def find_nearest_point(p: Point, points: list[Point], exclude: list[Point] = []) -> Point:  #
    """
    Найти ближайшую точку к p из облака points, игнорируя точки exclude
    """
    closest = points[0]
    min_dist = 10e10
    for _, point in enumerate(points):
        if point in exclude:
            continue
        if dist(p, point) < min_dist:
            min_dist = dist(p, point)
            closest = point
    return closest


def wind_down_angle(angle: float) -> float:
    """
    Привести угол к диапазону [-pi, pi]
    """
    angle = angle % (2 * math.pi)
    if angle > math.pi:
        angle -= 2 * math.pi
    return angle


def closest_point_on_line(point1: Point, point2: Point, point: Point, is_inf: str = "S") -> Point:
    """
    Получить ближайшую к точке point току на линии point1-point2

    is_inf задает ограничения на точку пересечения.

    S(segment) - задан отрезок
    R(ay) - задан луч (начало - point1, направление - point2), точка пересечения валидна только
    если находится на луче point1 - point2
    L(ine) - задана прямая
    """
    line_vector = (point2.x - point1.x, point2.y - point1.y)
    line_length = dist(point1, point2)

    if line_length == 0:
        return point1

    line_direction = (line_vector[0] / line_length, line_vector[1] / line_length)

    point_vector = (point.x - point1.x, point.y - point1.y)
    dot_product = point_vector[0] * line_direction[0] + point_vector[1] * line_direction[1]

    if dot_product <= 0 and is_inf != "L":
        return point1
    if dot_product >= line_length and is_inf == "S":
        return point2

    closest_point = Point(
        point1.x + line_direction[0] * dot_product,
        point1.y + line_direction[1] * dot_product,
    )

    return closest_point


def point_on_line(robo: Point, point: Point, distance: float) -> Point:
    """
    Получить точку на линии robot-point,
    отстоящую от точки robot на расстояние distance
    """
    vec_arg = math.atan2(point.y - robo.y, point.x - robo.x)

    # Calculate the new point on the line at the specified distance from the robot
    new_x = robo.x + distance * math.cos(vec_arg)
    new_y = robo.y + distance * math.sin(vec_arg)
    return Point(new_x, new_y)


def lerp(p1: typing.Any, p2: typing.Any, t: float) -> typing.Any:
    """
    Получить линейно интерполированное значение
    """
    return p1 * (1 - t) + p2 * t


def minmax(x: float, a: float, b: Optional[float] = None) -> float:
    """
    Получить ближайшее к x число из диапазона [a, b] или [b, a]
    """
    if b is None:
        b = -a
    a, b = min(a, b), max(a, b)
    return min(max(x, a), b)


def angle_to_point(point1: Point, point2: Point) -> float:
    """
    Получить угол вектора p = point2 - point1
    """
    return (point2 - point1).arg()


def sign(num: float) -> int:
    """
    Получить знак числа num (0 если num == 0)
    """
    if num == 0:
        return 0
    return int(num / abs(num))


def det(a: float, b: float, c: float, d: float) -> float:
    """
    Получить определитель матрицы:
    |a b|
    |c d|
    """
    return a * d - b * c


def nearest_point_on_poly(p: Point, poly: list[Point]) -> Point:
    """
    TODO
    """
    min_ = 10e10
    ans = Point(0, 0)
    for i, _ in enumerate(poly):
        pnt = closest_point_on_line(poly[i - 1], poly[i], p)
        d = dist(pnt, p)
        if d < min_:
            min_ = d
            ans = pnt
    return ans


def in_place(point: Point, end: Point, epsilon: float) -> bool:
    """
    Проверить, находится ли точка st в радиусе epsilon около end
    """
    return (point - end).mag() < epsilon


def circles_inter(p0: Point, p1: Point, r0: float, r1: float) -> tuple[Point, Point]:
    """
    Get intersects of 2 circles:
        p0, r0 - координаты центра и радиус первой окружности
        p1, r1 - координаты центра и радиус второй окружности
    """
    d = dist(p0, p1)
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h = math.sqrt(r0**2 - a**2)
    x2 = p0.x + a * (p1.x - p0.x) / d
    y2 = p0.y + a * (p1.y - p0.y) / d
    x3 = x2 + h * (p1.y - p0.y) / d
    y3 = y2 - h * (p1.x - p0.x) / d
    x4 = x2 - h * (p1.y - p0.y) / d
    y4 = y2 + h * (p1.x - p0.x) / d
    return Point(x3, y3), Point(x4, y4)


def get_tangent_points(point0: Point, point1: Point, r: float) -> list[Point]:
    """
    Get tangents (point0 - center of circle)
    """
    d = dist(point0, point1)
    if d < r:
        return []
    elif d == r:
        return [point1]
    else:
        midx, midy = (point0.x + point1.x) / 2, (point0.y + point1.y) / 2
        p2, p3 = circles_inter(point0, Point(midx, midy), r, d / 2)
        return [p2, p3]


def get_angle_between_points(a: Point, b: Point, c: Point) -> float:
    """
    Возвращает угол между направлениями из b на a и c
    """
    ang = math.atan2(c.y - b.y, c.x - b.x) - math.atan2(a.y - b.y, a.x - b.x)
    return wind_down_angle(ang)


def cosine_theorem(a: float, b: float, angle: float) -> float:
    """Теорема косинусов"""
    return math.sqrt(a * a + b * b - 2 * a * b * math.cos(angle))


def line_circle_intersect(x1: Point, x2: Point, c: Point, radius: float) -> Optional[list[Point]]:
    """TODO"""
    h = closest_point_on_line(x1, x2, c, "L")
    if radius < dist(c, h):
        return None
    elif radius == dist(c, h):
        return [h]

    d = math.sqrt(radius**2 - dist(c, h) ** 2)
    vec = (x2 - x1).unity() * d
    p1 = h + vec
    p2 = h - vec

    c1 = closest_point_on_line(x1, x2, p1)
    c2 = closest_point_on_line(x1, x2, p2)

    if p1 != c1 and p2 != c2:
        return None
    elif p1 != c1:
        return [p2]
    elif p2 != c2:
        return [p1]
    return [p1, p2]


def is_point_inside_circle(a: Point, c: Point, radius: float) -> bool:
    """Return TRUE if point inside circle"""
    return dist(a, c) < radius


def nearest_point_on_circle(a: Point, c: Point, radius: float) -> Point:
    """Return nearest point in circle"""
    return c + (a - c).unity() * radius


def is_point_on_line(point: Point, line_start: Point, line_end: Point, is_inf: str = "L") -> bool:
    return dist(point, closest_point_on_line(line_start, line_end, point, is_inf)) < 0.1
