import math
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt

OBSTACLE_ANGLE = math.pi / 60
GOAL_VIEW_ANGLE = math.pi / 40
GOAL_HULL_DIST = 100.0
SHOOT_ANGLE = math.pi / 80


def estimate_pass_point(
    enemies: list[aux.Point],
    frm: aux.Point,
    to: aux.Point,
) -> float:
    """
    Оценивает пас из точки "frm" в точку "to", возвращая положительное значение до 1
    """
    lerp: float = 0.0

    for enemy in enemies:
        frm_enemy = aux.dist(frm, enemy)
        if frm_enemy > const.ROBOT_R:
            if frm_enemy <= aux.dist(frm, to):
                tgs = aux.get_tangent_points(enemy, frm, const.ROBOT_R)
                if len(tgs) < 2:
                    continue

                ang1 = aux.get_angle_between_points(to, frm, tgs[0])
                ang2 = aux.get_angle_between_points(to, frm, tgs[1])

                ang = min(abs(ang1), abs(ang2))
                if (
                    ang1 * ang2 < 0
                    and abs(ang1) < math.pi / 2
                    and abs(ang2) < math.pi / 2
                ):
                    ang *= -1  # enemy between to and frm
            else:  # circle around enemy
                enemy_to = aux.dist(enemy, to)

                enemy_angle = math.asin(const.ROBOT_R / frm_enemy)
                to_enemy_angle = 2 * math.asin((enemy_to / 2) / frm_enemy)
                ang = to_enemy_angle - enemy_angle

            if ang < OBSTACLE_ANGLE:
                delta_lerp = abs((OBSTACLE_ANGLE - ang) / OBSTACLE_ANGLE) ** 1.5

                lerp += delta_lerp

    return lerp  # 0 - perfect; bigger => worse


def estimate_goal_view(point: aux.Point, field: fld.Field) -> float:
    goal_angle = abs(
        aux.get_angle_between_points(field.enemy_goal.up, point, field.enemy_goal.down)
    )

    return min(goal_angle / GOAL_VIEW_ANGLE, 1)  # 1 - perfect; smaller => worse


def estimate_dist_to_goal(point: aux.Point, field: fld.Field) -> float:
    dist_to_goal_zone = aux.dist(
        point, aux.nearest_point_on_poly(point, field.enemy_goal.hull)
    )
    if aux.is_point_inside_poly(point, field.enemy_goal.hull):
        dist_to_goal_zone *= -1

    return max(1 - dist_to_goal_zone / GOAL_HULL_DIST, 0)


def estimate_shoot(
    point: aux.Point, field: fld.Field, enemies: list[aux.Point]
) -> float:
    lerp: float = 0.0

    for enemy in enemies:
        frm_enemy = aux.dist(point, enemy)
        if frm_enemy > const.ROBOT_R:

            ang1 = aux.get_angle_between_points(field.enemy_goal.down, point, enemy)
            ang2 = aux.get_angle_between_points(field.enemy_goal.up, point, enemy)

            ang = min(abs(ang1), abs(ang2))
            if ang1 * ang2 < 0 and abs(ang1) < math.pi / 2 and abs(ang2) < math.pi / 2:
                ang *= -1  # enemy between to and frm

            if ang < SHOOT_ANGLE:
                delta_lerp = abs((SHOOT_ANGLE - ang) / SHOOT_ANGLE) ** 1.5

                lerp += delta_lerp

    return lerp  # 0 - perfect; bigger => worse


def estimate_point(
    point: aux.Point, kick_point: aux.Point, field: fld.Field, enemies: list[aux.Point]
) -> float:
    lerp1 = estimate_pass_point(enemies, kick_point, point)
    lerp2 = estimate_goal_view(point, field)
    lerp3 = estimate_dist_to_goal(point, field)
    lerp4 = estimate_shoot(point, field, enemies)

    lerp = lerp2 - lerp1 - lerp3 - lerp4  # lerp2 - lerp1 - lerp3 - lerp4
    return lerp  # 1 - perfect; smaller => worse


def choose_segment_in_goal(
    field: fld.Field,
    kicker_id: int,
    goal: fld.Goal,
    ball_pos: aux.Point,
    interfering_robots: list[rbt.Robot],
) -> tuple[aux.Point, aux.Point]:
    """
    Выбирает самый большой угловой промежуток на воротах (если смотреть из точки ball_pos)
    """
    positions = []
    for robot in interfering_robots:
        if robot != field.allies[kicker_id]:
            if (
                aux.dist(robot.get_pos(), goal.center) < aux.dist(goal.center, ball_pos)
                and robot.is_used()
            ):
                positions.append(robot.get_pos())

    positions = sorted(positions, key=lambda x: x.y * -goal.eye_up.y)

    segments = [goal.up]
    for p in positions:
        tangents = aux.get_tangent_points(p, ball_pos, const.ROBOT_R)
        if len(tangents) < 2:
            continue

        int1 = aux.get_line_intersection(
            ball_pos,
            tangents[0],
            goal.down,
            goal.up,
            "RS",
        )
        int2 = aux.get_line_intersection(
            ball_pos,
            tangents[1],
            goal.down,
            goal.up,
            "RS",
        )

        if int1 is None and int2 is not None:
            segments.append(goal.up)
            segments.append(int2)
        elif int1 is not None and int2 is None:
            segments.append(int1)
            segments.append(goal.down)
        elif int1 is not None and int2 is not None:
            segments.append(int1)
            segments.append(int2)

    segments.append(goal.down)
    max_ = 0.0
    maxId = -1
    for i in range(0, len(segments), 2):
        c = segments[i]
        a = segments[i + 1]
        b = ball_pos
        if (c.y - a.y) * goal.eye_up.y < 0:
            continue  # Shadow intersection
        ang = abs(
            aux.get_angle_between_points(a, b, c)
        )  # Саша, я тут градусы на радианы заменил, надеюсь ничего не сломалось
        if ang > max_:
            max_ = ang
            maxId = i

    if maxId == -1:
        return (goal.down, goal.up)

    return (segments[maxId], segments[maxId + 1])


def choose_kick_point(
    field: fld.Field,
    kicker_id: int,
    goal: Optional[fld.Goal] = None,
    ball_pos: Optional[aux.Point] = None,
    interfering_robots: Optional[list[rbt.Robot]] = None,
) -> aux.Point:
    """
    Выбирает оптимальную точку в воротах для удара
    """
    if goal is None:
        goal = field.enemy_goal
    if ball_pos is None:
        ball_pos = field.ball.get_pos()
    if interfering_robots is None:
        interfering_robots = field.enemies
        if const.SELF_PLAY:
            interfering_robots = field.allies

    A, C = choose_segment_in_goal(field, kicker_id, goal, ball_pos, interfering_robots)
    B = ball_pos

    tmp1 = (C - B).mag()
    tmp2 = (A - B).mag()
    CA = A - C
    pnt = A
    if tmp2 != 0:
        pnt = C + CA * 0.5 * (tmp1 / tmp2)

    return pnt
