import math
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt


def estimate_pass_point(field: fld.Field, frm: Optional[aux.Point], to: Optional[aux.Point]) -> float:
    """
    Оценивает пас из точки "frm" в точку "to, возвращая положительное значение до 0.8
    """
    if frm is None or to is None:
        return 0
    positions: list[tuple[int, aux.Point]] = []

    poses = field.enemies if not const.SELF_PLAY else field.allies
    for robot in poses:
        if robot.is_used() and robot.get_pos() != to:
            positions.append((robot.r_id, robot.get_pos()))
    positions = sorted(positions, key=lambda x: x[1].y)

    tangents: list[tuple[int, list[aux.Point]]] = []
    for p in positions:
        tgs = aux.get_tangent_points(p[1], frm, const.ROBOT_R)
        if tgs is None or len(tgs) < 2:
            continue
        tangents.append((p[0], tgs))

    min_ = 10e3

    shadows_bots = []
    for tangent in tangents:
        ang1 = aux.get_angle_between_points(to, frm, tangent[1][0])
        ang2 = aux.get_angle_between_points(to, frm, tangent[1][1])

        if ang1 * ang2 < 0 and abs(ang1) < math.pi / 2 and abs(ang2) < math.pi / 2:
            shadows_bots.append(tangent[0])
        ang1 = abs(ang1)
        ang2 = abs(ang2)
        # if ang1 > 180:
        #     ang1 = 360 - ang1
        # if ang2 > 180:
        #     ang2 = 360 - ang2

        if ang1 < min_:
            min_ = ang1
        if ang2 < min_:
            min_ = ang2
    # if minId == -1:
    #     return 0

    if min_ == 10e3 or len(shadows_bots) != 0:
        return 0
    dist = (frm - to).mag() / 1000
    max_ang = abs(aux.wind_down_angle(2 * math.atan2(const.ROBOT_SPEED, -0.25 * dist + 4.5)))
    # max_ang = 10
    return min(abs(min_ / max_ang), 1)


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
            if aux.dist(robot.get_pos(), goal.center) < aux.dist(goal.center, ball_pos) and robot.is_used():
                positions.append(robot.get_pos())

    positions = sorted(positions, key=lambda x: x.y * -goal.eye_up.y)

    segments = [goal.up]
    for p in positions:
        tangents = aux.get_tangent_points(p, ball_pos, const.ROBOT_R)
        if tangents is None or len(tangents) < 2:
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
