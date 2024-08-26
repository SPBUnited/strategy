import math
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt
from bridge.router import waypoint as wp
from bridge.strategy import accessories as acc


def set_wall_enemy(field: fld.Field) -> aux.Point:
    ball = field.ball.get_pos()
    if aux.is_point_inside_poly(ball, field.ally_goal.hull):
        return fld.find_nearest_robot(field.ally_goal.center, field.enemies).get_pos()

    # if not field.is_ball_moves():
    #     return fld.find_nearest_robot(ball, field.enemies).get_pos()

    if field.is_ball_moves() and not field.is_ball_moves_to_goal():
        sorted_enemies = fld.find_nearest_robots(field.ally_goal.center, field.enemies)

        for robot in sorted_enemies:
            if field.is_ball_moves_to_point(robot.get_pos()):
                return robot.get_pos()

    return ball


def calc_wall_pos(field: fld.Field, ball: aux.Point) -> aux.Point:
    """Рассчитывает точку, по которой будут выбираться роботы для стенки"""
    inter = aux.segment_poly_intersect(field.ally_goal.center, ball, field.ally_goal.hull)
    if inter:
        return inter
    return field.ally_goal.frw


def set_wall_targets(field: fld.Field, num: int, ball: aux.Point) -> list[aux.Point]:
    """Рассчитывает точки для стенки из num роботов"""
    if aux.is_point_inside_poly(ball, field.ally_goal.big_hull):
        ball = fld.find_nearest_robot(field.ally_goal.center, field.enemies).get_pos()
    poses = []

    intersections: list[Optional[aux.Point]] = []

    intersections.append(aux.segment_poly_intersect(ball, field.ally_goal.down, field.ally_goal.big_hull))
    intersections.append(aux.segment_poly_intersect(ball, field.ally_goal.up, field.ally_goal.big_hull))

    if intersections[0] is None or intersections[1] is None:
        return [
            field.ally_goal.frw + field.ally_goal.eye_up * const.ROBOT_R * (-(num - 1) + 2 * i) for i in range(num)
        ]  # ball in goal.hull
    elif aux.dist(ball, intersections[0]) > aux.dist(ball, intersections[1]):
        intersections[1], intersections[0] = intersections[0], intersections[1]

    length = min((ball - intersections[0]).mag(), (ball - intersections[1]).mag())
    intersections[0] = ball + (intersections[0] - ball).unity() * length
    intersections[1] = ball + (intersections[1] - ball).unity() * length
    wall_vec = intersections[1] - intersections[0]
    if field.is_ball_moves_to_goal() and field.ball_start_point is not None:
        wall_middle = aux.get_line_intersection(
            intersections[0],
            intersections[1],
            field.ball_start_point,
            field.ball.get_pos(),
            "SR",
        )
        if wall_middle is not None:
            for i in range(num):
                delta = const.ROBOT_R * (-(num - 1) + 2 * i)
                poses.append(wall_middle + wall_vec.unity() * delta)
                field.strategy_image.draw_dot(
                    wall_middle + wall_vec.unity() * delta,
                    (200, 200, 200),
                    const.ROBOT_R * 0.9,
                )
            return poses

    for point in field.ally_goal.big_hull:
        inter = aux.get_line_intersection(ball, point, intersections[0], intersections[1], "RS")
        if inter is None:
            continue
        if aux.is_point_inside_poly(point, [ball, intersections[0], intersections[1]]):
            scale = aux.dist(ball, point) / aux.dist(ball, inter)
            intersections[0] = aux.lerp(ball, intersections[0], scale)
            intersections[1] = aux.lerp(ball, intersections[1], scale)

    if wall_vec.mag() > num * 2 * const.ROBOT_R:
        for i in range(num):
            delta = const.ROBOT_R * (2 * i + 1)
            poses.append(intersections[0] + wall_vec.unity() * delta)
            field.strategy_image.draw_dot(
                intersections[0] + wall_vec.unity() * delta,
                (128, 128, 128),
                const.ROBOT_R,
            )
    else:
        wall_middle = intersections[0] + wall_vec / 2
        for i in range(num):
            delta = const.ROBOT_R * (-(num - 1) + 2 * i)
            poses.append(wall_middle + wall_vec.unity() * delta)
            field.strategy_image.draw_dot(
                wall_middle + wall_vec.unity() * delta,
                (200, 200, 200),
                const.ROBOT_R,
            )

    return poses


def set_wallliners_wps(
    field: fld.Field,
    waypoints: list[wp.Waypoint],
    wallliners: list[rbt.Robot],
    enemy_pos: aux.Point,
) -> None:
    """Создает путевые точки для роботов в стенке"""

    wall_pos = calc_wall_pos(field, set_wall_enemy(field))

    active_wallers: list[rbt.Robot] = []
    for liner in wallliners:
        if aux.dist(liner.get_pos(), wall_pos) < 500:  # less when 5 robots in wall!!
            active_wallers.append(liner)

    poses = set_wall_targets(field, len(active_wallers), enemy_pos)

    projections: list[tuple[float, int]] = []
    for _, waller in enumerate(active_wallers):
        point = aux.closest_point_on_line(poses[0], poses[len(poses) - 1], waller.get_pos(), "L")
        projections.append((point.y, waller.r_id))
    projections = sorted(projections, key=lambda x: x[0])
    poses = sorted(poses, key=lambda x: x.y)

    wp_type = wp.WType.R_IGNORE_GOAl_HULL
    if aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.big_hull):
        wp_type = wp.WType.S_ENDPOINT

    for ally in wallliners:
        angle = (field.ball.get_pos() - wall_pos).arg()
        waypoints[ally.r_id] = wp.Waypoint(wall_pos, angle, wp_type)

    for i, pos in enumerate(poses):
        idx = projections[i][1]
        angle = (field.ball.get_pos() - field.ally_goal.center).arg()
        waypoints[idx] = wp.Waypoint(pos, angle, wp_type)


def get_enemies_near_goal(field: fld.Field) -> list[aux.Point]:
    enemies_near_goal: list[aux.Point] = []
    enemy_dist: list[tuple[aux.Point, float]] = []

    enemy_with_ball = fld.find_nearest_robot(field.ball.get_pos(), field.enemies)
    for enemy in field.enemies:
        if aux.dist(enemy.get_pos(), field.ally_goal.center) < const.GOAL_DX and enemy != enemy_with_ball:
            dist_to_ally = aux.dist(
                enemy.get_pos(),
                fld.find_nearest_robot(enemy.get_pos(), field.allies).get_pos(),
            )
            enemy_dist.append((enemy.get_pos(), dist_to_ally))

    enemy_dist = sorted(enemy_dist, key=lambda x: x[1])
    for en_d in enemy_dist:
        enemies_near_goal.append(en_d[0])

    return enemies_near_goal


def set_pass_defenders_wps(
    field: fld.Field,
    waypoints: list[wp.Waypoint],
    pass_defenders: list[rbt.Robot],
    enemies_near_goal: list[aux.Point],
) -> None:
    """Создает путевые точки для роботов в стенке"""
    defeated_enemies: list[aux.Point] = []
    ball = field.ball.get_pos()
    for defender in pass_defenders:
        enemy = aux.find_nearest_point(defender.get_pos(), enemies_near_goal, defeated_enemies)
        defeated_enemies.append(enemy)
        if field.is_ball_moves_to_point(defender.get_pos()) and field.ball_start_point is not None:
            target = aux.closest_point_on_line(field.ball_start_point, ball, defender.get_pos(), "R")
            if (enemy.y - target.y) * (target.y - ball.y) < 0:  # target is not between enemy and ball
                enemy_target = aux.closest_point_on_line(field.ball_start_point, ball, enemy, "R")
                delta_vec = (ball - enemy_target).unity() * const.ROBOT_R * 3
                target = enemy + delta_vec
            angle = (ball - target).arg()
            waypoints[defender.r_id] = wp.Waypoint(target, angle, wp.WType.S_ENDPOINT)
            field.strategy_image.draw_dot(target, (100, 200, 255), 55)
            continue

        point_to_close = aux.closest_point_on_line(ball, enemy, defender.get_pos())
        if aux.dist(point_to_close, defender.get_pos()) > const.ROBOT_R:
            delta_vec = (ball - enemy).unity() * aux.dist(point_to_close, enemy) * 0.8
        else:
            delta_vec = (ball - enemy).unity() * const.ROBOT_R * 3

        target = enemy + delta_vec

        field.strategy_image.draw_dot(target, (100, 200, 255), 25)
        angle = (ball - target).arg()
        waypoints[defender.r_id] = wp.Waypoint(target, angle, wp.WType.S_ENDPOINT)


def goalk(
    field: fld.Field,
    wallliners: list[rbt.Robot],
    robot_with_ball: Optional[rbt.Robot],
) -> wp.Waypoint:
    """
    Управление вратарём и стенкой
    Возвращает массив с элементами класса Waypoint, где 0й элемент - вратарь, а остальные - стенка
    """
    gk_pos: aux.Point | None = None
    gk_angle: float
    if field.gk_id < 9:
        gk_angle = field.ally_goal.eye_forw.arg()
    else:
        gk_angle = math.pi / 2
    ball = field.ball.get_pos()
    goal_down = field.ally_goal.down
    goal_up = field.ally_goal.up

    if abs(ball.x) > const.GOAL_DX:  # NOTE
        return wp.Waypoint(field.ally_goal.center, gk_angle, wp.WType.S_IGNOREOBSTACLES)

    if field.is_ball_stop_near_goal() or field.robot_with_ball == field.allies[field.gk_id]:
        return wp.Waypoint(ball, field.ally_goal.eye_forw.arg(), wp.WType.S_BALL_KICK_UP)

    if (
        field.is_ball_moves_to_goal()
        and field.ball_start_point is not None
        and (field.ball_start_point - ball).mag() > const.INTERCEPT_SPEED
    ):
        tmp_pos = aux.get_line_intersection(
            field.ball_start_point,
            ball,
            field.ally_goal.down,
            field.ally_goal.up,
            "RS",
        )
        if tmp_pos is not None:
            gk_pos = aux.closest_point_on_line(ball, tmp_pos, field.allies[field.gk_id].get_pos())
            return wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)
        else:
            tmp_pos = aux.get_line_intersection(
                field.ball_start_point,
                ball,
                field.ally_goal.center_down,
                field.ally_goal.center_up,
                "RS",
            )
            if tmp_pos is not None:
                poses = [
                    goal_up + (field.ally_goal.eye_forw - field.ally_goal.eye_up) * const.ROBOT_R * 0.8,
                    goal_down + (field.ally_goal.eye_forw + field.ally_goal.eye_up) * const.ROBOT_R * 0.8,
                ]
                gk_pos = aux.find_nearest_point(tmp_pos, poses)
                return wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)

    if len(wallliners) != 0:
        segment = acc.choose_segment_in_goal(field, -1, field.ally_goal, ball, wallliners)
        goal_down, goal_up = segment

    if robot_with_ball is not None:
        predict = aux.get_line_intersection(
            robot_with_ball.get_pos(),
            robot_with_ball.get_pos() + aux.rotate(aux.RIGHT, robot_with_ball.get_angle()),
            goal_down,
            goal_up,
            "RS",
        )
        if predict is not None:
            p_ball = (ball - predict).unity()
            gk_pos = aux.lerp(
                aux.point_on_line(field.ally_goal.center, ball, const.GK_FORW),
                p_ball * const.GK_FORW
                + aux.get_line_intersection(
                    robot_with_ball.get_pos(),
                    robot_with_ball.get_pos() + aux.rotate(aux.RIGHT, robot_with_ball.get_angle()),
                    goal_down,
                    goal_up,
                    "RS",
                ),
                0.5,
            )
            if gk_pos is not None:
                return wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)

    tmp1 = (goal_up - ball).mag()
    tmp2 = (goal_down - ball).mag()
    seg = goal_down - goal_up
    if tmp2 != 0:
        kick_point = goal_up + seg * 0.5 * (tmp1 / tmp2)
    else:
        kick_point = goal_down

    radius = (const.GK_FORW**2 + (const.GOAL_DY / 2) ** 2) / 2 / const.GK_FORW
    circle_center = field.ally_goal.center - field.ally_goal.eye_forw * (radius - const.GK_FORW - const.ROBOT_R)
    intersects = aux.line_circle_intersect(kick_point, ball, circle_center, radius)

    if intersects is None:
        gk_pos = field.ally_goal.center + field.ally_goal.eye_forw * const.ROBOT_R
    else:
        gk_pos = intersects[0]

    gk_pos.x = aux.minmax(gk_pos.x, const.GOAL_DX - const.ROBOT_R)

    return wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)
