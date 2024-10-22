"""Control for attack roles"""

from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt
from bridge.router import kicker
from bridge.router import waypoint as wp
from bridge.strategy import accessories as acc


def attacker(
    field: fld.Field,
    waypoints: list[wp.Waypoint],
    attacker_id: int,
    pass_points: list[tuple[aux.Point, float]],
    forwards: list[rbt.Robot],
) -> None:
    """Логика действий для робота с мячом"""
    atk = field.allies[attacker_id]
    kick_point = acc.choose_kick_point(field, attacker_id)
    kick_est = acc.estimate_pass_point(
        [e.get_pos() for e in field.active_enemies()]
        + [field.enemies[field.enemy_gk_id].get_pos()],
        field.ball.get_pos(),
        kick_point,
    )
    # print("Kick est: ", kick_est)
    field.strategy_image.draw_dot(kick_point, (255, 0, 0), 15)

    if (
        aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.hull)
        and not field.is_ball_moves_to_goal()
    ):
        waypoints[attacker_id] = wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_ENDPOINT)
        return

    # if self.pass_or_kick_decision_border.is_lower(kick_est) and len(forwards) > 0:
    if kick_est > 1 and len(forwards) > 0:
        receiver_id, pass_est = choose_receiver(field, forwards)

        if receiver_id is not None and pass_est is not None:
            waypoints[attacker_id] = pass_kicker(field, attacker_id, receiver_id)
            return

    waypoints[attacker_id] = kicker.shoot_to_goal(field, atk, kick_point)
    # print("attacker: shoot to goal")


def choose_receiver(
    field: fld.Field, forwards: list[rbt.Robot]
) -> tuple[Optional[int], Optional[float]]:
    """Выбирает робота для получения паса"""
    receiver_id = None
    receiver_score = None
    for forward in forwards:
        pass_score = acc.estimate_point(field, forward.get_pos(), field.ball.get_pos())
        if receiver_id is None or pass_score > receiver_score:
            receiver_id = forward.r_id
            receiver_score = pass_score
    return receiver_id, pass_score


def set_forwards_wps(
    field: fld.Field,
    waypoints: list[wp.Waypoint],
    forwards: list[rbt.Robot],
    pass_points: list[tuple[aux.Point, float]],
) -> None:
    """Расставляет роботов по точкам для получения паса"""
    pos_num = len(forwards)
    poses = pass_points.copy()

    if field.ally_color != const.COLOR:
        k = -1 if const.SELF_PLAY else 1
        # poses = [
        #     aux.Point(-1500 * field.polarity * k, 1250),
        #     aux.Point(-1500 * field.polarity * k, -1250),
        #     aux.Point(-1000 * field.polarity * k, 0),
        # ]
        poses = [
            (aux.Point(-3500 * field.polarity * k, 1250), 1),
            (aux.Point(-3500 * field.polarity * k, -1250), 1),
            (aux.Point(-3000 * field.polarity * k, 0), 1),
        ]
        poses = poses[: (pos_num + 1)]

    used_poses: list[aux.Point] = []

    for forward in forwards:
        if len(poses) - len(used_poses) == 0:
            continue
        best_pos: Optional[aux.Point] = None
        best_pos_est: Optional[float] = None
        for pos in poses:  # TODO сделать зависимость от времени до паса
            if pos[0] in used_poses:
                continue
            est = (
                pos[1] - aux.dist(pos[0], forward.get_pos()) / 1000
            )  # <- coefficient of fear to move to far point
            if best_pos_est is None or est > best_pos_est:
                best_pos = pos[0]
                best_pos_est = pos[1]

        if best_pos is not None:
            used_poses.append(best_pos)
            pass_receiver(field, waypoints, forward.r_id, best_pos)


def pass_kicker(field: fld.Field, kicker_id: int, receiver_id: int) -> wp.Waypoint:
    """
    Отдает пас от робота kicker_id роботу receiver_id
    Должна вызываться в конечном автомате постоянно, пока первый робот не даст пас
    """
    receiver = field.allies[receiver_id]
    # if not field.is_ball_moves_to_point(receiver.get_pos()):
    waypoint = kicker.pass_to_point(field, field.allies[kicker_id], receiver.get_pos())
    # else: #TODO bad case
    #     waypoint = wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_STOP)

    # тут нужно пользоваться пониманием до куда мяч катится
    # если мяч нормально летит, то забираем роль атакующего

    return waypoint


def pass_receiver(
    field: fld.Field,
    waypoints: list[wp.Waypoint],
    receiver_id: int,
    receive_point: aux.Point,
) -> None:
    """
    Ловит мяч
    """
    receiver = field.allies[receiver_id]

    waypoints[receiver_id] = wp.Waypoint(
        receive_point,
        aux.angle_to_point(receiver.get_pos(), field.ball.get_pos()),
        wp.WType.S_ENDPOINT,
    )
    field.strategy_image.draw_dot(receive_point, (255, 255, 0), 5)


def set_pass_receivers_wps(
    field: fld.Field,
    waypoints: list[wp.Waypoint],
    receivers: list[rbt.Robot],
) -> None:
    """Catch the ball for all receivers"""
    for receiver in receivers:
        target = aux.closest_point_on_line(
            field.ball_start_point, field.ball.get_pos(), receiver.get_pos(), "R"
        )
        field.strategy_image.draw_line(target, receiver.get_pos(), (255, 127, 0), 2)
        field.strategy_image.draw_dot(target, (128, 128, 255), const.ROBOT_R)

        waypoints[receiver.r_id] = wp.Waypoint(
            target,
            aux.angle_to_point(field.ball.get_pos(), field.ball_start_point),
            wp.WType.S_CATCH_BALL,
        )
