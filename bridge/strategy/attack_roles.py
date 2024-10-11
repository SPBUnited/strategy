from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt
from bridge.router import waypoint as wp
from bridge.strategy import accessories as acc
from bridge.strategy import kicker

kick = kicker.KickerAux()


def attacker(
    field: fld.Field,
    waypoints: list[wp.Waypoint],
    attacker_id: int,
    pass_points: list[tuple[aux.Point, float]],
    forwards: list[rbt.Robot],
) -> None:
    """Логика действий для робота с мячом"""
    attacker = field.allies[attacker_id]
    kick_point = acc.choose_kick_point(field, attacker_id)
    kick_est = acc.estimate_pass_point(
        [e.get_pos() for e in field.active_enemies()] + [field.enemies[field.enemy_gk_id].get_pos()],
        field.ball.get_pos(),
        kick_point,
    )
    print("Kick est: ", kick_est)
    field.strategy_image.draw_dot(kick_point, (255, 0, 0), 15)

    if aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.hull) and not field.is_ball_moves_to_goal():
        waypoints[attacker_id] = wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_ENDPOINT)
        return

    # if self.pass_or_kick_decision_border.is_lower(kick_est) and len(forwards) > 0:
    if kick_est > 0.5 and len(pass_points) > 0 and len(forwards) > 0:
        pass_point = pass_points[0]
        receiver = fld.find_nearest_robot(pass_point[0], forwards)
        # print(pass_est, kick_est)
        if pass_point is not None and pass_point[1] > kick_est and receiver is not None:
            waypoints[attacker_id] = pass_kicker(field, attacker_id, receiver.r_id)
            if waypoints[attacker_id].type == wp.WType.S_BALL_KICK:
                enemy_near = fld.find_nearest_robot(attacker.get_pos(), field.enemies)
                if enemy_near is not None:
                    enemy_point = aux.closest_point_on_line(
                        attacker.get_pos(),
                        attacker.get_pos() + aux.rotate(aux.RIGHT, attacker.get_angle()) * const.ROBOT_R * 5,
                        enemy_near.get_pos(),
                        "S",
                    )
                    if aux.dist(enemy_near.get_pos(), enemy_point) < const.ROBOT_R:
                        waypoints[attacker_id].type = wp.WType.S_BALL_KICK_UP
            # print("attacker: pass to", receiver_id)
            return

    waypoints[attacker_id] = kick.shoot_to_goal(field, attacker, kick_point)
    # print("attacker: shoot to goal")


def choose_receiver(field: fld.Field, forwards: list[rbt.Robot]) -> tuple[Optional[int], Optional[float]]:
    """Выбирает робота для получения паса"""
    receiver_id = None
    receiver_score = None
    for forward in forwards:
        pass_score = acc.estimate_point(field, field.ball.get_pos(), forward.get_pos())

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

    if field.ally_color != const.COLOR:
        k = -1 if const.SELF_PLAY else 1
        # poses = [
        #     aux.Point(-1500 * field.polarity * k, 1250),
        #     aux.Point(-1500 * field.polarity * k, -1250),
        #     aux.Point(-1000 * field.polarity * k, 0),
        # ]
        pass_points = [
            (aux.Point(-3500 * field.polarity * k, 1250), 0),
            (aux.Point(-3500 * field.polarity * k, -1250), 0),
            (aux.Point(-3000 * field.polarity * k, 0), 0),
        ]
        pass_points = pass_points[: (pos_num + 1)]

    used_forwards: list[int] = []

    for pos in pass_points:
        print(pos[0])
        if len(used_forwards) == pos_num:
            return
        pop = fld.find_nearest_robot(pos[0], forwards, used_forwards)
        if pop is None:
            continue
        used_forwards.append(pop.r_id)
        pass_receiver(field, waypoints, pop.r_id, pos[0])


def pass_kicker(field: fld.Field, kicker_id: int, receiver_id: int) -> wp.Waypoint:
    """
    Отдает пас от робота kicker_id роботу receiver_id
    Должна вызываться в конечном автомате постоянно, пока первый робот не даст пас
    """
    receiver = field.allies[receiver_id]
    if not field.is_ball_moves_to_point(receiver.get_pos()):
        # waypoints[kicker_id] = wp.Waypoint(
        #     field.ball.get_pos(),
        #     aux.angle_to_point(field.ball.get_pos(), receiver.get_pos()),
        #     wp.WType.S_BALL_PASS,
        # )
        # print("pass to", receiver_id)
        waypoint = kick.pass_to_point(field, field.allies[kicker_id], receiver.get_pos())
        field.strategy_image.draw_dot(
            field.ball.get_pos()
            + aux.rotate(
                aux.RIGHT,
                aux.angle_to_point(field.ball.get_pos(), receiver.get_pos()),
            ),
            (255, 0, 255),
            5,
        )
    else:
        waypoint = wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_STOP)

    return waypoint


def pass_receiver(
    field: fld.Field,
    waypoints: list[wp.Waypoint],
    receiver_id: int,
    receive_point: aux.Point,
) -> None:
    """
    Отдает пас от робота kicker_id роботу receiver_id
    Должна вызываться в конечном автомате постоянно, пока второй робот не поймает мяч
    TODO: прописать действия отдающего пас робота после удара и принимающего пас робота до удара
    """
    receiver = field.allies[receiver_id]
    if field.is_ball_moves_to_point(receiver.get_pos()) and field.ball_start_point is not None:
        target = aux.closest_point_on_line(field.ball_start_point, field.ball.get_pos(), receiver.get_pos(), "R")
        field.strategy_image.draw_line(target, receiver.get_pos(), (255, 127, 0), 2)
        field.strategy_image.draw_dot(target, (128, 128, 255), const.ROBOT_R)

        receiver.set_dribbler_speed(15)

        waypoints[receiver_id] = wp.Waypoint(
            target,
            aux.angle_to_point(field.ball.get_pos(), field.ball_start_point),
            wp.WType.S_ENDPOINT,
        )
    else:
        waypoints[receiver_id] = wp.Waypoint(
            receive_point,
            aux.angle_to_point(receiver.get_pos(), field.ball.get_pos()),
            wp.WType.S_ENDPOINT,
        )
        field.strategy_image.draw_dot(receive_point, (255, 255, 0), 5)
