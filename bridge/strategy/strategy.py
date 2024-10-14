"""High-level strategy code"""


import math

# !v DEBUG ONLY
from enum import Enum
from time import time
from typing import Optional

import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, fld, rbt
from bridge.processors.referee_state_processor import State as GameStates
from bridge.strategy import attack_roles, defense_roles, kicker, ref_states


class Role(Enum):
    """Class with robot's roles"""

    GOALKEEPER = 0
    ATTACKER = 1

    PASS_DEFENDER = 2
    WALLLINER = 3

    FORWARD = 11
    PASS_RECEIVER = 12

    UNAVAILABLE = 100


class FloatingBorder:
    """Floating border for dangerous borders in conditions"""

    def __init__(self, middle: float, moving: float = 0.1) -> None:
        self.low = middle * (1 - moving)
        self.high = middle * (1 + moving)

        self.middle = middle
        self.border = middle

    def reset(self) -> None:
        """Reset"""
        self.border = self.middle

    def is_higher(self, val: float) -> bool:
        """Return True if higher, return False if lower"""
        if val > self.border:
            self.border = self.low
            return True

        self.border = self.high
        return False

    def is_lower(self, val: float) -> bool:
        """Return False if higher, return True if lower"""
        return not self.is_higher(val)


class Strategy:
    """Main class of strategy"""

    def __init__(
        self,
        dbg_game_status: GameStates = GameStates.RUN,
    ) -> None:

        self.game_status = dbg_game_status
        self.active_team: const.Color = const.Color.ALL
        self.we_active = False
        self.timer = time()

        self.forwards: list[rbt.Robot] = []
        self.prev_roles: list[Role] = [Role.UNAVAILABLE for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]

        self.kick = kicker.KickerAux()

        self.pass_or_kick_decision_border = FloatingBorder(0.2, 0.3)

        self.flag = 0

        self.pass_points: list[tuple[aux.Point, float]] = []

    def change_game_state(self, new_state: GameStates, upd_active_team: const.Color) -> None:
        """Change game state and active team's color"""
        self.game_status = new_state
        self.active_team = upd_active_team

    def process(self, field: fld.Field) -> list[wp.Waypoint]:
        """Game State Management"""
        if self.game_status not in [GameStates.KICKOFF, GameStates.PENALTY]:
            if self.active_team in [const.Color.ALL, field.ally_color]:
                self.we_active = True
            else:
                self.we_active = False

        waypoints: list[wp.Waypoint] = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoints.append(
                wp.Waypoint(
                    field.allies[i].get_pos(),
                    field.allies[i].get_angle(),
                    wp.WType.S_STOP,
                )
            )

        # self.game_status = GameStates.RUN
        if field.ally_color == const.COLOR:
            print("-" * 32)
            print(self.game_status, "\twe_active:", self.we_active)

        # self.debug(field, waypoints)
        # return waypoints

        match self.game_status:
            case GameStates.RUN:
                self.run(field, waypoints)
            case GameStates.TIMEOUT:
                ref_states.timeout(field, waypoints)
            case GameStates.HALT:
                pass
                # self.halt(field, waypoints)
            case GameStates.PREPARE_PENALTY:
                ref_states.prepare_penalty(field, waypoints, self.we_active)
            case GameStates.PENALTY:
                if self.we_active:
                    ref_states.penalty_kick(field, waypoints)
                else:
                    waypoints[field.gk_id] = defense_roles.goalk(field, [])
            case GameStates.PREPARE_KICKOFF:
                ref_states.prepare_kickoff(field, waypoints, self.we_active)
            case GameStates.KICKOFF:
                ref_states.kickoff(field, waypoints, self.we_active)
            case GameStates.FREE_KICK:
                self.run(field, waypoints)
            case GameStates.STOP:
                self.run(field, waypoints)

        # self.debug(field, waypoints)  # NOTE

        return waypoints

    def run(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """
        Assigning roles to robots and managing them
            roles - robot roles sorted by priority
            robot_roles - list of robot id and role matches
        """

        # Calculating endpoints that are not affected by robot selection
        wall_pos = defense_roles.calc_wall_pos(field)
        enemies_near_goal = defense_roles.get_enemies_near_goal(field)

        # Selecting roles based on calculated points
        roles = manage_roles(field, choose_roles(field), enemies_near_goal)
        robot_roles = self.choose_robots_for_roles(field, roles, wall_pos, enemies_near_goal.copy())

        if field.ally_color == const.COLOR:
            print("Roles", field.ally_color)
            for idx, role in enumerate(robot_roles):
                if role != Role.UNAVAILABLE:
                    print("  ", idx, role)

        # Calculate remaining endpoints and create waypoints
        self.forwards = find_role(field, robot_roles, Role.FORWARD)
        if self.forwards is not None:
            attack_roles.set_forwards_wps(field, waypoints, self.forwards, self.pass_points)

        if Role.ATTACKER in robot_roles:
            attacker_id = find_role(field, robot_roles, Role.ATTACKER)[0].r_id
            attack_roles.attacker(field, waypoints, attacker_id, self.pass_points, self.forwards)

        pass_defenders = find_role(field, robot_roles, Role.PASS_DEFENDER)
        if len(pass_defenders) > 0:
            defense_roles.set_pass_defenders_wps(field, waypoints, pass_defenders, enemies_near_goal)

        wallliners = find_role(field, robot_roles, Role.WALLLINER)
        if len(wallliners) > 0:
            defense_roles.set_wallliners_wps(field, waypoints, wallliners)

        pass_receivers = find_role(field, robot_roles, Role.PASS_RECEIVER)
        if len(pass_receivers) > 0:
            attack_roles.set_pass_receivers_wps(field, waypoints, pass_receivers)

        if Role.GOALKEEPER in robot_roles:
            waypoints[field.gk_id] = defense_roles.goalk(field, wallliners)

    def debug(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> list[wp.Waypoint]:
        """Debug"""
        # waypoints[10] = wp.Waypoint(aux.Point(200, 0), 1.5, wp.WType.S_VELOCITY)
        # field.allies[10].set_dribbler_speed(15)
        # waypoints[10] = self.kick.shoot_to_goal(
        #     field, field.allies[10], field.enemy_goal.center
        # )
        # return

        match self.flag:
            case 0:
                pos = aux.Point(-250, 1000)
                angle = math.pi

            case 1:
                pos = aux.Point(-500, -1000)
                angle = math.pi

            case 2:
                pos = aux.Point(-500, -2000)
                angle = math.pi

            case 3:
                pos = aux.Point(-750, 1200)
                angle = math.pi
        idd = 1
        if aux.in_place(field.allies[idd].get_pos(), pos, 50):
            if time() - self.timer > 0.5:
                self.flag += 1
                self.flag = self.flag % 4
        else:
            self.timer = time()
        angle += math.pi / 4
        waypoints[idd] = wp.Waypoint(pos, angle, wp.WType.S_ENDPOINT)
        print("vel", field.allies[idd].get_vel().mag())
        print("dist to pos", (field.allies[idd].get_pos() - pos).mag())
        field.strategy_image.draw_dot(pos, (0, 0, 0), const.ROBOT_R)

        return waypoints

    def choose_robots_for_roles(
        self,
        field: fld.Field,
        roles: list[Role],
        wall_pos: aux.Point,
        enemies_near_goal: list[aux.Point],
    ) -> list[Role]:
        """Selects a robot for each role (based on role's priority)"""
        robot_roles: list[Role] = [Role.UNAVAILABLE for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        used_ids: list[int] = []

        for r_id, role in enumerate(self.prev_roles):  # fix defenders's roles if they are caching a ball
            if role in [Role.WALLLINER, Role.PASS_DEFENDER] and field.is_ball_moves_to_point(field.allies[r_id].get_pos()):
                used_ids.append(r_id)
                robot_roles[r_id] = role
                delete_role(roles, role)

        # used_ids.append(13)
        # robot_roles[13] = Role.WALLLINER
        # delete_role(roles, Role.WALLLINER)

        for role in roles:
            robot_id: Optional[int] = None
            match role:
                case Role.GOALKEEPER:
                    robot_id = field.gk_id
                case Role.ATTACKER:
                    if field.robot_with_ball not in field.allies or field.robot_with_ball == field.allies[field.gk_id]:
                        robot_id = fld.find_nearest_robot(field.ball.get_pos(), field.allies, used_ids).r_id
                    else:
                        robot_id = field.robot_with_ball.r_id
                case Role.PASS_DEFENDER:
                    enemy = enemies_near_goal.pop(0)
                    robot_id = fld.find_nearest_robot(enemy, field.allies, used_ids).r_id
                case Role.WALLLINER:
                    robot_id = fld.find_nearest_robot(wall_pos, field.allies, used_ids).r_id
                case Role.FORWARD:
                    robot_id = fld.find_nearest_robot(field.enemy_goal.center, field.allies, used_ids).r_id

            if robot_id is not None:
                robot_roles[robot_id] = role
                used_ids.append(robot_id)

        for robot in field.active_allies():
            if field.is_ball_moves_to_point(
                robot.get_pos()
            ):  # and self.prev_roles[robot.r_id] not in [Role.WALLLINER, Role.PASS_DEFENDER]
                robot_roles[robot.r_id] = Role.PASS_RECEIVER

        for r_id, rbt_rl in enumerate(robot_roles):
            if rbt_rl != Role.UNAVAILABLE:
                match rbt_rl:
                    case Role.GOALKEEPER:
                        text = "GK"
                        clr = 255
                    case Role.ATTACKER:
                        text = "AT"
                        clr = 255
                    case Role.PASS_DEFENDER:
                        text = "PD"
                        clr = 150
                    case Role.WALLLINER:
                        text = "WL"
                        clr = 100
                    case Role.FORWARD:
                        text = "FRW"
                        clr = 200
                    case Role.PASS_RECEIVER:
                        text = "PR"
                        clr = 255
                text_pos = field.allies[r_id].get_pos() + aux.Point(0, const.ROBOT_R * 1.5)
                field.strategy_image.print(text_pos, text, (clr, clr, clr))

        self.prev_roles = robot_roles
        return robot_roles


def choose_roles(field: fld.Field) -> list[Role]:
    """Defining a set of roles depending on the situation on the field"""

    attackers = [
        Role.FORWARD,
        Role.FORWARD,
        Role.FORWARD,
        Role.FORWARD,
        Role.FORWARD,
        Role.FORWARD,
    ]

    defenders = [
        Role.WALLLINER,
        Role.PASS_DEFENDER,
        Role.WALLLINER,
        Role.PASS_DEFENDER,
        Role.WALLLINER,
        Role.WALLLINER,
    ]

    # atk_min = 1
    # def_min = 2
    atk_min = 1
    def_min = 1

    free_allies = -atk_min - def_min - 1
    total_active = 0

    for ally in field.allies:
        if ally.is_used() and ally.r_id != field.gk_id:
            total_active += 1

    free_allies = max(0, free_allies + total_active)

    ball_pos = aux.minmax(field.ball.get_pos().x, const.GOAL_DX)
    atks = round(free_allies / (2 * const.GOAL_DX) * (-ball_pos * field.polarity + const.GOAL_DX)) + atk_min
    defs = free_allies - (atks - atk_min) + def_min

    roles = attackers[:atks] + defenders[:defs]

    res_roles = [Role.GOALKEEPER, Role.ATTACKER] + roles
    total_active += 1
    return res_roles[:total_active]


def manage_roles(
    field: fld.Field,
    roles: list[Role],
    enemies_near_goal: list[aux.Point],
) -> list[Role]:
    """Fixes possible errors in the set of roles"""
    pass_defenders_num = len(find_role(field, roles, Role.PASS_DEFENDER))
    if pass_defenders_num > len(enemies_near_goal):
        roles = replace_role(
            roles,
            Role.PASS_DEFENDER,
            Role.WALLLINER,
            pass_defenders_num - len(enemies_near_goal),
        )

    return sorted(roles, key=lambda x: x.value)


def delete_role(roles: list[Role], role_to_delete: Role) -> None:
    """Removes a role from an array (if the role is not in the array, removes the role with the lowest priority)"""
    for i, role in enumerate(roles):
        if role == role_to_delete:
            roles.pop(i)
            return
    roles.pop()


def find_role(field: fld.Field, roles: list[Role], role_to_find: Role) -> list[rbt.Robot]:
    """Returns an array with all robots of the role 'role_to_find'"""
    robots: list[rbt.Robot] = []
    for i, role in enumerate(roles):
        if role == role_to_find:
            robots.append(field.allies[i])

    return robots


def replace_role(roles: list[Role], old_role: Role, new_role: Role, count: int = 1) -> list[Role]:
    """Replaces 'old_role' with 'new_role', executed 'count' times"""
    num = 0
    for i, role in enumerate(roles):
        if role == old_role:
            roles[i] = new_role
            num += 1
            if num >= count:
                return roles
    return roles
