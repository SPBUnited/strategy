"""
Модуль-прослойка между стратегией и отправкой пакетов на роботов
"""

import struct
from time import time
from typing import Optional

import attr
import zmq
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.processors import BaseProcessor

from bridge import const
from bridge.auxiliary import aux, fld
from bridge.processors.python_controller import RobotCommand
from bridge.router import router
from bridge.router import waypoint as wp


@attr.s(auto_attribs=True)
class CommandSink(BaseProcessor):
    """
    Прослойка между стратегией и отправкой пакетов на роботов
    """

    processing_pause: Optional[float] = 0.2
    reduce_pause_on_process_time: bool = False

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализация
        """
        super().initialize(data_bus)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.commands_sink_reader = DataReader(data_bus, const.CONTROL_TOPIC)
        self.image_writer = DataWriter(data_bus, const.IMAGE_TOPIC, 20)

        self.field_b = fld.Field(const.Color.BLUE)
        self.field_y = fld.Field(const.Color.YELLOW)
        self.field: dict[const.Color, fld.Field] = {const.Color.BLUE: self.field_b, const.Color.YELLOW: self.field_y}

        self.router_b = router.Router(self.field_b)
        self.router_y = router.Router(self.field_y)
        self.router: dict[const.Color, router.Router] = {const.Color.BLUE: self.router_b, const.Color.YELLOW: self.router_y}

        self.waypoints_b: list[wp.Waypoint] = [
            wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_STOP) for _ in range(const.TEAM_ROBOTS_MAX_COUNT)
        ]
        self.waypoints_y: list[wp.Waypoint] = [
            wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_STOP) for _ in range(const.TEAM_ROBOTS_MAX_COUNT)
        ]
        self.waypoints: dict[const.Color, list[wp.Waypoint]] = {
            const.Color.BLUE: self.waypoints_b,
            const.Color.YELLOW: self.waypoints_y,
        }

        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{config.COMMANDS_PUBLISH_PORT}")

    def process(self) -> None:
        """
        Метод обратного вызова процесса
        """
        updated = False

        new_field = self.field_reader.read_last()
        if new_field is not None:
            updated_field: fld.Field = new_field.content
            if self.field_b.last_update != updated_field.last_update:
                self.field_b.update_field(updated_field)
                self.field_y.update_field(updated_field)
                updated = True

        cmds = self.commands_sink_reader.read_new()
        for cmd in cmds:
            print(len(cmds))
            command: RobotCommand = cmd.content
            if command.color == const.Color.BLUE:
                self.waypoints_b[command.r_id] = command.waypoint
            else:
                self.waypoints_y[command.r_id] = command.waypoint
            updated = True

        if updated:
            for color in [const.Color.BLUE, const.Color.YELLOW]:
                self.router[color].update(self.field[color])

                for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                    if self.field[color].allies[i].is_used():
                        self.field[color].allies[i].clear_fields()
                        self.router[color].get_route(i).clear()

                        self.router[color].set_dest(i, self.waypoints[color][i], self.field[color])
                        self.router[color].reroute(i, self.field[color])
                        self.router[color].get_route(i).go_route(self.field[color].allies[i], self.field[color])

            self.image_writer.write(self.field[const.COLOR].router_image)
            self.image_writer.write(self.field[const.COLOR].path_image)
            self.field_b.clear_images()
            self.field_y.clear_images()

            rules = self.get_rules()
            self.socket.send(rules)

    def get_rules(self) -> bytes:
        """
        Сформировать массив команд для отправки на роботов
        """
        rules: list[float] = []

        if const.IS_SIMULATOR_USED:
            b_control_team = self.field_b.allies
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                rules.append(0)
                rules.append(b_control_team[i].speed_x)
                rules.append(b_control_team[i].speed_y)
                rules.append(b_control_team[i].speed_r)
                rules.append(b_control_team[i].kick_up_)
                rules.append(b_control_team[i].kick_forward_)
                rules.append(b_control_team[i].auto_kick_)
                rules.append(b_control_team[i].kicker_voltage_ / 3)
                rules.append(b_control_team[i].dribbler_enable_)
                rules.append(b_control_team[i].dribbler_speed_)
                rules.append(b_control_team[i].kicker_charge_enable_)
                rules.append(b_control_team[i].beep)
                rules.append(0)

            y_control_team = self.field_y.allies
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                rules.append(0)
                rules.append(y_control_team[i].speed_x)
                rules.append(y_control_team[i].speed_y)
                rules.append(y_control_team[i].speed_r)
                rules.append(y_control_team[i].kick_up_)
                rules.append(y_control_team[i].kick_forward_)
                rules.append(y_control_team[i].auto_kick_)
                rules.append(y_control_team[i].kicker_voltage_ / 3)
                rules.append(y_control_team[i].dribbler_enable_)
                rules.append(y_control_team[i].dribbler_speed_)
                rules.append(y_control_team[i].kicker_charge_enable_)
                rules.append(b_control_team[i].beep)
                rules.append(0)
        else:
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if self.field_b.allies[i].is_used():
                    control_team = self.field_b.allies
                elif self.field_y.allies[i].is_used():
                    control_team = self.field_y.allies
                else:
                    for _ in range(13):
                        rules.append(0)
                    continue

                robot = control_team[i]
                if i in const.REVERSED_KICK:
                    robot.kick_forward_, robot.kick_up_ = robot.kick_up_, robot.kick_forward_
                    if robot.auto_kick_ == 2:
                        robot.auto_kick_ = 1
                    elif robot.auto_kick_ == 1:
                        robot.auto_kick_ = 2

                if not const.IS_DRIBBLER_USED:
                    if round(time() * 2) % 10 == 0:
                        control_team[i].dribbler_enable_ = 1
                        control_team[i].dribbler_speed_ = 1
                    else:
                        control_team[i].dribbler_enable_ = 0
                        control_team[i].dribbler_speed_ = 0

                rules.append(0)
                rules.append(control_team[i].speed_x)
                rules.append(control_team[i].speed_y)
                rules.append(control_team[i].delta_angle)
                rules.append(control_team[i].kick_up_)
                rules.append(control_team[i].kick_forward_)
                rules.append(control_team[i].auto_kick_)
                rules.append(control_team[i].kicker_voltage_)
                rules.append(control_team[i].dribbler_enable_)
                rules.append(control_team[i].dribbler_speed_)
                rules.append(control_team[i].kicker_charge_enable_)
                rules.append(control_team[i].beep)
                rules.append(0)
            for _ in range(const.TEAM_ROBOTS_MAX_COUNT):
                for _ in range(13):
                    rules.append(0)

        # rules = [15] * 13 * 32
        b = bytes()
        return b.join((struct.pack("d", rule) for rule in rules))
