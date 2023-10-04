import asyncio
import logging
import typing
import struct

import attr

from strategy_bridge.bus import DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.model.referee import RefereeCommand
from strategy_bridge.processors import BaseProcessor
from strategy_bridge.bus import DataBus
from strategy_bridge.utils.debugger import debugger
from strategy_bridge.pb.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket

from bridge.processors.singleton import Singleton
import bridge.processors.robot as robot
import bridge.processors.const as const

@attr.s(auto_attribs=True)
class Sink():

    b_control_team = [robot.Robot(const.GRAVEYARD_POS, 0, const.ROBOT_R, 'b', i) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]
    y_control_team = [robot.Robot(const.GRAVEYARD_POS, 0, const.ROBOT_R, 'y', i) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]

    def __init__(self):
        pass

    def write_ctrl(self, color, robots, mapping):
        print("WRITE")
        # print(self.b_control_team)
        if color == 'b':
            for i in range(len(mapping)):
                self.b_control_team[mapping[i]].copy_control_fields(robots[i])
        elif color == 'y':
            for i in range(len(mapping)):
                self.y_control_team[mapping[i]].copy_control_fields(robots[i])
        
    def get_rules(self):
        print("GET")
        # print(self.b_control_team)
        """
        Сформировать массив команд для отправки на роботов
        """
        rules = []

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            rules.append(0)
            rules.append(self.b_control_team[i].speedX)
            rules.append(self.b_control_team[i].speedY)
            rules.append(self.b_control_team[i].speedR)
            rules.append(self.b_control_team[i].kickForward)
            rules.append(self.b_control_team[i].kickUp)
            rules.append(self.b_control_team[i].autoKick)
            rules.append(self.b_control_team[i].kickerVoltage)
            rules.append(self.b_control_team[i].dribblerEnable)
            rules.append(self.b_control_team[i].speedDribbler)
            rules.append(self.b_control_team[i].kickerChargeEnable)
            rules.append(self.b_control_team[i].beep)            
            rules.append(0)
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            rules.append(0)
            rules.append(self.y_control_team[i].speedX)
            rules.append(self.y_control_team[i].speedY)
            rules.append(self.y_control_team[i].speedR)
            rules.append(self.y_control_team[i].kickForward)
            rules.append(self.y_control_team[i].kickUp)
            rules.append(self.y_control_team[i].autoKick)
            rules.append(self.y_control_team[i].kickerVoltage)
            rules.append(self.y_control_team[i].dribblerEnable)
            rules.append(self.y_control_team[i].speedDribbler)
            rules.append(self.y_control_team[i].kickerChargeEnable)
            rules.append(self.y_control_team[i].beep)            
            rules.append(0)
        
        b = bytes()
        rules = b.join((struct.pack('d', rule) for rule in rules))
        return rules

@attr.s(auto_attribs=True)
class CommandSink(BaseProcessor):

    processing_pause: typing.Optional[int] = 0.1
    max_commands_to_persist: int = 20
    commands_writer: DataWriter = attr.ib(init=False)
    sink: Sink = Sink()

    def initialize(self, data_bus: DataBus) -> None:
        super(CommandSink, self).initialize(data_bus)
        self.commands_writer = DataWriter(data_bus, config.ROBOT_COMMANDS_TOPIC, self.max_commands_to_persist)

    def process(self) -> None:
        print(self.sink.b_control_team[0])
        rules = self.sink.get_rules()
        print(self.sink.b_control_team[0])
        # print(rules)
        self.commands_writer.write(rules)
