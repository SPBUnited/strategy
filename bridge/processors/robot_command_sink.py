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

import bridge.processors.robot as robot
import bridge.processors.const as const

@attr.s(auto_attribs=True)
class CommandSink(BaseProcessor):

    processing_pause: typing.Optional[int] = 0.01
    max_commands_to_persist: int = 20
    commands_sink_reader: DataReader = attr.ib(init=False)
    commands_writer: DataWriter = attr.ib(init=False)

    b_control_team = [robot.Robot(const.GRAVEYARD_POS, 0, const.ROBOT_R, 'b', i, 0) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]
    y_control_team = [robot.Robot(const.GRAVEYARD_POS, 0, const.ROBOT_R, 'y', i, 0) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]


    def initialize(self, data_bus: DataBus) -> None:
        super(CommandSink, self).initialize(data_bus)
        self.commands_sink_reader = DataReader(data_bus, const.TOPIC_SINK, 20)
        self.commands_writer = DataWriter(data_bus, config.ROBOT_COMMANDS_TOPIC, self.max_commands_to_persist)

    def process(self) -> None:
        
        cmds = self.commands_sink_reader.read_new()

        for cmd in cmds:
            r = cmd.content
            rId = r.rId
            ctrlId = r.ctrlId

            if ctrlId == None:
                continue

            if r.color == 'b':
                self.b_control_team[ctrlId].copy_control_fields(r)
            elif r.color == 'y':
                self.y_control_team[ctrlId].copy_control_fields(r)
        
        rules = self.get_rules()

        self.commands_writer.write(rules)

    def get_rules(self):
        """
        Сформировать массив команд для отправки на роботов
        """
        rules = []

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if abs(self.b_control_team[i].speedX) < 1.5:
                self.b_control_team[i].speedX = 0
            if abs(self.b_control_team[i].speedY) < 1.5:
                self.b_control_team[i].speedY = 0
            if abs(self.b_control_team[i].speedR) < 1.5:
                self.b_control_team[i].speedR = 0
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
            if abs(self.y_control_team[i].speedX) < 1.5:
                self.y_control_team[i].speedX = 0
            if abs(self.y_control_team[i].speedY) < 1.5:
                self.y_control_team[i].speedY = 0
            if abs(self.y_control_team[i].speedR) < 1.5:
                self.y_control_team[i].speedR = 0
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
