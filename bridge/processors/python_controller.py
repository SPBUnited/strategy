import attr
import numpy as np
import typing
import time
import bridge.processors.const as const
import bridge.processors.robot as robot

from strategy_bridge.bus import DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.model.referee import RefereeCommand
from strategy_bridge.processors import BaseProcessor
from strategy_bridge.bus import DataBus
from strategy_bridge.utils.debugger import debugger
from strategy_bridge.pb.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket

import math
import bridge.processors.auxiliary as auxiliary

import bridge.processors.field as field
import bridge.processors.router as router
import bridge.processors.strategy as strategy
import bridge.processors.waypoint as wp
import bridge.processors.signal as signal

from bridge.processors.robot_command_sink import CommandSink

# TODO: Refactor this class and corresponding matlab scripts
@attr.s(auto_attribs=True)
class SSLController(BaseProcessor):

    max_commands_to_persist: int = 20
    our_color: str = 'b'

    vision_reader: DataReader = attr.ib(init=False)
    referee_reader: DataReader = attr.ib(init=False)
    commands_sink_writer: DataWriter = attr.ib(init=False)

    dbg_game_status: strategy.GameStates = strategy.GameStates.TIMEOUT
    dbg_state: strategy.States = strategy.States.DEBUG

    cur_time = time.time()
    dt = 0

    ctrl_mapping = const.CONTROL_MAPPING
    print(ctrl_mapping)
    count_halt_cmd = 0

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализировать контроллер
        """
        super(SSLController, self).initialize(data_bus)
        self.vision_reader = DataReader(data_bus, config.VISION_DETECTIONS_TOPIC)
        self.referee_reader = DataReader(data_bus, config.REFEREE_COMMANDS_TOPIC)
        self.commands_sink_writer = DataWriter(data_bus, const.TOPIC_SINK, 20)
        self._ssl_converter = SSL_WrapperPacket()

        self.field = field.Field(self.ctrl_mapping, self.our_color)
        self.router = router.Router(self.field)
        self.strategy = strategy.Strategy(self.dbg_game_status, self.dbg_state)

    def get_last_referee_command(self) -> RefereeCommand:
        """
        Получить последнюю команду рефери
        """
        referee_commands = self.referee_reader.read_new()
        if referee_commands:
            return referee_commands[-1].content
        return RefereeCommand(0, 0, False)

    def read_vision(self) -> bool:
        """
        Прочитать новые пакеты из SSL-Vision
        """
        status = False

        balls = np.zeros(const.BALL_PACKET_SIZE * const.MAX_BALLS_IN_FIELD)
        field_info = np.zeros(const.GEOMETRY_PACKET_SIZE)

        queue = self.vision_reader.read_new()

        for ssl_package in queue:
            try:
                ssl_package_content = ssl_package.content
            except:
                None
            if not ssl_package_content:
                continue

            status = True

            ssl_package_content = self._ssl_converter.FromString(ssl_package_content)
            geometry = ssl_package_content.geometry
            if geometry:
                field_info[0] = geometry.field.field_length
                field_info[1] = geometry.field.field_width

            detection = ssl_package_content.detection
            camera_id = detection.camera_id
            for ball_ind, ball in enumerate(detection.balls):
                balls[ball_ind + (camera_id - 1) * const.MAX_BALLS_IN_CAMERA] = camera_id
                balls[ball_ind + const.MAX_BALLS_IN_FIELD + (camera_id - 1) * const.MAX_BALLS_IN_CAMERA] = ball.x
                balls[ball_ind + 2 * const.MAX_BALLS_IN_FIELD + (camera_id - 1) * const.MAX_BALLS_IN_CAMERA] = ball.y
                self.field.updateBall(auxiliary.Point(ball.x, ball.y))

            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if time.time() - self.field.b_team[i].last_update() > 1:
                    self.field.b_team[i].used(0)
                if time.time() - self.field.y_team[i].last_update() > 1:
                    self.field.y_team[i].used(0)

            
            self.strategy.changeGameState(strategy.GameStates.RUN, 0)

            # curCmd = self.get_last_referee_command()
            # if curCmd.state == 0:
            #     self.count_halt_cmd += 1
            # else:
            #     self.count_halt_cmd = 0
            #     if curCmd.state == 1:
            #         self.strategy.changeGameState(strategy.GameStates.STOP, curCmd.commandForTeam)
            #     elif curCmd.state == 2:
            #         self.strategy.changeGameState(strategy.GameStates.RUN, curCmd.commandForTeam)
            #     elif curCmd.state == 3:
            #         self.strategy.changeGameState(strategy.GameStates.TIMEOUT, curCmd.commandForTeam)
            #     elif curCmd.state == 4:
            #         print("Uknown command")
            #     elif curCmd.state == 5:
            #         self.strategy.changeGameState(strategy.GameStates.PREPARE_KICKOFF, curCmd.commandForTeam)
            #     elif curCmd.state == 6:
            #         self.strategy.changeGameState(strategy.GameStates.KICKOFF, curCmd.commandForTeam)
            #     elif curCmd.state == 7:
            #         self.strategy.changeGameState(strategy.GameStates.PREPARE_PENALTY, curCmd.commandForTeam)
            #     elif curCmd.state == 8:
            #         self.strategy.changeGameState(strategy.GameStates.PENALTY, curCmd.commandForTeam)
            #     elif curCmd.state == 9:
            #         self.strategy.changeGameState(strategy.GameStates.FREE_KICK, curCmd.commandForTeam)
            #     elif curCmd.state == 10:
            #         print("Uknown command")
            #     elif curCmd.state == 11:
            #         self.strategy.changeGameState(strategy.GameStates.BALL_PLACMENT, curCmd.commandForTeam)
                
            # if self.count_halt_cmd > 10:
            #     self.strategy.changeGameState(strategy.GameStates.HALT, curCmd.commandForTeam)

            # TODO: Barrier states
            for robot in detection.robots_blue:
                if time.time() - self.field.b_team[robot.robot_id].last_update() > 0.3:
                    self.field.b_team[robot.robot_id].used(0)
                else:
                    self.field.b_team[robot.robot_id].used(1)
                self.field.updateBluRobot(robot.robot_id, auxiliary.Point(robot.x, robot.y), robot.orientation, time.time())

            for robot in detection.robots_yellow:
                if time.time() - self.field.y_team[robot.robot_id].last_update() > 0.3:
                    self.field.y_team[robot.robot_id].used(0)
                else: 
                    self.field.y_team[robot.robot_id].used(1)
                self.field.updateYelRobot(robot.robot_id, auxiliary.Point(robot.x, robot.y), robot.orientation, time.time())
        return status
    
    def control_loop(self):
        """
        Рассчитать стратегию, тактику и физику для роботов на поле
        """
        self.router.update(self.field)
        waypoints = self.strategy.process(self.field)
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.router.getRoute(i).clear()
            self.router.setDest(i, waypoints[i])
        self.router.reRoute(self.field)

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.field.allies[i].go_route(self.router.getRoute(i), self.field)

        # for i in range(const.TEAM_ROBOTS_MAX_COUNT):
        #     print(self.field.y_team[i])
        # for i in range(const.TEAM_ROBOTS_MAX_COUNT):
        #     print(self.field.b_team[i])
        print(self.router.getRoute(const.DEBUG_ID))

    square = signal.Signal(2, 'SQUARE', lohi=(-20, 20))
    sine = signal.Signal(2, 'SINE', ampoffset=(1000, 0))
    cosine = signal.Signal(2, 'COSINE', ampoffset=(1000, 0))
    def control_assign(self):
        """
        Определить связь номеров роботов с каналами управления
        """
        # self.field.allies[const.DEBUG_ID].speedR = self.square.get()
        # self.field.allies[const.DEBUG_ID].speedX = 0
        # self.field.allies[const.DEBUG_ID].speedY = 0
        # print(self.square.get())
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.commands_sink_writer.write(self.field.allies[i])

    @debugger
    def process(self) -> None:
        """
        Выполнить цикл процессора
        
        \offtop Что означает @debugger?
        """

        self.dt = time.time() - self.cur_time
        self.cur_time = time.time()

        # print(self.field.ally_goal.center)

        self.read_vision()
        self.control_loop()
        self.control_assign()
