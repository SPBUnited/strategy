import attr
import numpy as np
import typing
import struct
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

# TODO: Refactor this class and corresponding matlab scripts
@attr.s(auto_attribs=True)
class MatlabController(BaseProcessor):

    processing_pause: typing.Optional[float] = 0.001
    max_commands_to_persist: int = 20

    vision_reader: DataReader = attr.ib(init=False)
    referee_reader: DataReader = attr.ib(init=False)
    commands_writer: DataWriter = attr.ib(init=False)

    cur_time = time.time()
    dt = 0
    ball = auxiliary.Point(0, 0)

    controll_team = [robot.Robot(const.GRAVEYARD_POS, 0, const.ROBOT_R, 'b', i) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]

    field = field.Field()
    router = router.Router()
    strategy = strategy.Strategy()

    def initialize(self, data_bus: DataBus) -> None:
        super(MatlabController, self).initialize(data_bus)
        self.vision_reader = DataReader(data_bus, config.VISION_DETECTIONS_TOPIC)
        self.referee_reader = DataReader(data_bus, config.REFEREE_COMMANDS_TOPIC)
        self.commands_writer = DataWriter(data_bus, config.ROBOT_COMMANDS_TOPIC, self.max_commands_to_persist)
        self._ssl_converter = SSL_WrapperPacket()

    def get_last_referee_command(self) -> RefereeCommand:
        referee_commands = self.referee_reader.read_new()
        if referee_commands:
            return referee_commands[-1].content
        return RefereeCommand(0, 0, False)

    def read_vision(self) -> bool:
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
                self.ball = auxiliary.Point(ball.x, ball.y)
                self.field.updateBall(self.ball)

            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if time.time() - self.field.b_team[i].last_update() > 1:
                    self.field.b_team[i].used(0)
                if time.time() - self.field.y_team[i].last_update() > 1:
                    self.field.y_team[i].used(0)

            # TODO: Barrier states
            for robot in detection.robots_blue:
                # self.b_team.robot(robot.robot_id).update(robot.x, robot.y, robot.orientation)
                if time.time() - self.field.b_team[robot.robot_id].last_update() > 0.3:
                    self.field.b_team[robot.robot_id].used(0)
                else: 
                    self.field.b_team[robot.robot_id].used(1)
                self.field.updateBluRobot(robot.robot_id, auxiliary.Point(robot.x, robot.y), robot.orientation, time.time())

            for robot in detection.robots_yellow:
                # self.y_team.robot(robot.robot_id).update(robot.x, robot.y, robot.orientation)
                if time.time() - self.field.y_team[robot.robot_id].last_update() > 0.3:
                    self.field.y_team[robot.robot_id].used(0)
                else: 
                    self.field.y_team[robot.robot_id].used(1)
                self.field.updateYelRobot(robot.robot_id, auxiliary.Point(robot.x, robot.y), robot.orientation, time.time())
                #self.y_team.robot(robot.robot_id).isUsed = 1
        return status
    
    """
    Рассчитать стратегию, тактику и физику для роботов на поле
    """
    def control_loop(self):
        waypoints = self.strategy.process(self.field)
        for i in range(6):
            self.router.setWaypoint(i, waypoints[i])
        self.router.calcRoutes(self.field)

        # TODO алгоритм следования по траектории
        # TODO Убрать артефакты
        for i in range(1):
            # self.y_team.robot(i).go_to_point_with_detour(self.router.getRoute(i)[-1].pos, self.b_team, self.y_team)
            if self.router.getRoute(i)[-1].type == wp.WType.IGNOREOBSTACLES:
                self.field.b_team[i].go_to_point(self.router.getRoute(i)[-1].pos)
            else:
                self.field.b_team[i].go_to_point_vector_field(self.router.getRoute(i)[-1].pos, self.field)

            self.field.b_team[i].rotate_to_angle(self.router.getRoute(i)[-1].angle)
            #self.field.b_team[i].rotate_to_angle(0)
            #self.field.b_team[i].rotate_to_angle(math.pi)

        dbg = 0

        if dbg:
            self.field.b_team[5].go_to_point_vector_field(auxiliary.Point(1000, 1000), self.field)
            self.field.b_team[3].go_to_point_vector_field(auxiliary.Point(700, 1300), self.field)
            self.field.b_team[5].rotate_to_angle(0)
            self.field.b_team[3].rotate_to_angle(0)

    """
    Определить связь номеров роботов с каналами управления
    """
    def control_assign(self):
        for r in self.controll_team:
            r.clear_fields()

        # TODO Задавать соответствие списком
        for i in range(6):
            self.controll_team[i].copy_control_fields(self.field.b_team[i])

    """
    Сформировать массив команд для отправки на роботов
    """
    def get_rules(self):
        rules = []

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            # self.controll_team[i].remake_speed()
            rules.append(0)
            rules.append(self.controll_team[i].speedX)
            rules.append(self.controll_team[i].speedY)
            rules.append(self.controll_team[i].speedR)
            rules.append(self.controll_team[i].kickForward)
            rules.append(self.controll_team[i].kickUp)
            rules.append(self.controll_team[i].autoKick)
            rules.append(self.controll_team[i].kickerVoltage)
            rules.append(self.controll_team[i].dribblerEnable)
            rules.append(self.controll_team[i].speedDribbler)
            rules.append(self.controll_team[i].kickerChargeEnable)
            rules.append(self.controll_team[i].beep)            
            rules.append(0)
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            for _ in range(0, 13):
                rules.append(0.0)
            
        b = bytes()
        rules = b.join((struct.pack('d', rule) for rule in rules))
        return rules

    """
    Выполнить цикл процессора
    
    \offtop Что означает @debugger?
    """
    @debugger
    def process(self) -> None:

        ts_check = False
        while(time.time() - self.cur_time < const.Ts):
            ts_check = True
        if not ts_check:
            print("ПРЕВЫШЕН ПЕРИОД КВАНТОВАНИЯ НА: ", '%.3f' % (time.time() - self.cur_time - const.Ts))

        self.dt = time.time() - self.cur_time
        self.cur_time = time.time()

        self.read_vision()
        self.control_loop()
        self.control_assign()
        rules = self.get_rules()
        self.commands_writer.write(rules)

