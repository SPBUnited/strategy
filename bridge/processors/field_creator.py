"""
Processor that creates the field
"""

import typing
from time import time

import attr
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.pb.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from strategy_bridge.processors import BaseProcessor

from bridge import const
from bridge.auxiliary import aux, fld


@attr.s(auto_attribs=True)
class FieldCreator(BaseProcessor):
    """class that creates the field"""

    processing_pause: typing.Optional[float] = 0.01
    reduce_pause_on_process_time: bool = False
    # commands_sink_reader: DataReader = attr.ib(init=False)
    # box_feedback_reader: DataReader = attr.ib(init=False)
    # field_writer: DataWriter = attr.ib(init=False)
    _ssl_converter: SSL_WrapperPacket = attr.ib(init=False)

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализация
        """
        super().initialize(data_bus)
        self.vision_reader = DataReader(data_bus, config.VISION_DETECTIONS_TOPIC)
        self.box_feedback_reader = DataReader(data_bus, config.BOX_FEEDBACK_TOPIC)
        self.field_writer = DataWriter(data_bus, const.FIELD_TOPIC, 20)
        self._ssl_converter = SSL_WrapperPacket()
        self.field = fld.Field(const.COLOR)

    def process(self) -> None:
        """
        Метод обратного вызова процесса
        """

        queue = self.vision_reader.read_new()
        if not queue:
            return

        # print("field delay:", time() - self.field.last_update)
        balls: list[aux.Point] = []
        b_bots_id: list[int] = []
        b_bots_pos: list[list] = [[] for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        b_bots_ang: list[list] = [[] for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]

        y_bots_id: list[int] = []
        y_bots_pos: list[list] = [[] for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        y_bots_ang: list[list] = [[] for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        # field_info = np.zeros(const.GEOMETRY_PACKET_SIZE)

        for ssl_package in queue:
            try:
                ssl_package_content = ssl_package.content
            except AttributeError:
                continue

            ssl_package_content = self._ssl_converter.FromString(ssl_package_content)
            # geometry = ssl_package_content.geometry
            # if geometry:
            #     field_info[0] = geometry.field.field_length
            #     field_info[1] = geometry.field.field_width
            #     if geometry.field.field_length != 0 and geometry.field.goal_width != 0:
            #         const.GOAL_DX = geometry.field.field_length / 2
            #         const.GOAL_DY = geometry.field.goal_width

            detection = ssl_package_content.detection
            for ball in detection.balls:
                balls.append(aux.Point(ball.x, ball.y))

            for robot_det in detection.robots_blue:
                b_bots_id.append(robot_det.robot_id)
                b_bots_pos[robot_det.robot_id].append(aux.Point(robot_det.x, robot_det.y))
                b_bots_ang[robot_det.robot_id].append(robot_det.orientation)

            for robot_det in detection.robots_yellow:
                y_bots_id.append(robot_det.robot_id)
                y_bots_pos[robot_det.robot_id].append(aux.Point(robot_det.x, robot_det.y))
                y_bots_ang[robot_det.robot_id].append(robot_det.orientation)

        if len(balls) != 0:
            balls_sum = aux.Point(0, 0)
            balls_num = 0
            for ball in balls:
                if (
                    const.IS_SIMULATOR_USED
                    or (ball - self.field.ball.get_pos()).mag() / (time() - self.field.ball_real_update_time)
                    < const.BALL_MAX_SPEED
                ):
                    balls_sum += ball
                    balls_num += 1
            if balls_num != 0:
                ball = balls_sum / balls_num
                self.field.update_ball(ball, time())
                self.field.ball_real_update_time = time()
        elif self.field.robot_with_ball is not None:
            ally = self.field.robot_with_ball
            ball = ally.get_pos() + aux.rotate(aux.RIGHT, ally.get_angle()) * ally.get_radius() / 1.5
            self.field.update_ball(ball, time())

        self.field.robot_with_ball = None
        for r in self.field.all_bots:
            if self.field._is_ball_in(r):
                self.field.robot_with_ball = r

        for r_id in set(b_bots_id):
            position = aux.average_point(b_bots_pos[r_id])
            angle = aux.average_angle(b_bots_ang[r_id])
            if position != self.field.b_team[r_id].get_pos() or const.IS_SIMULATOR_USED:
                self.field.update_blu_robot(r_id, position, angle, time())
        for robot in self.field.b_team:
            if time() - robot.last_update() > 0.5:
                robot.used(0)
            else:
                robot.used(1)

        for r_id in set(y_bots_id):
            position = aux.average_point(y_bots_pos[r_id])
            angle = aux.average_angle(y_bots_ang[r_id])
            if position != self.field.y_team[r_id].get_pos() or const.IS_SIMULATOR_USED:
                self.field.update_yel_robot(r_id, position, angle, time())
        for robot in self.field.y_team:
            if time() - robot.last_update() > 0.5:
                robot.used(0)
            else:
                robot.used(1)

        self.field.last_update = time()
        self.field_writer.write(self.field)

        # if len(b_bots_pos[0]) > 0:
        #     print(b_bots_pos[0][0])

        # feedback_queue = self.box_feedback_reader.read_new()
        # if feedback_queue:
        #   print("feedback from box:", feedback_queue)
