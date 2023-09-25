import attr
import numpy as np
import typing
import struct
import time
import bridge.processors.const as const
import bridge.processors.robot as robot
import bridge.processors.team as team

from bridge.bus import DataReader, DataWriter
from bridge.common import config
#from bridge.matlab.engine import matlab_engine
from bridge.model.referee import RefereeCommand
from bridge.processors import BaseProcessor
import math
import bridge.processors.auxiliary as auxiliary

import bridge.processors.field as field
import bridge.processors.router as router
import bridge.processors.strategy as strategy
import bridge.processors.waypoint as wp


#import bridge.processors.drawer as drw

# TODO: Refactor this class and corresponding matlab scripts
@attr.s(auto_attribs=True)
class MatlabController(BaseProcessor):

    processing_pause: typing.Optional[float] = 0.001
    max_commands_to_persist: int = 20

    vision_reader: DataReader = attr.ib(init=False, default=DataReader(config.VISION_DETECTIONS_TOPIC))
    referee_reader: DataReader = attr.ib(init=False, default=DataReader(config.REFEREE_COMMANDS_TOPIC))
    commands_writer: DataWriter = attr.ib(init=False)

    cur_time = time.time()
    dt = 0
    ball = auxiliary.Point(0, 0)

    controll_team = [robot.Robot(const.GRAVEYARD_POS, 0, const.ROBOT_R, 'b', i) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]

    field = field.Field()
    router = router.Router()
    strategy = strategy.Strategy()

    def __attrs_post_init__(self):
        self.commands_writer = DataWriter(config.ROBOT_COMMANDS_TOPIC, self.max_commands_to_persist)

    def get_last_referee_command(self) -> RefereeCommand:
        referee_commands = self.referee_reader.read_new()
        if referee_commands:
            return referee_commands[-1]
        return RefereeCommand(0, 0, False)

    async def process(self) -> None:
        balls = np.zeros(const.BALL_PACKET_SIZE * const.MAX_BALLS_IN_FIELD)
        field_info = np.zeros(const.GEOMETRY_PACKET_SIZE)
        ssl_package = 0
        try:
            ssl_package = self.vision_reader.read_new()[-1]
        except: None
        if ssl_package:            
            geometry = ssl_package.geometry
            if geometry:
                field_info[0] = geometry.field.field_length
                field_info[1] = geometry.field.field_width

            detection = ssl_package.detection
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

            # print(time.time(), end=" ")
            # for r in self.field.all_bots:
            #     print(r.is_used(), end=" ")
            #     # print(r.last_update(), end=" ")
            # print()
            
            #self.field.draw()

            waypoints = self.strategy.process(self.field)
            for i in range(6):
                self.router.setWaypoint(i, waypoints[i])
            self.router.calcRoutes(self.field)

            # TODO алгоритм следования по траектории
            for i in range(6):
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


            rules = []

            # self.b_team.robot(3).rotate_to_point(auxiliary.Point(4500, 500))
            # if auxiliary.dist(self.b_team.robot(3), self.ball) < 300 and \
            #     auxiliary.scal_mult((self.field.ball.pos - self.field.b_team[3].pos).unity(), (self.field.y_goal - self.field.b_team[3].pos).unity()) > 0.9:
            #     self.b_team.robot(3).go_to_point(self.ball)
            #     self.b_team.robot(3).kick_forward()
            # else:
            #     self.b_team.robot(3).go_to_point_with_detour(auxiliary.point_on_line(self.ball, auxiliary.Point(4500, 0), -200), self.b_team, self.y_team)

            for r in self.controll_team:
                r.clear_fields()

            self.controll_team[10].copy_control_fields(self.field.b_team[5])
            self.controll_team[11].copy_control_fields(self.field.b_team[3])
            self.controll_team[12].copy_control_fields(self.field.b_team[1])


            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                self.controll_team[i].remake_speed()
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
                    
            self.dt = time.time() - self.cur_time
            self.cur_time = time.time()
            b = bytes()
            rules = b.join((struct.pack('d', rule) for rule in rules))
            self.commands_writer.write(rules)

            #drw.Drawer().flip()
        # except: None
        from datetime import datetime
        time1 = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        with open("tmp/matlab_controller.txt", "a") as f:
            f.write(time1 + "\n")
