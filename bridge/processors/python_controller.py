import attr
import numpy as np
import typing
import struct

from bridge.bus import DataReader, DataWriter
from bridge.common import config
#from bridge.matlab.engine import matlab_engine
from bridge.model.referee import RefereeCommand
from bridge.processors import BaseProcessor
import math
import bridge.processors.auxiliary as auxiliary



class Robot:
    def __init__(self, r_id, x, y, orientation):
        self.rId = r_id
        self.isUsed = 1
        self.x = x
        self.y = y
        self.orientation = orientation
        self.maxSpeed = 6
        self.maxSpeedR = 10

        # Changeable params
        self.koef = 1.0
        self.speedX = 0.0
        self.speedY = 0.0
        self.speedR = 0.0
        self.kickUp = 0.0
        self.kickForward = 0.0
        self.autoKick = 0.0
        self.kickerVoltage = 15.0
        self.dribblerEnable = 0.0
        self.speedDribbler = 0.0
        self.kickerChargeEnable = 0.0
        self.beep = 0.0

    def update(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.kickForward = 0
        self.kickUp = 0

    def kick_forward(self):
        self.kickForward = 1

    def kick_up(self):
        self.kickUp = 1

    def go_to_point(self, point):
        # Calculate the angle to the ball
        angle_to_point = math.atan2(point.y - self.y, point.x - self.x)

        # Calculate the distance to the ball
        distance_to_point = math.dist((self.x, self.y), (point.x, point.y))

        self.speedX = distance_to_point * math.cos(angle_to_point + self.orientation) * 0.03
        self.speedY = -distance_to_point * math.sin(angle_to_point + self.orientation) * 0.03
        print(point.x, point.y, self.x, self.y, distance_to_point)

    def rotate_to_point(self, point):
        vx = self.x - point.x
        vy = self.y - point.y
        ux = -math.cos(self.orientation)
        uy = -math.sin(self.orientation)
        dif = -math.atan2(auxiliary.scal_mult(auxiliary.Point(vx, vy), auxiliary.Point(ux, uy)),
                          auxiliary.vect_mult(auxiliary.Point(vx, vy), auxiliary.Point(ux, uy)))

        if abs(dif) > 0.001:
            self.speedR = dif * 30
        else:
            self.speedR = 0

koef = 1.0

yRobots = []
for i in range(16):
    yRobots.append(Robot(i, 0, 0, 0))
bRobots = []
for i in range(16):
    bRobots.append(Robot(i, 0, 0, 0))


# TODO: Refactor this class and corresponding matlab scripts
@attr.s(auto_attribs=True)
class MatlabController(BaseProcessor):

    processing_pause: typing.Optional[float] = 0.001
    max_commands_to_persist: int = 20

    vision_reader: DataReader = attr.ib(init=False, default=DataReader(config.VISION_DETECTIONS_TOPIC))
    referee_reader: DataReader = attr.ib(init=False, default=DataReader(config.REFEREE_COMMANDS_TOPIC))
    commands_writer: DataWriter = attr.ib(init=False)

    CAMERAS_COUNT: int = 4
    MAX_BALLS_IN_CAMERA: int = 64
    MAX_BALLS_IN_FIELD: int = CAMERAS_COUNT * MAX_BALLS_IN_CAMERA
    BALL_PACKET_SIZE: int = 3

    ROBOTS_MAX_COUNT: int = 32
    TEAM_ROBOTS_MAX_COUNT: int = ROBOTS_MAX_COUNT // 2
    SINGLE_ROBOT_PACKET_SIZE = 5
    ROBOT_TEAM_PACKET_SIZE: int = SINGLE_ROBOT_PACKET_SIZE * TEAM_ROBOTS_MAX_COUNT

    GEOMETRY_PACKET_SIZE: int = 2

    def __attrs_post_init__(self):
        self.commands_writer = DataWriter(config.ROBOT_COMMANDS_TOPIC, self.max_commands_to_persist)

    def get_last_referee_command(self) -> RefereeCommand:
        referee_commands = self.referee_reader.read_new()
        if referee_commands:
            return referee_commands[-1]
        return RefereeCommand(0, 0, False)

    async def process(self) -> None:
        balls = np.zeros(self.BALL_PACKET_SIZE * self.MAX_BALLS_IN_FIELD)
        robots_blue = np.zeros(self.ROBOT_TEAM_PACKET_SIZE)
        robots_yellow = np.zeros(self.ROBOT_TEAM_PACKET_SIZE)
        field_info = np.zeros(self.GEOMETRY_PACKET_SIZE)
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
                balls[ball_ind + (camera_id - 1) * self.MAX_BALLS_IN_CAMERA] = camera_id
                balls[ball_ind + self.MAX_BALLS_IN_FIELD + (camera_id - 1) * self.MAX_BALLS_IN_CAMERA] = ball.x
                balls[ball_ind + 2 * self.MAX_BALLS_IN_FIELD + (camera_id - 1) * self.MAX_BALLS_IN_CAMERA] = ball.y

            # TODO: Barrier states
            # print(detection.robots_blue)
            for robot in detection.robots_blue:
                # robots_blue[robot.robot_id] = camera_id
                # robots_blue[robot.robot_id + self.TEAM_ROBOTS_MAX_COUNT] = robot.x
                # robots_blue[robot.robot_id + self.TEAM_ROBOTS_MAX_COUNT * 2] = robot.y
                # robots_blue[robot.robot_id + self.TEAM_ROBOTS_MAX_COUNT * 3] = robot.orientation
                bRobots[robot.robot_id].update(robot.x, robot.y, robot.orientation)

            for robot in detection.robots_yellow:
                # robots_yellow[robot.robot_id] = camera_id
                # robots_yellow[robot.robot_id + self.TEAM_ROBOTS_MAX_COUNT] = robot.x
                # robots_yellow[robot.robot_id + self.TEAM_ROBOTS_MAX_COUNT * 2] = robot.y
                # robots_yellow[robot.robot_id + self.TEAM_ROBOTS_MAX_COUNT * 3] = robot.orientation
                yRobots[robot.robot_id].update(robot.x, robot.y, robot.orientation)

            # referee_command = self.get_last_referee_command()
            
                #yRobots[i].speedR = 15.0
            # yRobots[11].autoKick = 2.0
            # yRobots[11].kickVoltage = 15.0
            # yRobots[10].speedR = 5.0
            # global koef
            # yRobots[10].speedR = 15 * koef
            # koef *= -1
            # KOEF *= -1
            # yRobots[10].autoKick = 2.0
            # yRobots[10].kickVoltage = 15.0
            # yRobots[10].kickerChargeEnable = 1.0

            rules = []

            bRobots[0].go_to_point(auxiliary.Point(0, 0))
            yRobots[0].go_to_point(auxiliary.Point(0, 0))
            
            for i in range(self.TEAM_ROBOTS_MAX_COUNT):
                if yRobots[i].isUsed:
                    rules.append(0)
                    rules.append(bRobots[i].speedX)
                    rules.append(bRobots[i].speedY)
                    rules.append(bRobots[i].speedR)
                    rules.append(bRobots[i].kickForward)
                    rules.append(bRobots[i].kickUp)
                    rules.append(bRobots[i].autoKick)
                    rules.append(bRobots[i].kickerVoltage)
                    rules.append(bRobots[i].dribblerEnable)
                    rules.append(bRobots[i].speedDribbler)
                    rules.append(bRobots[i].kickerChargeEnable)
                    rules.append(bRobots[i].beep)            
                    rules.append(0)
                else:
                    for _ in range(0, 13):
                        rules.append(0.0)    
            for i in range(self.TEAM_ROBOTS_MAX_COUNT):
                if yRobots[i].isUsed:
                    rules.append(0)
                    rules.append(yRobots[i].speedX)
                    rules.append(yRobots[i].speedY)
                    rules.append(yRobots[i].speedR)
                    rules.append(yRobots[i].kickForward)
                    rules.append(yRobots[i].kickUp)
                    rules.append(yRobots[i].autoKick)
                    rules.append(yRobots[i].kickerVoltage)
                    rules.append(yRobots[i].dribblerEnable)
                    rules.append(yRobots[i].speedDribbler)
                    rules.append(yRobots[i].kickerChargeEnable)
                    rules.append(yRobots[i].beep)            
                    rules.append(0)
                else:
                    for _ in range(0, 13):
                        rules.append(0.0)    
          
                    
            b = bytes()
            rules = b.join((struct.pack('d', rule) for rule in rules))
            self.commands_writer.write(rules)
        # except: None
        from datetime import datetime
        time = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        with open("tmp/matlab_controller.txt", "a") as f:
            f.write(time + "\n")
