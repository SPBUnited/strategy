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

IS_SIMULATOR_USED = 1

class Team:
    def __init__(self):
        self.robots = []

    def add_robot(self, robot):
        self.robots.append(robot)
    
    def robot(self, id):
        return self.robots[id]

    def point_on_line(self, robot, point, distance):
        # Calculate the angle to the target point
        angle_to_point = math.atan2(point.y - robot.y, point.x - robot.x)

        # Calculate the new point on the line at the specified distance from the robot
        new_x = robot.x + distance * math.cos(angle_to_point)
        new_y = robot.y + distance * math.sin(angle_to_point)
        return auxiliary.Point(new_x, new_y)

class Robot:
    def __init__(self, color, r_id, x, y, orientation):
        self.rId = r_id
        self.isUsed = 1
        self.x = x
        self.y = y
        self.orientation = orientation
        self.maxSpeed = 6
        self.maxSpeedR = 10
        self.color = color

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

    def go_to_point_with_detour(self, target_point, y_team, b_team):
        # Calculate the angle to the target point
        angle_to_point = math.atan2(target_point.y - self.y, target_point.x - self.x)

        # Calculate the distance to the target point
        distance_to_point = math.dist((self.x, self.y), (target_point.x, target_point.y))

        # Check if there are any robots in the way
        obstacles = []
        print('--------------------------')

        obstacle_distance_threshold = 1000  # Adjust this threshold

        for i in range(len(y_team.robots)):
            r = y_team.robot(i)
            if r != self:
                distance_to_robot = auxiliary.dist(self, r)
                if distance_to_robot < obstacle_distance_threshold and auxiliary.dist(self, target_point) != 0:
                    angle_to_robot = math.atan2(r.y - self.y, r.x - self.x)
                    angle_difference = abs(angle_to_robot - angle_to_point)
                    if angle_difference < math.pi / 4:  # Adjust this threshold for the angle
                        # Find the closest point on the line between self and target_point to y_robot
                        closest_point = auxiliary.closest_point_on_line(self, target_point, r)
                        
                        # Calculate the distance from y_robot to the closest point on the line
                        distance_to_line = auxiliary.dist(r, closest_point)
                        
                        if distance_to_line < obstacle_distance_threshold:
                            obstacles.append(r)
                            print(r.rId)

        for i in range(len(b_team.robots)):
            r = b_team.robot(i)
            if r != self:
                distance_to_robot = auxiliary.dist(self, r)
                if distance_to_robot < obstacle_distance_threshold and auxiliary.dist(self, target_point) != 0:
                    angle_to_robot = math.atan2(r.y - self.y, r.x - self.x)
                    angle_difference = abs(angle_to_robot - angle_to_point)
                    if angle_difference < math.pi / 4:  # Adjust this threshold for the angle
                        # Find the closest point on the line between self and target_point to y_robot
                        closest_point = auxiliary.closest_point_on_line(self, target_point, r)
                        
                        # Calculate the distance from y_robot to the closest point on the line
                        distance_to_line = auxiliary.dist(r, closest_point)
                        
                        if distance_to_line < obstacle_distance_threshold:
                            obstacles.append(r)
                            print(r.rId)

        if len(obstacles) == 0:
            # No obstacles, go directly to the target point
            self.go_to_point(target_point)
        else:
            # Find the closest obstacle
            closest_obstacle = min(obstacles, key=lambda robot: math.dist((self.x, self.y), (robot.x, robot.y)))

            # Calculate the angle to the obstacle
            angle_to_obstacle = math.atan2(closest_obstacle.y - self.y, closest_obstacle.x - self.x)

            # Calculate the tangent points
            angle_offset = math.pi / 2
            tangent_distance = 100  # Adjust this distance
            tangent_point1 = auxiliary.Point(
                self.x + tangent_distance * math.cos(angle_to_obstacle + angle_offset),
                self.y + tangent_distance * math.sin(angle_to_obstacle + angle_offset)
            )
            tangent_point2 = auxiliary.Point(
                self.x + tangent_distance * math.cos(angle_to_obstacle - angle_offset),
                self.y + tangent_distance * math.sin(angle_to_obstacle - angle_offset)
            )

            # Choose the closer tangent point as the detour point
            if math.dist((self.x, self.y), (tangent_point1.x, tangent_point1.y)) < math.dist((self.x, self.y), (tangent_point2.x, tangent_point2.y)):
                detour_point = tangent_point1
            else:
                detour_point = tangent_point2

            # Go to the detour point
            self.go_to_point(detour_point)


    def go_to_point(self, point):
        # Calculate the angle to the ball
        angle_to_point = math.atan2(point.y - self.y, point.x - self.x)

        # Calculate the distance to the ball
        distance_to_point = math.dist((self.x, self.y), (point.x, point.y))

        if IS_SIMULATOR_USED:
            self.speedX = distance_to_point * math.cos(angle_to_point - self.orientation) * 0.03
            self.speedY = -distance_to_point * math.sin(angle_to_point - self.orientation) * 0.03
        else:
            self.speedX = distance_to_point * math.cos(angle_to_point - self.orientation) * 0.03
            self.speedY = distance_to_point * math.sin(angle_to_point - self.orientation) * 0.03


    def rotate_to_point(self, point):
        vx = self.x - point.x
        vy = self.y - point.y
        ux = -math.cos(self.orientation)
        uy = -math.sin(self.orientation)
        if IS_SIMULATOR_USED:
            dif = -math.atan2(auxiliary.scal_mult(auxiliary.Point(vx, vy), auxiliary.Point(ux, uy)),
                            auxiliary.vect_mult(auxiliary.Point(vx, vy), auxiliary.Point(ux, uy)))
        else: 
            dif = math.atan2(auxiliary.scal_mult(auxiliary.Point(vx, vy), auxiliary.Point(ux, uy)),
                            auxiliary.vect_mult(auxiliary.Point(vx, vy), auxiliary.Point(ux, uy)))

        if abs(dif) > 0.001:
            self.speedR = dif * 30
        else:
            self.speedR = 0

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

    b_team = Team()
    y_team = Team()
    for i in range(TEAM_ROBOTS_MAX_COUNT):
        y_team.add_robot(Robot('y', i, 10e10, 10e10, 0))
    for i in range(TEAM_ROBOTS_MAX_COUNT):
        b_team.add_robot(Robot('b', i, 10e10, 10e10, 0))

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
            for robot in detection.robots_blue:
                self.b_team.robot(robot.robot_id).update(robot.x, robot.y, robot.orientation)

            for robot in detection.robots_yellow:
                self.y_team.robot(robot.robot_id).update(robot.x, robot.y, robot.orientation)

            # referee_command = self.get_last_referee_command()
            #for i in range(6):
            #    self.y_team.robot(i).go_to_point(stg.point_on_line(self.b_team.robot(i), auxiliary.Point(4500, 0), 300))
            self.y_team.robot(3).go_to_point_with_detour(auxiliary.Point(0, 0), self.y_team, self.b_team)
            
            rules = []

            for i in range(self.TEAM_ROBOTS_MAX_COUNT):
                if self.b_team.robot(i).isUsed:
                    rules.append(0)
                    rules.append(self.b_team.robot(i).speedX)
                    rules.append(self.b_team.robot(i).speedY)
                    rules.append(self.b_team.robot(i).speedR)
                    rules.append(self.b_team.robot(i).kickForward)
                    rules.append(self.b_team.robot(i).kickUp)
                    rules.append(self.b_team.robot(i).autoKick)
                    rules.append(self.b_team.robot(i).kickerVoltage)
                    rules.append(self.b_team.robot(i).dribblerEnable)
                    rules.append(self.b_team.robot(i).speedDribbler)
                    rules.append(self.b_team.robot(i).kickerChargeEnable)
                    rules.append(self.b_team.robot(i).beep)            
                    rules.append(0)
                else:
                    for _ in range(0, 13):
                        rules.append(0.0)    
            for i in range(self.TEAM_ROBOTS_MAX_COUNT):
                if self.y_team.robot(i).isUsed:
                    rules.append(0)
                    rules.append(self.y_team.robot(i).speedX)
                    rules.append(self.y_team.robot(i).speedY)
                    rules.append(self.y_team.robot(i).speedR)
                    rules.append(self.y_team.robot(i).kickForward)
                    rules.append(self.y_team.robot(i).kickUp)
                    rules.append(self.y_team.robot(i).autoKick)
                    rules.append(self.y_team.robot(i).kickerVoltage)
                    rules.append(self.y_team.robot(i).dribblerEnable)
                    rules.append(self.y_team.robot(i).speedDribbler)
                    rules.append(self.y_team.robot(i).kickerChargeEnable)
                    rules.append(self.y_team.robot(i).beep)            
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
