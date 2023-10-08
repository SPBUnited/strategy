import math
import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as field
import bridge.processors.entity as entity
import bridge.processors.waypoint as wp
from typing import List

class Robot(entity.Entity):


    def __init__(self, pos, angle, R, color, r_id, ctrl_id):
        super().__init__(pos, angle, R)

        self.rId = r_id
        self.ctrlId = ctrl_id
        self.isUsed = 0
        self.color = color
        self.lastUpdate = 0
        self.errOld = 0

        self.speedX = 0
        self.speedY = 0
        self.speedR = 0
        self.kickUp = 0
        self.kickForward = 0
        self.autoKick = 0
        self.kickerVoltage = 15
        self.dribblerEnable = 0
        self.speedDribbler = 0
        self.kickerChargeEnable = 1
        self.beep = 0   
        self.role = None

        #v! SIM
        self.Kxx = -833/20
        self.Kyy = 833/20
        self.Kww = 1.25/20
        self.Kwy = -0.001
        self.Twy = 0.15
        self.RcompFdy = entity.FOD(self.Twy, const.Ts)
        self.RcompFfy = entity.FOLP(self.Twy, const.Ts)

        #v! REAL
        # self.Kxx = 250/20
        # self.Kyy = -250/20
        # self.Kww = 6/20
        # self.Kwy = 0
        # self.Twy = 0.15
        # self.RcompFdy = entity.FOD(self.Twy, const.Ts)
        # self.RcompFfy = entity.FOLP(self.Twy, const.Ts)

        self.xxTF = 0.1
        self.xxFlp = entity.FOLP(self.xxTF, const.Ts)
        self.yyTF = 0.1
        self.yyFlp = entity.FOLP(self.yyTF, const.Ts)


    def used(self, a):
        self.isUsed = a

    def is_used(self):
        return self.isUsed
    
    def last_update(self):
        return self.lastUpdate

    def update(self, pos, angle, t):
        super().update(pos, angle)
        self.kickForward = 0
        self.kickUp = 0
        self.lastUpdate = t

    def kick_forward(self):
        self.kickForward = 1

    def kick_up(self):
        self.kickUp = 1

    def copy_control_fields(self, robot):
        self.speedX = robot.speedX
        self.speedY = robot.speedY
        self.speedR = robot.speedR
        self.kickUp = robot.kickUp
        self.kickForward = robot.kickForward
        self.autoKick = robot.autoKick
        self.kickerVoltage = robot.kickerVoltage
        self.dribblerEnable = robot.dribblerEnable
        self.speedDribbler = robot.speedDribbler
        self.kickerChargeEnable = robot.kickerChargeEnable
        self.beep = robot.beep

    def clear_fields(self):
        self.speedX = 0
        self.speedY = 0
        self.speedR = 0
        self.kickUp = 0
        self.kickForward = 0
        self.autoKick = 0
        self.kickerVoltage = 0
        self.dribblerEnable = 0
        self.speedDribbler = 0
        self.kickerChargeEnable = 0
        self.beep = 0
    
    def is_kick_aligned(self, target: wp.Waypoint):
        return (self.getPos() - target.pos).mag() < const.KICK_ALIGN_DIST*const.KICK_ALIGN_DIST_MULT and \
            abs(aux.wind_down_angle(self._angle - target.angle)) < const.KICK_ALIGN_ANGLE and \
            abs(aux.wind_down_angle((target.pos - self.getPos()).arg() - target.angle)) < const.KICK_ALIGN_OFFSET
    
    def is_ball_in(self, field: field.Field):
        return (self.getPos() - field.ball.getPos()).mag() < const.BALL_GRABBED_DIST and \
            abs(aux.wind_down_angle((field.ball.getPos() - self.getPos()).arg() - self._angle)) < const.BALL_GRABBED_ANGLE

    def go_route(self, route, field: field.Field):
        cur_speed = self._vel.mag()

        # print(route)

        dist = route.getLenght()

        target_point = route.getNextWP()
        end_point = route.getDestWP()
        
        vel0 = (self.getPos() - target_point.pos).unity()

        dangle = (target_point.pos - self.getPos()).arg()
        rangle = aux.wind_down_angle(self._angle - dangle)
        twpangle = aux.wind_down_angle(target_point.angle - dangle)

        angle60_abs = math.pi/4 if abs(rangle) < math.pi/2 else 2*math.pi/3
        angle60_sign = aux.sign(twpangle + rangle)

        angle60 = dangle + angle60_abs * angle60_sign

        lerp_angles = [target_point.angle, angle60]

        angle0 = aux.LERP(lerp_angles[0], lerp_angles[1], aux.minmax((dist-100)/1000, 0, 1))

        k = 0.2
        gain = 6
        k_a = 0.04
        gain_a = 1

        if end_point.type == wp.WType.S_BALL_KICK or \
            end_point.type == wp.WType.S_BALL_GRAB or \
            end_point.type == wp.WType.S_BALL_GO:

            # print("IS KICK ALIGNED: ", self.is_kick_aligned(end_point))

            k = 0.6
            gain = 2
            gain_a = 1
            angle0 = aux.LERP(lerp_angles[0], lerp_angles[1], aux.minmax((dist-400)/1000, 0, 1))

            if end_point.type == wp.WType.S_BALL_GO:
                angle0 = end_point.angle

        if end_point.type == wp.WType.S_BALL_KICK or end_point.type == wp.WType.S_BALL_GRAB and \
            self.is_kick_aligned(end_point):
            vel0 = (self.getPos() - end_point.pos).unity()
            angle0 = end_point.angle
            dist = 400

            self.dribblerEnable = True
            self.speedDribbler = 5
            if end_point.type == wp.WType.S_BALL_KICK:
                self.autoKick = 1
            else:
                self.autoKick = 0
        else:
            self.autoKick = 0

        err = dist - cur_speed * k
        u = aux.minmax(err * gain, -const.MAX_SPEED, const.MAX_SPEED)
        # print('%d'%dist, '%d'%cur_speed, err, u)
        transl_vel = vel0 * u

        aerr = aux.wind_down_angle(angle0 - self.getAngle())

        
        aerr -= self._anglevel * k_a
        u_a = min(max(aerr * gain_a, -const.MAX_SPEED_R), const.MAX_SPEED_R)
        ang_vel = u_a

        # if self.rId == 0 and self.color == 'b':
        #     print(self.is_kick_aligned(end_point), '%.2f'%angle60, '%.2f'%end_point.angle, '%.2f'%angle0, '%.2f'%aerr, '%.2f'%self.angle)
        self.update_vel_xyw(transl_vel, ang_vel)

    def update_vel_xyw(self, vel: aux.Point, wvel: float):
        """
        Выполнить тик регуляторов скорости робота

        vel - требуемый вектор скорости [мм/с] \\
        wvel - требуемая угловая скорость [рад/с]
        """
        self.speedX = self.xxFlp.process(1/self.Kxx * aux.rotate(vel, -self._angle).x)
        self.speedY = self.yyFlp.process(1/self.Kyy * aux.rotate(vel, -self._angle).y)

        # RcompY = self.Kwy * self.RcompFfy.process(self.RcompFdy.process(self.speedY))
        # RcompY = self.Kwy * self.RcompFdy.process(abs(float(self.speedY)**2))
        RcompY = 0
        self.speedR = 1/self.Kww * (wvel - RcompY)
        
    def clamp_motors(self):
        """
        Ограничить управляющее воздействие"""

    def __str__(self) -> str:
        return str(str(self.color) + " " + str(self.rId) + " " + str(self.getPos()) + " " + str(self.speedX) + " " + str(self.speedY)) + " " + str(self.speedR)

