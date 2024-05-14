import math
import bridge.processors.auxiliary as auxiliary
import bridge.processors.fncs as fncs
import bridge.processors.const as const

myPrint = fncs.PrintTime(100)

class Robot:
    def __init__(self, color, r_id, x, y, orientation):
        self.rId = r_id
        self.isUsed = 1
        self.x = x
        self.y = y
        self.orientation = orientation
        self.maxSpeed = const.MAX_SPEED
        self.maxSpeedR = const.MAX_SPEED_R
        self.minSpeed = const.MAX_SPEED / 10
        self.minSpeedR = const.MAX_SPEED_R / 10
        self.color = color
        self.minDist = 10
        self.minRotate = 0.075
        self.withBall = 0

        # Changeable params
        self.speedX = 0.0
        self.speedY = 0.0
        self.speedR = 0.0
        self.kickUp = 0.0
        self.kickForward = 0.0
        self.autoKick = 1.0
        self.kickerVoltage = const.BASE_KICKER_VOLTAGE
        self.dribblerEnable = 0.0
        self.speedDribbler = 0.0
        self.kickerChargeEnable = 1.0
        self.beep = 0.0
        self.acc = const.ACCELERATION

    def update(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.kickForward = 0
        self.kickUp = 0
        self.isUsed = 1

    def kick_forward(self):
        self.kickForward = 1

    def kick_up(self):
        self.kickUp = 1

    def go_to_point_with_detour(self, target_point, y_team, b_team):
        pass

    def go_to_point(self, point, finish = True):
        #return
        dist = auxiliary.dist(self, point)
        kp = 0.02
        angle = math.atan2(point.y - self.y, point.x - self.x)
        ###print(dist)
        if finish:
            v = min(dist * kp, self.maxSpeed)
            v = max(v, self.maxSpeed / 20 * (dist > self.minDist))
        else:
            v = self.maxSpeed
        self.speedX = v * math.cos(angle - self.orientation)
        self.speedY = v * math.sin(angle - self.orientation)
        if const.IS_SIMULATOR_USED:
            self.speedY *= -1
        ###print(dist)

    def gone_to_point(self, point):
        dist = auxiliary.dist(self, point)
        return dist <= self.minDist
            

    def get_speed(self, distance):
        pass


    def rotate_to_point(self, point):
        #return
        degree = math.atan2(point.y - self.y, point.x - self.x)
        delta = degree - self.orientation
        if(abs(delta) > math.pi):
            delta = (2 * math.pi - abs(delta)) * auxiliary.sgn(delta)
        kp = 10
        v = min(abs(delta * kp), self.maxSpeedR) * auxiliary.sgn(delta)
        v = max(abs(v), self.maxSpeedR / 10 * (abs(delta) > self.minRotate)) * auxiliary.sgn(v)
        self.speedR = v
    
    def rotated_to_point(self, point):
        degree = math.atan2(point.y - self.y, point.x - self.x)
        delta = degree - self.orientation
        return abs(delta) <= self.minRotate

    def go_to_ball(self, ball, enemies):
        robot_r = 220
        lines = [(-400, 400)]
        for i in range(3):
            now_rb = y_team.robot(i)
            if now_rb.isUsed:
                angle1 = math.atan2(ball.y - now_rb.y + robot_r, now_rb.x - ball.x)
                angle2 = math.atan2(ball.y - now_rb.y - robot_r, now_rb.x - ball.x)
                if abs(angle1) > math.pi / 2 and abs(angle2) > math.pi / 2:
                    continue
                now_ln = (ball.y - (4500 - ball.x) * math.tan(angle1), ball.y - (4500 - ball.x) * math.tan(angle2))
                if now_ln[0] > now_ln[1]:
                    now_ln = (now_ln[1], now_ln[0])
                # myPrint.myPrint(str(now_ln[0]) + " " + str(now_ln[1]))
                # myPrint.myPrint(i)
                j = 0
                while j < len(lines):
                    lines_flag = 0
                    if now_ln[0] > lines[j + lines_flag][0] and now_ln[0] < lines[j + lines_flag][1]:
                        lines.insert(j + lines_flag, (lines[j + lines_flag][0], now_ln[0]))
                        lines_flag += 1
                    if now_ln[1] > lines[j + lines_flag][0] and now_ln[1] < lines[j + lines_flag][1]:
                        lines.insert(j + lines_flag, (now_ln[1], lines[j + lines_flag][1]))
                        lines_flag += 1
                    if lines_flag != 0:
                        lines.pop(j + lines_flag)
                        j += lines_flag - 1
                    if now_ln[0] < lines[j][0] and now_ln[1] > lines[j][1] and lines_flag == 0:
                        lines.pop(j)
                        j -= 1
                    j += 1
        max_line = -1
        for i in range(len(lines)):
            if max_line != -1:
                if lines[max_line][1] - lines[max_line][0] < lines[i][1] - lines[i][0]:
                    max_line = i
            else:
                max_line = i
        if max_line == -1:
            grades = auxiliary.Point(4500, 0)
        else:
            grades = auxiliary.Point(4500, (lines[max_line][0] + lines[max_line][1]) / 2)
        # myPrint.myPrint(grades.y)
        alphaTr = math.atan2(ball.y - grades.y, grades.x - ball.x)
        point = auxiliary.Point(ball.x, ball.y)
        point.x -= robot_r * math.cos(alphaTr)
        point.y += robot_r * math.sin(alphaTr)
        aTr = auxiliary.dist(self, point)
        bTr = auxiliary.dist(self, ball)
        cTr = auxiliary.dist(ball, point)
        pTr = (aTr + bTr + cTr) / 2
        STr = math.sqrt(pTr * (pTr - aTr) * (pTr - bTr) * (pTr - cTr))
        hTr = 2 * STr / aTr
        gamma = math.acos((bTr ** 2 + cTr ** 2 - aTr ** 2) / (2 * bTr * cTr))
        ###print(str(ball.x) + " " + str(ball.y))
        ###print(str(self.x) + " " + str(self.y))
        self.rotate_to_point(ball)
        if abs(ball.y) > 3000 or abs(ball.x) > 4500:
            self.go_to_point(ball)
            return
        if bTr > robot_r * 2:
            self.withBall = 0
        if hTr > robot_r or aTr < bTr or gamma < math.pi / 4:
            beta = math.atan2(self.y - ball.y, ball.x - self.x)
            #if abs(math.sin(beta - alphaTr) * bTr) < 15 and gamma < 0.1 and abs(abs(-self.orientation - alphaTr) - abs(gamma)) < 0.1:
            if self.rotated_to_point(ball) == 1 and self.gone_to_point(point):
                self.withBall = 1
            if self.withBall:
                self.go_to_point(ball)
            else:
                self.go_to_point(point)
            ###print(str(gamma) + " " + str(math.sin(beta - alphaTr) * bTr) + " " + str(abs(-self.orientation - alphaTr)))
        else:
            ###print(1)
            bTr = max(robot_r, bTr)
            aPr = math.sqrt(bTr ** 2 - robot_r ** 2)
            alphaPr = math.atan2(robot_r, aPr)
            beta = math.atan2(ball.y - self.y, self.x - ball.x)
            way1 = auxiliary.Point(self.x - (aPr + robot_r) * math.cos(beta - alphaPr), self.y + (aPr + robot_r) * math.sin(beta - alphaPr))
            way2 = auxiliary.Point(self.x - (aPr + robot_r) * math.cos(beta + alphaPr), self.y + (aPr + robot_r) * math.sin(beta + alphaPr))
            ###print(str(auxiliary.dist(way1, point)) + " " + str(auxiliary.dist(way2, point)))
            if(auxiliary.dist(way1, point) < auxiliary.dist(way2, point)):
                self.go_to_point(way1)
                ###print(str(way1.x) + " " + str(way1.y))
                #pass
            else:
                self.go_to_point(way2)
                ###print(str(way2.x) + " " + str(way2.y))
                #pass

    def stWay(self, pToGo, nRobots, rRobots, allies, enemies):
        startP = PointTree(allies.robot(self.rId).x, allies.robot(self.rId).y)
        endP = PointTree(pToGo.x, pToGo.y)
        allRobots = []
        queue = [startP]
        nMax = float('inf')
        nSteps = 0
        finish = None
        ###print(queue[nSteps].x, queue[nSteps].y)
        tree = []
        pointGoNow = None
        for i in range(nRobots):
            allRobots.append(allies.robot(i))
            allRobots.append(enemies.robot(i))
            ###print(allies.robot(i).x, allies.robot(i).y, enemies.robot(i).x, enemies.robot(i).y)
        allRobots.pop(self.rId * 2)
        while nSteps < nMax:
            onWay = []
            for i in range(len(allRobots)):
                if hLine(queue[nSteps], endP, allRobots[i]) < rRobots * 2 and queue[nSteps].myRobot != i:
                    onWay.append(i)
            if(len(onWay) == 0):
                flag = 1
                if endP.father:
                    oldWayL = 0
                    pointMas = endP
                    while pointMas.father != None:
                        oldWayL += auxiliary.dist(pointMas, pointMas.father)
                        pointMas = pointMas.father
                    newWayL = auxiliary.dist(endP, queue[nSteps])
                    pointMas = queue[nSteps]
                    while pointMas.father != None:
                        newWayL += auxiliary.dist(pointMas, pointMas.father)
                        pointMas = pointMas.father
                    if oldWayL < newWayL:
                        flag = 0
                if flag:
                    nMax = min(nSteps + min(5, len(queue) - nSteps - 1), nMax)
                    endP.father = queue[nSteps]
                    #print("start")
                    pointMas = endP
                    finish = True
                    while pointMas.father.father != None:
                        #print(pointMas.x, pointMas.y)
                        finish = False
                        pointMas = pointMas.father
                    pointGoNow = pointMas
            else:
                minNumOnWay = -1
                minDistOnWay = float('inf')
                for i in range(len(onWay)):
                    if auxiliary.dist(allRobots[onWay[i]], queue[nSteps]) < minDistOnWay:
                        minDistOnWay = auxiliary.dist(allRobots[onWay[i]], queue[nSteps])
                        minNumOnWay = i
                mas2 = [onWay[minNumOnWay]]
                i2 = 0
                while i2 < len(mas2):
                    for another in range(len(allRobots)):
                        if auxiliary.dist(allRobots[mas2[i2]], allRobots[another]) < rRobots * 4 and another not in mas2:
                            mas2.append(another)
                    i2 += 1
                angle1 = 0
                dist1 = 0
                angle2 = 0
                dist2 = 0
                gamma = math.atan2(endP.y - queue[nSteps].y, endP.x - queue[nSteps].x)
                for wall in mas2:
                    a = auxiliary.dist(allRobots[wall], queue[nSteps])
                    b = rRobots * 2
                    c = math.sqrt(abs(a ** 2 - b ** 2))
                    alpha = math.atan2(b, c)
                    beta = math.atan2(allRobots[wall].y - queue[nSteps].y, allRobots[wall].x - queue[nSteps].x)
                    tetta = beta + alpha - gamma
                    if abs(tetta) > math.pi:
                        tetta = math.pi*2 * -auxiliary.sgn(tetta) - tetta
                    if tetta > 0:
                        if abs(angle1) < abs(tetta):
                            angle1 = tetta
                            dist1 = c + 100
                    else:
                        if abs(angle2) < abs(tetta):
                            angle2 = tetta
                            dist2 = c + 100
                    tetta = beta - alpha - gamma
                    if abs(tetta) > math.pi:
                        tetta = math.pi*2 * -auxiliary.sgn(tetta) + tetta
                    if tetta > 0:
                        if abs(angle1) < abs(tetta):
                            angle1 = tetta
                            dist1 = c + 100
                    else:
                        if abs(angle2) < abs(tetta):
                            angle2 = tetta
                            dist2 = c + 100
                queue.append(PointTree(queue[nSteps].x + math.cos(angle2 + gamma) * dist2, queue[nSteps].y + math.sin(angle2 + gamma) * dist2, queue[nSteps], onWay[minNumOnWay]))
                queue.append(PointTree(queue[nSteps].x + math.cos(angle1 + gamma) * dist1, queue[nSteps].y + math.sin(angle1 + gamma) * dist1, queue[nSteps], onWay[minNumOnWay]))
            nSteps += 1
        self.go_to_point(pointGoNow, finish)
        self.rotate_to_point(pointGoNow)
        #print(pointGoNow.x, pointGoNow.y)

def withLine(edge1, edge2, rPoint, maxAngle = math.pi / 2):
    alpha = math.atan2(edge2.y - edge1.y, edge2.x - edge1.x)
    beta1 = math.atan2(rPoint.y - edge1.y, rPoint.x - edge1.x)
    beta2 = math.atan2(rPoint.x - edge2.x, edge2.y - rPoint.y)
    corner1 = abs(alpha - beta1)
    corner2 = abs(math.pi / 2 - alpha + beta2)
    corner1 = min(corner1, 2 * math.pi - corner1)
    corner2 = min(corner2, 2 * math.pi - corner2)
    # ##print(str(corner1) + " " + str(corner2))
    ###print(corner1, corner2)
    if corner1 > maxAngle or corner2 > maxAngle:
        return False
    return True

def hLine(edge1, edge2, rPoint, onLine = True, maxAngle = math.pi / 2):
    a = auxiliary.dist(edge1, edge2)
    b = auxiliary.dist(edge1, rPoint)
    c = auxiliary.dist(edge2, rPoint)
    p = (a + b + c) / 2
    sTr = math.sqrt(p * (p - a) * (p - b) * (p - c))
    hTr = sTr * 2 / a
    if onLine:
        if not withLine(edge1, edge2, rPoint, maxAngle):
            hTr = float('inf')
    return hTr

class PointTree:
    def __init__(self, x, y, father = None, myRobot = None):
        self.x = x
        self.y = y
        self.father = father
        self.myRobot = myRobot