"""
Структура для хранения информации об одном объекте на поле (робот или мяч)

Хранит:
- положение
- скорость
- ускорение
- угол
- радиус
"""

import math
import bridge.processors.auxiliary as aux
import bridge.processors.entity as entity
import bridge.processors.const as const


class FOD:
    def __init__(self, T, dT):
        self.T = T
        self.dT = dT
        self.I = 0
        self.out = 0

    def process(self, x):
        err = x - self.I
        self.out = err/self.T
        self.I += self.out * self.dT
        return self.out

    def getVal(self):
        return self.out

class FOLP:
    def __init__(self, T, dT):
        self.T = T
        self.dT = dT
        self.I = 0
        self.out = 0

    def process(self, x):
        err = x - self.out
        self.I += err * self.dT
        self.out = self.I/self.T
        return self.out

    def getVal(self):
        return self.out

class Entity:

    """
    Конструктор
    
    @param pos Изначальное положение объекта. Тип: aux.Point
    @param angle Угол поворота объекта [рад]
    @param R Радиус объекта [м]
    """
    def __init__(self, pos, angle, R) -> None:
        T = 0.05
        dT = const.Ts

        self.pos = pos
        self.vel = aux.Point(0, 0)
        self.velFx = FOD(T, dT)
        self.velFy = FOD(T, dT)
        self.acc = aux.Point(0, 0)
        self.accFx = FOD(T, dT)
        self.accFy = FOD(T, dT)
        self.angle = angle
        self.anglevel = 0
        self.velFr = FOD(T, dT)
        self.R = R

    """
    Обновить положение и рассчитать исходя из этого скорость и ускорение
    !!! Вызывать один раз за итерацию с постоянной частотой !!!

    @todo Реализовать расчет скоростей и ускорений
    """
    def update(self, pos, angle):
        self.pos = pos
        self.angle = angle
        self.vel.x = self.velFx.process(self.pos.x)
        self.vel.y = self.velFy.process(self.pos.y)
        self.acc.x = self.accFx.process(self.vel.x)
        self.acc.y = self.accFy.process(self.vel.y)
        self.anglevel = self.velFr.process(self.angle)

    """ Геттер положения """
    def getPos(self) -> aux.Point:
        return self.pos
    
    """ Геттер скорости """
    def getVel(self) -> aux.Point:
        return self.vel
    
    """ Геттер ускорения """
    def getAcc(self) -> aux.Point:
        return self.acc
    
    """ Геттер угла """
    def getAngle(self) -> float:
        return self.angle
    
    """ Геттер радиуса """
    def getR(self) -> float:
        return self.R

    def __str__(self) -> str:
        return str(self.pos)
