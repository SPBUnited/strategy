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

class Entity:

    """
    Конструктор
    
    @param pos Изначальное положение объекта. Тип: aux.Point
    @param angle Угол поворота объекта [рад]
    @param R Радиус объекта [м]
    """
    def __init__(self, pos, angle, R) -> None:
        self.pos = pos
        self.vel = aux.Point(0, 0)
        self.acc = aux.Point(0, 0)
        self.angle = angle
        self.R = R

    """
    Обновить положение и рассчитать исходя из этого скорость и ускорение
    !!! Вызывать один раз за итерацию с постоянной частотой !!!

    @todo Реализовать расчет скоростей и ускорений
    """
    def update(self, pos, angle):
        self.pos = pos
        self.angle = angle

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
