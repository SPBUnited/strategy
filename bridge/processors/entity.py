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
    """
    Реальное дифференцирующее звено первого порядка
    """
    def __init__(self, T, dT, is_angle = False):
        """
        Конструктор

        T - постоянная времени ФНЧ
        dT - период квантования
        """
        self._T = T
        self._dT = dT
        self._I = 0
        self._out = 0
        self._is_angle = is_angle

    def process(self, x):
        """
        Рассчитать и получить следующее значение выхода звена
        
        ВЫЗЫВАТЬ РАЗ В ПЕРИОД КВАНТВАНИЯ

        x - новое значение входа
        """
        err = x - self._I
        if self._is_angle:
            # print(err, x, self.I)
            if err > math.pi:
                err -= 2*math.pi
                self._I += 2*math.pi
            elif err < -math.pi:
                err += 2*math.pi
                self._I -= 2*math.pi
        self._out = err/self._T
        self._I += self._out * self._dT
        return self._out

    def getVal(self):
        """
        Получить последнее значение выхода звена без расчета
        """
        return self._out

class FOLP:
    """
    Фильтр низких частот первого порядка
    """
    def __init__(self, T, dT):
        """
        Конструктор

        T - постоянная времени ФНЧ
        dT - период квантования
        """
        self._T = T
        self._dT = dT
        self._I = 0
        self._out = 0

    def process(self, x):
        """
        Рассчитать и получить следующее значение выхода звена
        
        ВЫЗЫВАТЬ РАЗ В ПЕРИОД КВАНТВАНИЯ

        x - новое значение входа
        """
        err = x - self._out
        self._I += err * self._dT
        self._out = self._I/self._T
        return self._out

    def getVal(self):
        """
        Получить последнее значение выхода звена без расчета
        """
        return self._out

class Entity:
    """
    Класс для описания геометрического объекта на поле

    Хранит положение, скорость, угол и тп.
    """
    def __init__(self, pos, angle, R) -> None:
        """
        Конструктор
        
        @param pos Изначальное положение объекта. Тип: aux.Point
        @param angle Угол поворота объекта [рад]
        @param R Радиус объекта [м]
        """
        T = 0.05
        dT = const.Ts

        self._pos = pos
        self._vel = aux.Point(0, 0)
        self._velFx = FOD(T, dT)
        self._velFy = FOD(T, dT)
        self._acc = aux.Point(0, 0)
        self._accFx = FOD(T, dT)
        self._accFy = FOD(T, dT)
        self._angle = angle
        self._anglevel = 0
        self._velFr = FOD(T, dT, True)
        self._R = R

    def update(self, pos, angle):
        """
        Обновить положение и рассчитать исходя из этого скорость и ускорение
        !!! Вызывать один раз за итерацию с постоянной частотой !!!

        @todo Реализовать расчет скоростей и ускорений
        """
        self._pos = pos
        self._angle = angle
        self._vel.x = self._velFx.process(self._pos.x)
        self._vel.y = self._velFy.process(self._pos.y)
        self._acc.x = self._accFx.process(self._vel.x)
        self._acc.y = self._accFy.process(self._vel.y)
        self._anglevel = self._velFr.process(self._angle)

    def getPos(self) -> aux.Point:
        """ Геттер положения """
        return self._pos
    
    def getVel(self) -> aux.Point:
        """ Геттер скорости """
        return self._vel
    
    def getAcc(self) -> aux.Point:
        """ Геттер ускорения """
        return self._acc
    
    def getAngle(self) -> float:
        """ Геттер угла """
        return self._angle
    
    def getR(self) -> float:
        """ Геттер радиуса """
        return self._R

    def __str__(self) -> str:
        """ Для print """
        return str(self._pos)
