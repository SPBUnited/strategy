"""
Динамические звенья и регуляторы
"""

import math
import bridge.processors.auxiliary as aux
from enum import Enum, auto

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

class Integrator():
    """
    Интегратор
    """
    def __init__(self, dT):
        """
        Конструктор

        dT - период квантования
        """
        self._dT = dT
        self._I = 0
        self._out = 0
    
    def reset(self):
        self._I = 0

    def process(self, x):
        """
        Рассчитать и получить следующее значение выхода звена
        
        ВЫЗЫВАТЬ РАЗ В ПЕРИОД КВАНТВАНИЯ

        x - новое значение входа
        """
        self._I += x * self._dT
        self._out = self._I
        return self._out

    def getVal(self):
        """
        Получить последнее значение выхода звена без расчета
        """
        return self._out

class Mode(Enum):
    NORMAL = 0
    SOFT = auto()

class PISD():
    """
    Пропорционально-скользяще-интегральный регулятор

    (В отличие от ПИД беред производную от скорости изменения регулируемой
    величины, а не ошибки)
    """
    def __init__(self, dT, gain, kd, ki, max_out) -> None:
        """
        Конструктор

        каждый параметр - список коэффициентов для разных режимов
        gain - коэффициент усиления регулятора (П составляющая)
        kd - коэффициент дифференциальной части (типа Д составляющая)
        ki - коэффициент интегрирующей части (И составляющая)
        max_out - Максимальное значение управляющего воздействия
        """
        self.__gain = gain
        self.__kd = kd
        self.__ki = ki
        self.__max_out = max_out
        self.__I = Integrator(dT)
        self.__out = 0
        self.__mode = Mode.NORMAL
    
    def select_mode(self, mode: Mode):
        self.__mode = mode
        self.__I.reset()
    
    def __get_gains(self):
        return (self.__gain[self.__mode.value], self.__kd[self.__mode.value], self.__ki[self.__mode.value], self.__max_out[self.__mode.value])

    def process(self, xerr, x_i):
        """
        Рассчитать следующий тик регулятора
        """
        gain, kd, ki, max_out = self.__get_gains()
        # print(gain, kd, ki, max_out)

        s = xerr + kd*x_i + ki*self.__I.getVal()
        u = gain * s

        u_clipped = aux.minmax(u, -max_out, max_out)

        if u != u_clipped:
            self.__I.process(xerr + kd*x_i)

        self.__out = u_clipped
        return self.__out

    def getVal(self):
        """
        Получить последнее значение выхода звена без расчета
        """
        return self.__out

class RateLimiter():
    """
    Ограничитель скорости роста
    """
    def __init__(self, Ts, max_der) -> None:
        self.__out = 0
        self.__I = Integrator(Ts)
        self.__k = 1/Ts
        self.__max_der = max_der
    
    def process(self, x):
        u = aux.minmax(self.__k * (x - self.__out), -self.__max_der, self.__max_der)
        self.__out = self.__I.process(u)
        return self.__out

    def getVal(self):
        return self.__out

