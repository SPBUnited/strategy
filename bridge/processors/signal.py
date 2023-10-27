"""
Модуль для задания эталонных сигналов для отладки всякого
"""
import math
import time


class Signal:
    """
    Класс-генератор сигналов
    """

    def __init__(self, period, waveform, ampoffset=None, lohi=None):
        """
        Конструтор

        Сигнал можно задать либо через амлитуду и смещение нуля (ampoffset),
        либо через минимальное и максимальное значения (lohi)
        """
        self.t_0 = time.time()
        self.period = period
        self.waveform = waveform

        self.amp = 1
        self.offset = 0

        if lohi is not None:
            self.amp = (lohi[1] - lohi[0]) / 2
            self.offset = (lohi[1] + lohi[0]) / 2
        elif ampoffset is not None:
            self.amp = ampoffset[0]
            self.offset = ampoffset[1]

        self.waveforms = {"SQUARE": self.square, "SINE": self.sine, "COSINE": self.cosine}

    def get(self):
        """
        Получить значение сигнала
        """
        return self.waveforms[self.waveform]()

    def square(self):
        """
        Получить значение меандра
        """
        return math.copysign(self.amp, math.sin(2 * math.pi * (time.time() - self.t_0) / self.period)) + self.offset

    def sine(self):
        """
        Получить значение синуса
        """
        return self.amp * math.sin(2 * math.pi * (time.time() - self.t_0) / self.period) + self.offset

    def cosine(self):
        """
        Получить значение косинуса
        """
        return self.amp * math.cos(2 * math.pi * (time.time() - self.t_0) / self.period) + self.offset
