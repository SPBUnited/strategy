"""
Модуль для задания эталонных сигналов для отладки всякого
"""
import time
import math

class Signal:


    def __init__(self, period, waveform, ampoffset = None, lohi = None):
        self.T0 = time.time()
        self.period = period
        self.waveform = waveform

        self.amp = 1
        self.offset = 0

        if lohi is not None:
            self.amp = (lohi[1] - lohi[0])/2
            self.offset = (lohi[1] + lohi[0])/2
        elif ampoffset is not None:
            self.amp = ampoffset[0]
            self.offset = ampoffset[1]

        self.waveforms = \
        {
            'SQUARE': self.square,
            'SINE': self.sine,
        }
    
    def get(self):
        return self.waveforms[self.waveform]()

    def square(self):
        return math.copysign(self.amp, math.sin(2*math.pi*(time.time() - self.T0)/self.period)) + self.offset

    def sine(self):
        return self.amp * math.sin(2*math.pi*(time.time() - self.T0)/self.period) + self.offset
