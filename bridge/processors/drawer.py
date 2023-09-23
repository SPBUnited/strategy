"""
Модуль для визуального дебага и графической отрисовки всяких векторов и точек
"""

import pygame

import bridge.processors.auxiliary as aux

WIDTH = 900
HEIGHT = 600
FPS = 30

FIELD_WIDTH = 9000
FIELD_HEIGHT = 6000
SCALE_FACTOR = WIDTH/FIELD_WIDTH

# Задаем цвета
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (8, 132, 20)
BLUE = (0, 0, 255)

"""
Метакласс для реализации паттерна singleton
https://stackoverflow.com/questions/6760685/creating-a-singleton-in-python
"""
class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class Drawer(metaclass=Singleton):
    def __init__(self):
        # Создаем игру и окно
        pygame.init()
        pygame.mixer.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("My Game")
        self.clock = pygame.time.Clock()

    def flip(self):
        # После отрисовки всего, переворачиваем экран
        pygame.display.flip()

    def scale2scr(self, x):
        return x*SCALE_FACTOR

    def vec2pos(self, p: aux.Point):
        x = self.scale2scr(p.x + FIELD_WIDTH/2)
        y = HEIGHT - self.scale2scr(p.y + FIELD_HEIGHT/2)
        return (x, y)

    def drawBall(self, pos: aux.Point):
        drwpos = self.vec2pos(pos)
        drwrad = self.scale2scr(50)
        pygame.draw.circle(self.screen, RED, drwpos, drwrad)

    def drawVec(self, start: aux.Point, end: aux.Point):
        drwstart = self.vec2pos(start)
        drwend = self.vec2pos(start + end)
        pygame.draw.line(self.screen, BLUE, drwstart, drwend)
