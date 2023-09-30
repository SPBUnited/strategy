"""
Модуль для визуального дебага и графической отрисовки всяких векторов и точек
"""

import pygame

import bridge.processors.auxiliary as aux

WIDTH = 600
HEIGHT = 400
FPS = 30

FIELD_WIDTH = 9000
FIELD_HEIGHT = 6000
SCALE_FACTOR = WIDTH/FIELD_WIDTH

# Задаем цвета
WHITE = (255,230,210)
YELLOW = (255,180,0)
BLUE = (50,180,255)
GREEN = (10,200,50)
VIOLET = (204,102,204)
RED = (199,59,50)
BROWN = (153,95,61)
BLACK = (30,40,50)

FIELD_WHITE = (255, 255, 255)
FIELD_GREEN = (8, 132, 20)
BALL_RED = (255, 80, 0)
BOT_YELLOW = (255, 255, 0)
BOT_BLUE = (0, 0, 255)

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
        pass

    def disable(self):
        self.enabled = False

    def enable(self):
        # Создаем игру и окно
        pygame.init()
        pygame.mixer.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("My Game")
        self.clock = pygame.time.Clock()
        self.enabled = True

    def flip(self):
        if not self.enabled: return

        # После отрисовки всего, переворачиваем экран
        pygame.display.flip()

    def scale2scr(self, x):
        return x*SCALE_FACTOR

    def vec2pos(self, p: aux.Point):
        x = self.scale2scr(p.x + FIELD_WIDTH/2)
        y = HEIGHT - self.scale2scr(p.y + FIELD_HEIGHT/2)
        return (x, y)

    def drawBall(self, pos: aux.Point):
        if not self.enabled: return

        drwpos = self.vec2pos(pos)
        drwrad = self.scale2scr(50)
        pygame.draw.circle(self.screen, BALL_RED, drwpos, drwrad)

    def drawAbsVec(self, start: aux.Point, end: aux.Point, color = BLUE):
        if not self.enabled: return

        drwstart = self.vec2pos(start)
        drwend = self.vec2pos(end)
        pygame.draw.line(self.screen, color, drwstart, drwend)

    def drawDVec(self, start: aux.Point, delta: aux.Point, color = BLUE):
        if not self.enabled: return

        drwstart = self.vec2pos(start)
        drwend = self.vec2pos(start + delta)
        pygame.draw.line(self.screen, color, drwstart, drwend)

    def drawBot(self, pos: aux.Point, angle: float, color):
        if not self.enabled: return

        COLOR = BOT_BLUE if color == 'b' else BOT_YELLOW
        drwpos = self.vec2pos(pos)
        drwrad = self.scale2scr(180/2)
        pygame.draw.circle(self.screen, COLOR, drwpos, drwrad)

    def drawField(self):
        if not self.enabled: return

        self.screen.fill(FIELD_GREEN)
        # field outline
        tlpos = self.vec2pos(aux.Point(-FIELD_WIDTH/2, FIELD_HEIGHT/2))
        fsize = (self.scale2scr(FIELD_WIDTH), self.scale2scr(FIELD_HEIGHT))
        pygame.draw.rect(self.screen, FIELD_WHITE, (tlpos, fsize), 1)

        # center lines
        top = self.vec2pos(aux.Point(0, FIELD_HEIGHT/2))
        bottom = self.vec2pos(aux.Point(0, -FIELD_HEIGHT/2))
        left = self.vec2pos(aux.Point(-FIELD_WIDTH/2, 0))
        right = self.vec2pos(aux.Point(FIELD_WIDTH/2, 0))
        pygame.draw.line(self.screen, FIELD_WHITE, top, bottom, 1)
        pygame.draw.line(self.screen, FIELD_WHITE, left, right, 1)

        # center circle
        center = self.vec2pos(aux.Point(0, 0))
        rad = self.scale2scr(500)
        pygame.draw.circle(self.screen, FIELD_WHITE, center, rad, 1)

        # defence rects
        # left
        tlpos = self.vec2pos(aux.Point(-FIELD_WIDTH/2, 1000))
        fsize = (self.scale2scr(1000), self.scale2scr(2000))
        pygame.draw.rect(self.screen, FIELD_WHITE, (tlpos, fsize), 1)
        # right
        tlpos = self.vec2pos(aux.Point(FIELD_WIDTH/2 - 1000, 1000))
        fsize = (self.scale2scr(1000), self.scale2scr(2000))
        pygame.draw.rect(self.screen, FIELD_WHITE, (tlpos, fsize), 1)
