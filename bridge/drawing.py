"""
draw field with robots and trajectory
"""

from enum import Enum
from typing import Optional

from bridge import const
from bridge.auxiliary import aux


class ImageTopic(Enum):
    """Topic for commands to draw"""

    FIELD = -1
    STRATEGY = 0
    ROUTER = 1
    PATH_GENERATION = 2
    PASSES = 3


class Command:
    """Command to draw something"""

    def __init__(
        self,
        color: tuple[int, int, int],
        dots: list[tuple[float, float]],
        size: float,
    ) -> None:
        self.color = color
        self.dots = dots
        self.size = size


class Image:
    """
    class with image's specs
    """

    def __init__(self, topic: Optional[ImageTopic] = None) -> None:
        self.topic: Optional[ImageTopic] = topic
        self.timer: FeedbackTimer = FeedbackTimer(0, 1, 1)

        self.commands: list[Command] = []
        self.prints: list[tuple[tuple[float, float], str, tuple[int, int, int]]] = []

    def clear(self) -> None:
        """clear the image"""
        self.commands = []
        self.prints = []

    def draw_dot(
        self,
        pos: aux.Point,
        color: tuple[int, int, int] = (255, 0, 0),
        size_in_mms: float = 10,
    ) -> None:
        """draw single point"""
        self.commands.append(Command(color, [(pos.x, pos.y)], size_in_mms))

    def draw_line(
        self,
        dot1: aux.Point,
        dot2: aux.Point,
        color: tuple[int, int, int] = (255, 255, 255),
        size_in_pixels: int = 2,
    ) -> None:
        """draw line"""
        new_dots = [(dot1.x, dot1.y), (dot2.x, dot2.y)]

        self.commands.append(Command(color, new_dots, size_in_pixels))

    def draw_poly(
        self,
        dots: list[aux.Point],
        color: tuple[int, int, int] = (255, 255, 255),
        size_in_pixels: int = 2,
    ) -> None:
        """Connect nearest dots with line"""
        new_dots: list[tuple[float, float]] = []
        for dot in dots:
            new_dots.append((dot.x, dot.y))

        self.commands.append(Command(color, new_dots, size_in_pixels))

    def draw_robot(
        self,
        pos: aux.Point,
        angle: float = 0.0,
        color: tuple[int, int, int] = (0, 0, 255),
    ) -> None:
        """draw robot"""
        eye_vec = aux.rotate(aux.RIGHT, angle) * 150
        self.draw_dot(pos, color, const.ROBOT_R)
        self.draw_line(pos, pos + eye_vec, color, 2)

    def draw_pixel(self, pos: tuple[int, int], color: tuple[int, int, int] = (255, 0, 0)) -> None:
        """draw single point"""
        self.commands.append(Command(color, [(pos[0], pos[1])], 1))

    def print(self, pos: aux.Point, text: str, color: tuple[int, int, int] = (255, 255, 255)) -> None:
        """print text"""
        self.prints.append(((pos.x, pos.y), text, color))


class FeedbackTimer:
    """Class for timers on screen"""

    def __init__(self, time: float, delay_lim: float, tps_lim: float) -> None:
        """
        delay_lim - limit of min process long
        tps_lim - limit of min tics per second for processor
        """

        self.delay = 0.0
        self.delay_lim = delay_lim  # in seconds
        self.delay_timer = 0.0
        self.delay_warning = False

        self.tps = 0.0
        self.tps_lim = tps_lim  # in ticks per seconds
        self.tps_timer = 0.0
        self.tps_warning = False

        self.memory: list[float] = []
        self.memory_long = 2.0  # in seconds

        self.last_update = time

    def start(self, time: float) -> None:
        """Start timer when processor starts"""
        self.last_update = time
        self.clean_memory()
        self.memory.append(time)
        if len(self.memory) > 1:
            self.tps = len(self.memory) / (self.memory[-1] - self.memory[0])
            if self.tps < self.tps_lim:
                self.tps_timer = time

    def end(self, time: float) -> None:
        """End timer when processor ends"""
        self.delay = time - self.last_update
        if self.delay > self.delay_lim:
            self.delay_timer = time

        self.delay_warning = time - self.delay_timer < self.memory_long
        self.tps_warning = time - self.tps_timer < self.memory_long

    def clean_memory(self) -> None:
        """Clean old data from 'self.memory'"""
        memory = self.memory.copy()
        for data in self.memory:
            if data < self.last_update - self.memory_long:
                memory.pop(0)
            else:
                break
        self.memory = memory
