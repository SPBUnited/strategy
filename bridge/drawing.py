"""
draw field with robots and trajectory
"""

from enum import Enum
from typing import Optional

from bridge import const
from bridge.auxiliary import aux


class ImageTopic(Enum):
    """Topic for commands to draw"""

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

        self.commands: list[Command] = []
        self.prints: list[tuple[tuple[float, float], str, tuple[int, int, int]]] = []

    def clear(self) -> None:
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
