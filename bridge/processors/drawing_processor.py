"""Processor for drawing"""

import typing

import attr
import pygame
from strategy_bridge.bus import DataBus, DataReader
from strategy_bridge.processors import BaseProcessor

from bridge import const, drawing
from bridge.auxiliary import aux, fld


@attr.s(auto_attribs=True)
class Drawer(BaseProcessor):
    """Class for drawing"""

    processing_pause: typing.Optional[float] = 1 / 30
    reduce_pause_on_process_time: bool = True
    commands_sink_reader: DataReader = attr.ib(init=False)

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализация
        """
        super().initialize(data_bus)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.image_reader = DataReader(data_bus, const.IMAGE_TOPIC)

        self.width = 1200
        self.heigh = int(self.width / const.GOAL_DX * const.FIELD_DY)
        goal_dx, goal_dy = abs(const.GOAL_DX), abs(const.FIELD_DY)
        self.scale = min(self.width / 2 / goal_dx, self.heigh / 2 / goal_dy)
        self.size_x = goal_dx * self.scale * 2
        self.size_y = goal_dy * self.scale * 2

        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((self.width, self.heigh))
        pygame.display.set_caption("Football Field")
        self.middle_x, self.middle_y = self.screen.get_size()
        self.middle_x = round(self.middle_x / 2)
        self.middle_y = round(self.middle_y / 2)
        self.upper_border = self.middle_y - goal_dy * self.scale
        self.lower_border = self.middle_y + goal_dy * self.scale
        self.left_border = self.middle_x - goal_dx * self.scale
        self.right_border = self.middle_x + goal_dx * self.scale

        self.images: list[tuple[drawing.Image, CheckBox]] = []
        box_pos = (10, 10)
        for topic in drawing.ImageTopic:
            if topic == drawing.ImageTopic.FIELD:
                continue
            self.images += [(drawing.Image(topic), CheckBox(self.screen, box_pos, topic.name))]
            box_pos = (box_pos[0], box_pos[1] + 20)
        self.images[drawing.ImageTopic.STRATEGY.value][1].is_pressed = True

        self.timers_box = CheckBox(self.screen, (self.width - 26, 10), "DELAY | TPS", text_side="left", is_pressed=True)

        self.font = pygame.font.SysFont("Open Sans", 25)

    def process(self) -> None:

        message_img = self.image_reader.read_new()
        message_fld = self.field_reader.read_last()
        if message_img is None or message_fld is None:
            return

        field: fld.Field = message_fld.content
        for ext_image in message_img:
            image: drawing.Image = ext_image.content
            if image.topic is not None and image.topic != drawing.ImageTopic.FIELD:
                self.images[image.topic.value] = (
                    image,
                    self.images[image.topic.value][1],
                )

        field_image = field.field_image

        for rbt in field.get_blu_team():
            if rbt.is_used():
                field_image.draw_robot(rbt.get_pos(), rbt.get_angle())
                field_image.print(rbt.get_pos(), str(rbt.r_id))
        for rbt in field.get_yel_team():
            if rbt.is_used():
                field_image.draw_robot(rbt.get_pos(), rbt.get_angle(), (255, 255, 0))
                field_image.print(rbt.get_pos(), str(rbt.r_id), (0, 0, 0))

        field_image.draw_dot(field.ball.get_pos(), (255, 0, 0), const.BALL_R)
        if field.ball_start_point is not None:
            field_image.draw_dot(field.ball_start_point, (255, 0, 0), const.BALL_R // 2)

        self.update_boxes()

        for image_box in self.images:
            if image_box[1].is_pressed:
                for command in image_box[0].commands:
                    cmd = self.scale_dots(command)
                    self.complete_command(cmd)

                for prnt in image_box[0].prints:
                    pos = self.cord_to_pixels(prnt[0])
                    self.print_text(pos, prnt[1], prnt[2])

        if self.timers_box.is_pressed:
            images = [field_image] + [img[0] for img in self.images]
            self.draw_timers(images)

        for command in field_image.commands:
            cmd = self.scale_dots(command)
            self.complete_command(cmd)
        for prnt in field_image.prints:
            pos = self.cord_to_pixels(prnt[0])
            self.print_text(pos, prnt[1], prnt[2])

        pygame.display.flip()

        self.draw_field()
        for goal in [field.ally_goal, field.enemy_goal]:
            new_dots = []
            for dot in goal.hull:
                new_dots.append(dot * self.scale + aux.Point(self.middle_x, self.middle_y))

            for i, _ in enumerate(new_dots):
                pygame.draw.line(
                    self.screen,
                    (255, 255, 255),
                    (new_dots[i - 1].x, new_dots[i - 1].y),
                    (new_dots[i].x, new_dots[i].y),
                    2,
                )
            goal_up = goal.up * self.scale + aux.Point(self.middle_x, self.middle_y)
            goal_down = goal.down * self.scale + aux.Point(self.middle_x, self.middle_y)
            pygame.draw.line(
                self.screen,
                (255, 255, 255),
                (goal_up.x, goal_up.y),
                (goal_down.x, goal_down.y),
                10,
            )

    def update_boxes(self) -> None:
        """check clicks on boxes and render its"""
        events = pygame.event.get()
        for event in events:
            for img_box in self.images:
                img_box[1].update(event)
            self.timers_box.update(event)

        for img_box in self.images:
            img_box[1].render_checkbox()
            self.timers_box.render_checkbox()

    def draw_timers(self, images: list[drawing.Image]) -> None:
        """render and draw on screen all of timers"""
        height = 10
        boarder_surf = self.font.render("|", True, (255, 255, 255))

        tps_x = self.width - 34 - self.font.size("TPS")[0]
        boarder_x = self.width - 34 - self.font.size("| TPS")[0]
        delay_x = self.width - 34 - self.font.size(" | TPS")[0]

        for image in images:
            if image.topic == drawing.ImageTopic.PATH_GENERATION:
                continue
            height += 20
            if image.timer.delay_warning:
                delay_color = (255, 0, 0)
            else:
                delay_color = (255, 255, 255)
            delay_text = f"{image.timer.delay * 1000:.1f}"
            delay_surf = self.font.render(delay_text, True, delay_color)
            delay_pos = (
                delay_x - self.font.size(delay_text)[0],
                height,
            )
            self.screen.blit(delay_surf, delay_pos)

            self.screen.blit(boarder_surf, (boarder_x, height))

            if image.timer.tps_warning:
                tps_color = (255, 0, 0)
            else:
                tps_color = (255, 255, 255)
            tps_text = f"{image.timer.tps:5.1f}"
            tps_surf = self.font.render(tps_text, True, tps_color)
            tps_pos = (
                tps_x,
                height,
            )
            self.screen.blit(tps_surf, tps_pos)

    def draw_field(self) -> None:
        """
        draw green field and white lines
        """

        back_color = (128, 128, 128)
        field_color = (20, 178, 10)
        line_color = (255, 255, 255)
        self.screen.fill(back_color)
        pygame.draw.rect(
            self.screen,
            field_color,
            (self.left_border, self.upper_border, self.size_x, self.size_y),
        )  # Поле
        pygame.draw.rect(
            self.screen,
            line_color,
            (self.left_border, self.upper_border, self.size_x, self.size_y),
            2,
        )  # Обводка поля
        pygame.draw.line(
            self.screen,
            line_color,
            (self.left_border, self.middle_y),
            (self.right_border, self.middle_y),
            2,
        )  # Горизонтальная линия
        pygame.draw.line(
            self.screen,
            line_color,
            (self.middle_x, self.upper_border),
            (self.middle_x, self.lower_border),
            2,
        )  # Вертикальная линия
        pygame.draw.circle(self.screen, line_color, (self.middle_x, self.middle_y), 50, 2)  # Круг в центре

    def complete_command(self, command: drawing.Command) -> None:
        """Draw something on a screen by command"""
        if len(command.dots) == 1:
            pygame.draw.circle(self.screen, command.color, command.dots[0], command.size * self.scale)
        elif len(command.dots) == 2:
            pygame.draw.line(
                self.screen,
                command.color,
                command.dots[0],
                command.dots[1],
                round(command.size),
            )
        elif len(command.dots) > 2:
            for i, _ in enumerate(command.dots):
                pygame.draw.line(
                    self.screen,
                    command.color,
                    command.dots[i - 1],
                    command.dots[i],
                    round(command.size),
                )

    def print_text(self, pos: tuple[int, int], text: str, color: tuple[int, int, int]) -> None:
        """Print text"""
        font_surf = self.font.render(text, True, color)
        width, heigh = self.font.size(text)
        font_pos = (
            pos[0] - width / 2,
            pos[1] - heigh / 2,
        )
        self.screen.blit(font_surf, font_pos)

    def scale_dots(self, command: drawing.Command) -> drawing.Command:
        """Scale dots from coordinates to pixels"""
        scaled_command = drawing.Command(command.color, command.dots.copy(), command.size)
        for i, _ in enumerate(scaled_command.dots):
            scaled_command.dots[i] = self.cord_to_pixels(scaled_command.dots[i])
        return scaled_command

    def cord_to_pixels(self, point: tuple[float, float]) -> tuple[int, int]:
        """Scale single dot from coordinates to pixels"""
        return (
            int(point[0] * self.scale + self.middle_x),
            int(-point[1] * self.scale + self.middle_y),
        )


class CheckBox:
    """class for clickable box"""

    def __init__(
        self,
        surface: pygame.Surface,
        pos: tuple[int, int],
        text: str,
        text_side: str = "right",
        is_pressed: bool = False,
    ) -> None:
        """text_side = 1 if text righter than box, -1 if lefter"""
        self.surface = surface

        self.x: int = pos[0]
        self.y: int = pos[1]
        self.text: str = text

        self.is_pressed: bool = is_pressed

        self.checkbox_size = 16
        self.checkbox_obj = pygame.Rect(self.x, self.y, self.checkbox_size, self.checkbox_size)
        self.font = pygame.font.SysFont("Open Sans", 25)
        self.font_surf = self.font.render(self.text, True, (255, 255, 255))
        width, heigh = self.font.size(self.text)
        if text_side == "right":
            self.font_pos = (
                self.x + self.checkbox_size * 1.5,
                self.y - heigh / 2 + self.checkbox_size / 2,
            )
        elif text_side == "left":
            self.font_pos = (
                self.x - self.checkbox_size * 0.5 - width,
                self.y - heigh / 2 + self.checkbox_size / 2,
            )
        else:
            self.font_pos = (self.x, self.y)  # Wrong arg "text_side"

    def render_checkbox(self) -> None:
        """render checkbox"""
        pygame.draw.rect(self.surface, (230, 230, 230), self.checkbox_obj)
        pygame.draw.rect(self.surface, (0, 0, 0), self.checkbox_obj, 1)

        if self.is_pressed:
            offset = 2
            obj = pygame.Rect(
                self.x + offset,
                self.y + offset,
                self.checkbox_size - offset * 2,
                self.checkbox_size - offset * 2,
            )
            pygame.draw.rect(self.surface, (0, 0, 0), obj)

        self.surface.blit(self.font_surf, self.font_pos)

    def update(self, event_object: pygame.event.Event) -> bool:
        """check click on box"""
        if event_object.type == pygame.MOUSEBUTTONDOWN:
            x, y = pygame.mouse.get_pos()
            pos_x, pos_y, w, heigh = self.checkbox_obj
            if pos_x < x < pos_x + w and pos_y < y < pos_y + heigh:
                if self.is_pressed:
                    self.is_pressed = False
                else:
                    self.is_pressed = True

        return self.is_pressed
