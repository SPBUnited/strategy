"""
Модуль стратегии игры
"""

from time import time

import attr
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.processors import BaseProcessor
from strategy_bridge.utils.debugger import debugger

import bridge.router.waypoint as wp
from bridge import const, drawing
from bridge.auxiliary import fld
from bridge.processors.referee_state_processor import State
from bridge.strategy import strategy


class RobotCommand:
    """Command to control robot"""

    def __init__(self, r_id: int, color: const.Color, waypoint: wp.Waypoint) -> None:
        self.r_id: int = r_id
        self.color: const.Color = color
        self.waypoint: wp.Waypoint = waypoint


@attr.s(auto_attribs=True)
class SSLController(BaseProcessor):
    """
    Процессор стратегии SSL
    """

    processing_pause: float = const.Ts
    reduce_pause_on_process_time: bool = True
    max_commands_to_persist: int = 20

    ally_color: const.Color = const.Color.BLUE

    dbg_game_state: State = State.RUN

    cur_time = time()
    delta_t = 0.0

    ctrl_mapping = const.CONTROL_MAPPING
    count_halt_cmd = 0

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализировать контроллер
        """
        super().initialize(data_bus)
        self.passes_reader = DataReader(data_bus, const.PASSES_TOPIC)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.gamestate_reader = DataReader(data_bus, const.GAMESTATE_TOPIC)

        self.robot_control_writer = DataWriter(data_bus, const.CONTROL_TOPIC, 50)
        self.image_writer = DataWriter(data_bus, const.IMAGE_TOPIC, 20)

        self.field = fld.Field(self.ally_color)
        self.field.strategy_image.timer = drawing.FeedbackTimer(time(), 0.05, 40)
        self.game_state: tuple[State, const.Color] = self.dbg_game_state, const.Color.ALL

        self.strategy = strategy.Strategy()
        self.waypoints: list[wp.Waypoint] = []

    def read_vision(self) -> None:
        """
        Прочитать новые пакеты из SSL-Vision
        """
        new_field = self.field_reader.read_last()
        if new_field is not None:
            updated_field = new_field.content
            self.field.update_field(updated_field)
        else:
            print("No new field")

    def get_pass_points(self) -> None:
        """
        Получить точки для пасов
        """
        points = self.passes_reader.read_last()
        if points is not None:
            points = points.content
            self.strategy.pass_points = points

    def process_referee_cmd(self) -> None:
        """Get referee commands"""
        new_state = self.gamestate_reader.read_last()
        if new_state is not None:
            self.game_state = new_state.content

    def control_loop(self) -> None:
        """
        Рассчитать стратегию, тактику и физику для роботов на поле
        """
        cur_state, cur_active = self.game_state

        self.strategy.change_game_state(cur_state, cur_active)
        self.waypoints = self.strategy.process(self.field)

    def control_assign(self) -> None:
        """Send commands to robots"""
        for robot in self.field.active_allies(True):
            message = RobotCommand(robot.r_id, robot.color, self.waypoints[robot.r_id])

            self.robot_control_writer.write(message)

    def send_image(self) -> None:
        """Send commands to drawer processor"""
        self.field.strategy_image.timer.end(time())
        if self.field.ally_color == const.COLOR:
            self.image_writer.write(self.field.strategy_image)
        self.field.clear_images()

    @debugger
    def process(self) -> None:
        """
        Выполнить цикл процессора
        """
        self.field.strategy_image.timer.start(time())

        self.read_vision()
        # self.process_referee_cmd()
        self.get_pass_points()
        self.control_loop()

        self.control_assign()
        self.send_image()

        print("Strategy long:", time() - self.cur_time)
