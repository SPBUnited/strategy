"""
Модуль стратегии игры
"""

import time

import attr
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.model.referee import RefereeCommand
from strategy_bridge.processors import BaseProcessor
from strategy_bridge.utils.debugger import debugger

import bridge.processors.referee_state_processor as state_machine
from bridge import const
from bridge.auxiliary import aux, fld
from bridge.router import router
from bridge.strategy import strategy


@attr.s(auto_attribs=True)
class SSLController(BaseProcessor):
    """
    Процессор стратегии SSL
    """

    processing_pause: float = const.Ts
    reduce_pause_on_process_time: bool = True
    max_commands_to_persist: int = 20

    ally_color: const.Color = const.Color.BLUE

    dbg_game_status: strategy.GameStates = strategy.GameStates.TIMEOUT

    cur_time = time.time()
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
        self.referee_reader = DataReader(data_bus, config.REFEREE_COMMANDS_TOPIC)
        self.commands_sink_writer = DataWriter(data_bus, const.TOPIC_SINK, 20)
        self.image_writer = DataWriter(data_bus, const.IMAGE_TOPIC, 20)

        self.field = fld.Field(self.ally_color)
        self.router = router.Router(self.field)

        self.strategy = strategy.Strategy()

        # Referee fields
        self.state_machine = state_machine.StateMachine()
        self.cur_cmd_state = None
        self.wait_10_sec_flag = False
        self.wait_10_sec = 0.0
        self.wait_ball_moved_flag = False
        self.wait_ball_moved = aux.Point(0, 0)
        self.tmp = 0

    def get_last_referee_command(self) -> RefereeCommand:
        """
        Получить последнюю команду рефери
        """
        referee_commands = self.referee_reader.read_new()
        if referee_commands:
            return referee_commands[-1].content
        return RefereeCommand(-1, 0, False)

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

    def draw_image(self) -> None:
        """Send commands to drawer processor"""
        if self.field.ally_color == const.COLOR:
            for image in [
                self.field.strategy_image,
                self.field.router_image,
                self.field.path_image,
            ]:
                self.image_writer.write(image)
                image.clear()

    def control_loop(self) -> None:
        """
        Рассчитать стратегию, тактику и физику для роботов на поле
        """
        self.router.update(self.field)
        waypoints = self.strategy.process(self.field)

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.router.get_route(i).clear()
            self.router.set_dest(i, waypoints[i], self.field)
        self.router.reroute(self.field)

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.router.get_route(i).go_route(self.field.allies[i], self.field)

    def control_assign(self) -> None:
        """
        Определить связь номеров роботов с каналами управления
        """
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if self.field.allies[i].is_used():
                self.field.allies[i].color = self.ally_color
                self.commands_sink_writer.write(self.field.allies[i])
                self.field.allies[i].clear_fields()

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
        cur_cmd = self.get_last_referee_command()
        cur_state, cur_active = self.state_machine.get_state()
        self.router.avoid_ball(False)

        if cur_cmd.state == -1:
            return

        if cur_state == state_machine.State.STOP or (cur_active not in [const.Color.ALL, self.field.ally_color]):
            self.router.avoid_ball(True)

        if cur_cmd.state != self.cur_cmd_state:
            self.state_machine.make_transition(cur_cmd.state)
            self.state_machine.active_team(cur_cmd.commandForTeam)
            self.cur_cmd_state = cur_cmd.state
            cur_state, _ = self.state_machine.get_state()

            self.wait_10_sec_flag = False
            self.wait_ball_moved_flag = False

            if cur_state in [
                state_machine.State.KICKOFF,
                state_machine.State.FREE_KICK,
                state_machine.State.PENALTY,
            ]:
                self.wait_10_sec_flag = True
                self.wait_10_sec = time.time()
            if cur_state in [
                state_machine.State.KICKOFF,
                state_machine.State.FREE_KICK,
            ]:
                self.wait_ball_moved_flag = True
                self.wait_ball_moved = self.field.ball.get_pos()
        else:
            if self.wait_10_sec_flag and time.time() - self.wait_10_sec > 10:
                self.state_machine.make_transition_(state_machine.Command.PASS_10_SECONDS)
                self.state_machine.active_team(0)
                self.wait_10_sec_flag = False
                self.wait_ball_moved_flag = False
            if self.wait_ball_moved_flag and self.field.is_ball_moves():
                self.state_machine.make_transition_(state_machine.Command.BALL_MOVED)
                self.state_machine.active_team(0)
                self.wait_10_sec_flag = False
                self.wait_ball_moved_flag = False
        self.tmp += 1

        cur_state, cur_active = self.state_machine.get_state()
        self.strategy.change_game_state(cur_state, cur_active)

    @debugger
    def process(self) -> None:
        """
        Выполнить цикл процессора
        """

        self.delta_t = time.time() - self.cur_time
        self.cur_time = time.time()

        self.read_vision()
        # self.process_referee_cmd()
        self.get_pass_points()
        self.control_loop()

        self.control_assign()
        self.draw_image()

        print("Strategy long:", time.time() - self.cur_time)
