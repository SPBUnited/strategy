"""Processor to get referee commands"""

import typing
from enum import Enum
from time import time
from typing import Optional

import attr
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.model.referee import RefereeCommand
from strategy_bridge.processors import BaseProcessor

from bridge import const
from bridge.auxiliary import aux, fld


class State(Enum):
    """Класс с состояниями игры"""

    HALT = 0
    TIMEOUT = 1
    STOP = 2
    PREPARE_KICKOFF = 3
    BALL_PLACEMENT = 4
    PREPARE_PENALTY = 5
    KICKOFF = 6
    FREE_KICK = 7
    PENALTY = 8
    RUN = 9


class Command(Enum):
    """Класс с командами от судей"""

    HALT = 0
    STOP = 1
    FORCE_START = 2
    TIMEOUT = 3
    PREPARE_KICKOFF = 5
    NORMAL_START = 6
    PREPARE_PENALTY = 7
    NORMAL_START2 = 8  # я не понимаю почему они разные ну ладно..
    FREE_KICK = 9
    CONTINUE = 10  # не факт, что правда
    BALL_PLACEMENT = 11

    BALL_MOVED = 101
    PASS_10_SECONDS = 102


CommandMap = {command.value: command for command in Command}


class StateMachine:
    """Machine witch get command and return next state"""

    def __init__(self, initial_state: State = State.HALT) -> None:
        self.__state = initial_state
        self.__transitions: dict = {}
        self.__active = const.Color.ALL

        self.add_transition(State.HALT, State.STOP, Command.STOP)
        self.add_transition(State.HALT, State.RUN, Command.FORCE_START)
        self.add_transition(State.HALT, State.FREE_KICK, Command.FREE_KICK)
        self.add_transition(State.HALT, State.PREPARE_KICKOFF, Command.PREPARE_KICKOFF)
        self.add_transition(State.HALT, State.PREPARE_PENALTY, Command.PREPARE_PENALTY)
        self.add_transition(State.HALT, State.KICKOFF, Command.NORMAL_START)

        for state in State:
            self.add_transition(state, State.HALT, Command.HALT)
            self.add_transition(state, State.STOP, Command.STOP)
            self.add_transition(state, State.TIMEOUT, Command.TIMEOUT)

        self.add_transition(State.STOP, State.PREPARE_KICKOFF, Command.PREPARE_KICKOFF)
        self.add_transition(State.STOP, State.BALL_PLACEMENT, Command.BALL_PLACEMENT)
        self.add_transition(State.STOP, State.PREPARE_PENALTY, Command.PREPARE_PENALTY)
        self.add_transition(State.STOP, State.FREE_KICK, Command.FREE_KICK)
        self.add_transition(State.STOP, State.RUN, Command.FORCE_START)

        self.add_transition(State.PREPARE_KICKOFF, State.KICKOFF, Command.NORMAL_START)

        self.add_transition(State.BALL_PLACEMENT, State.FREE_KICK, Command.CONTINUE)
        self.add_transition(State.BALL_PLACEMENT, State.STOP, Command.STOP)

        self.add_transition(State.PREPARE_PENALTY, State.PENALTY, Command.NORMAL_START2)

        self.add_transition(State.PENALTY, State.STOP, Command.PASS_10_SECONDS)

        self.add_transition(State.KICKOFF, State.RUN, Command.PASS_10_SECONDS)
        self.add_transition(State.KICKOFF, State.RUN, Command.BALL_MOVED)

        self.add_transition(State.FREE_KICK, State.RUN, Command.PASS_10_SECONDS)
        self.add_transition(State.FREE_KICK, State.RUN, Command.BALL_MOVED)

        self.add_transition(State.RUN, State.STOP, Command.STOP)

    def add_transition(self, from_state: State, to_state: State, transition: Command) -> None:
        """Add new transition from state"""
        if from_state not in self.__transitions:
            self.__transitions[from_state] = {}
        self.__transitions[from_state][transition] = to_state

    def make_transition(self, transition: int) -> None:
        """Make a transition (for user)"""
        self.make_transition_(CommandMap.get(transition))

    def make_transition_(self, transition: Optional[Command]) -> None:
        """Make a transition (for the program)"""
        if transition in self.__transitions[self.__state]:
            self.__state = self.__transitions[self.__state][transition]
        else:
            raise ValueError(f"No transition '{transition}' from state '{self.__state}'")

    def get_possible_transitions(self) -> list:
        """Returns a list with all possible transitions"""
        return list(self.__transitions[self.__state].keys()) if self.__state in self.__transitions else []

    def active_team(self, num: int) -> None:
        """Set active team"""
        if num == 0:
            self.__active = const.Color.ALL
        elif num == 1:
            self.__active = const.Color.BLUE
        elif num == 2:
            self.__active = const.Color.YELLOW

    def get_state(self) -> tuple[State, const.Color]:
        """Returns the current state"""
        return self.__state, self.__active

    def __str__(self) -> str:
        return f"State: {self.__state}, Active: {self.__active}"


@attr.s(auto_attribs=True)
class RefereeStateProcessor(BaseProcessor):
    """Class to work with referee commands"""

    processing_pause: typing.Optional[float] = 0.001
    reduce_pause_on_process_time: bool = False

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализация
        """
        super().initialize(data_bus)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.referee_reader = DataReader(data_bus, config.REFEREE_COMMANDS_TOPIC)

        self.gamestate_writer = DataWriter(data_bus, const.GAMESTATE_TOPIC, 1)

        self.field = fld.Field(const.COLOR)

        # Referee fields
        self.state_machine = StateMachine()
        self.cur_cmd_state = None
        self.wait_10_sec_flag = False
        self.wait_10_sec = 0.0
        self.wait_ball_moved_flag = False
        self.wait_ball_moved = aux.Point(0, 0)

    def process(self) -> None:
        """
        Метод обратного вызова процесса
        """
        new_field = self.field_reader.read_last()
        if new_field is not None:
            self.field = new_field.content

        new_commands = self.referee_reader.read_new()

        for new_command in new_commands:
            cur_cmd: RefereeCommand = new_command.content

            cur_state, _ = self.state_machine.get_state()

            if cur_cmd.state != self.cur_cmd_state:
                self.state_machine.make_transition(cur_cmd.state)
                self.state_machine.active_team(cur_cmd.commandForTeam)
                self.cur_cmd_state = cur_cmd.state
                cur_state, _ = self.state_machine.get_state()

                self.wait_10_sec_flag = False
                self.wait_ball_moved_flag = False

                if cur_state in [
                    State.KICKOFF,
                    State.FREE_KICK,
                    State.PENALTY,
                ]:
                    self.wait_10_sec_flag = True
                    self.wait_10_sec = time()
                if cur_state in [
                    State.KICKOFF,
                    State.FREE_KICK,
                ]:
                    self.wait_ball_moved_flag = True
            else:
                if self.wait_10_sec_flag and time() - self.wait_10_sec > 10:
                    self.state_machine.make_transition_(Command.PASS_10_SECONDS)
                    self.state_machine.active_team(0)
                    self.wait_10_sec_flag = False
                    self.wait_ball_moved_flag = False
                if self.wait_ball_moved_flag and self.field.is_ball_moves():
                    self.state_machine.make_transition_(Command.BALL_MOVED)
                    self.state_machine.active_team(0)
                    self.wait_10_sec_flag = False
                    self.wait_ball_moved_flag = False

            self.gamestate_writer.write(self.state_machine.get_state())
