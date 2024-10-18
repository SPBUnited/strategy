"""
Точка входа в стратегию
"""

# from strategy_bridge.processors import BoxFeedbackCollector
from strategy_bridge.processors.referee_commands_collector import (
    RefereeCommandsCollector,
)
from strategy_bridge.runner import Runner

from bridge import const
from bridge.processors.drawing_processor import Drawer
from bridge.processors.field_creator import FieldCreator
from bridge.processors.passes_explorer import ExplorePasses
from bridge.processors.python_controller import SSLController
from bridge.processors.referee_state_processor import RefereeStateProcessor, State
from bridge.processors.router_processor import CommandSink

if __name__ == "__main__":

    # config.init_logging("./logs")

    PROCESSORS = [
        FieldCreator(processing_pause=0.01),
        RefereeCommandsCollector(processing_pause=0.001),
        RefereeStateProcessor(),
        SSLController(
            ally_color=const.COLOR,
            processing_pause=const.Ts,
            reduce_pause_on_process_time=True,
            dbg_game_state=State.RUN,
        ),
        ExplorePasses(ally_color=const.COLOR),
        # SSLController(
        #     ally_color=const.Color.BLUE,
        #     processing_pause=const.Ts,
        #     reduce_pause_on_process_time=True,
        #     dbg_game_state=State.RUN,
        # ),
        Drawer(),
        CommandSink(processing_pause=0.001),
        # BoxFeedbackCollector(processing_pause=0.001),
    ]

    RUNNER = Runner(processors=PROCESSORS)
    RUNNER.run()
