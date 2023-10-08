from strategy_bridge.common import config
from strategy_bridge.processors import VisionDetectionsCollector, RobotCommandsSender
from strategy_bridge.processors.referee_commands_collector import RefereeCommandsCollector
from strategy_bridge.runner import Runner

from bridge.processors.python_controller import SSLController
from bridge.processors.robot_command_sink import CommandSink
import bridge.processors.const as const
import bridge.processors.strategy as strategy

if __name__ == '__main__':

    config.init_logging("./logs")

    # TODO: Move list of processors to config
    processors = [
        VisionDetectionsCollector(processing_pause=0.001, should_debug=True),
        RefereeCommandsCollector(processing_pause=0.001, should_debug=True),
        SSLController(
            our_color='b',
            should_debug=True,
            processing_pause=const.Ts,
            reduce_pause_on_process_time=True,
            dbg_game_status = strategy.GameStates.RUN,
            dbg_state = strategy.States.DEBUG),
        # MatlabController(
        #     our_color='y',
        #     should_debug=True,
        #     processing_pause=const.Ts,
        #     reduce_pause_on_process_time=True,
        #     dbg_game_status = strategy.GameStates.RUN,
        #     dbg_state = strategy.States.DEFENCE),
        CommandSink(processing_pause = const.Ts/2, should_debug=True),
        RobotCommandsSender(processing_pause = const.Ts/2, should_debug=True)
    ]

    runner = Runner(processors=processors)
    runner.run()
