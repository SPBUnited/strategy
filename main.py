from strategy_bridge.common import config
from strategy_bridge.processors import VisionDetectionsCollector, RobotCommandsSender
from strategy_bridge.processors.referee_commands_collector import RefereeCommandsCollector
from strategy_bridge.runner import Runner

from bridge.processors.python_controller import MatlabController

if __name__ == '__main__':

    config.init_logging("./logs")

    # TODO: Move list of processors to config
    processors = [
        VisionDetectionsCollector(processing_pause=0.001),
        RefereeCommandsCollector(processing_pause=0.1),
        MatlabController(),
        RobotCommandsSender(processing_pause=0.001)
    ]

    runner = Runner(processors=processors)
    runner.run()
