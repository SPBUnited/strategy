from bridge.common import config
from bridge.processors import VisionDetectionsCollector, RobotCommandsSender
from bridge.processors.python_controller import MatlabController
from bridge.processors.referee_commands_collector import RefereeCommandsCollector
from bridge.runner import Runner


if __name__ == '__main__':

    config.init_logging()

    # TODO: Move list of processors to config
    processors = [
        VisionDetectionsCollector(),
        RefereeCommandsCollector(),
        MatlabController(),
        RobotCommandsSender()
    ]

    runner = Runner(processors=processors)
    runner.run()
