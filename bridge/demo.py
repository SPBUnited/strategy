from bridge.bus import data_bus
from bridge.common import config
from bridge.runner import do_run

do_run()


ssl_packages = data_bus.read_all(config.VISION_DETECTIONS_TOPIC)
referee_commands = data_bus.read_all(config.REFEREE_COMMANDS_TOPIC)
control = data_bus.read_all(config.ROBOT_COMMANDS_TOPIC)
