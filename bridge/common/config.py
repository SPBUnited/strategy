import logging
import os
from logging.handlers import TimedRotatingFileHandler


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


VISION_DETECTIONS_SUBSCRIBE_PORT = 4242
REFEREE_COMMANDS_SUBSCRIBE_PORT = 4243
COMMANDS_PUBLISH_PORT = 5667

VISION_DETECTIONS_TOPIC = "vision-detections"
REFEREE_COMMANDS_TOPIC = "referee-commands"
ROBOT_COMMANDS_TOPIC = "robot-commands"

LOG_FILE_NAME = "strategy-bridge.log"
CONFIG_FILE_NAME = "bridge.yml"

LOG_PATH = os.path.join(PROJECT_ROOT, "logs")
CONFIG_PATH = os.path.join(PROJECT_ROOT, "conf")
MATLAB_SCRIPTS_PATH = os.path.join(PROJECT_ROOT, "..", "MLscripts_func_main")


def init_logging(log_file_name=LOG_FILE_NAME):
    logging_format = "[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s"
    formatter = logging.Formatter(logging_format)

    file_handler = TimedRotatingFileHandler(filename=os.path.join(LOG_PATH, log_file_name), when='W0')
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)

    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(formatter)

    logging.getLogger().addHandler(console_handler)
    logging.getLogger().addHandler(file_handler)
    logging.getLogger().setLevel(logging.INFO)
