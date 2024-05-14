import attr
import zmq
import typing

from bridge.bus import DataReader
from bridge.common import config
from bridge.processors import BaseProcessor


@attr.s(auto_attribs=True)
class RobotCommandsSender(BaseProcessor):

    processing_pause: typing.Optional[float] = 0.001
    commands_reader: DataReader = attr.ib(init=False, default=DataReader(config.ROBOT_COMMANDS_TOPIC))

    def __attrs_post_init__(self) -> None:
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{config.COMMANDS_PUBLISH_PORT}")

    async def process(self):
        commands = self.commands_reader.read_new()
        for command in commands:
            self.socket.send(command)
        from datetime import datetime
        time = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        with open("tmp/commands_sender.txt", "a") as f:
            f.write(time + "\n")
