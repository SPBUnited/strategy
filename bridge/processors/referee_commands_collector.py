import json

import attr

from bridge.bus import DataWriter
from bridge.common import config
from bridge.model.referee import RefereeCommand
from bridge.processors import BaseProcessor
from bridge.zmq.receiver import ZmqReceiver


@attr.s(auto_attribs=True)
class RefereeCommandsCollector(BaseProcessor):

    max_records_to_persist: int = 30
    records_writer: DataWriter = attr.ib(init=False)
    receiver: ZmqReceiver = attr.ib(init=False)

    def __attrs_post_init__(self) -> None:
        self.records_writer = DataWriter(config.REFEREE_COMMANDS_TOPIC, self.max_records_to_persist)
        self.receiver = ZmqReceiver(port=config.REFEREE_COMMANDS_SUBSCRIBE_PORT)

    async def process(self):
        message = self.receiver.next_message()
        parsed_message = json.loads(bytes(message))
        command = RefereeCommand(
            state=parsed_message['state'],
            commandForTeam=parsed_message['team'],
            isPartOfFieldLeft=parsed_message['is_left'],
        )
        self.records_writer.write(command)
        from datetime import datetime
        time = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        with open("tmp/referee_collector.txt", "a") as f:
            f.write(time + "\n")
