import json

import attr

from strategy_bridge.bus import DataWriter, DataBus
from strategy_bridge.common import config
from strategy_bridge.model.referee import RefereeCommand
from strategy_bridge.processors import BaseProcessor
from strategy_bridge.larcmacs.receiver import ZmqReceiver
from strategy_bridge.utils.debugger import debugger


@attr.s(auto_attribs=True)
class TmpRefereeCommandsCollector(BaseProcessor):

    max_records_to_persist: int = 30
    records_writer: DataWriter = attr.ib(init=False)
    receiver: ZmqReceiver = attr.ib(init=False)

    def initialize(self, data_bus: DataBus) -> None:
        super(TmpRefereeCommandsCollector, self).initialize(data_bus)
        self.records_writer = DataWriter(self.data_bus, config.REFEREE_COMMANDS_TOPIC, self.max_records_to_persist)
        self.receiver = ZmqReceiver(port=config.REFEREE_COMMANDS_SUBSCRIBE_PORT)

    @debugger
    def process(self):
        message = self.receiver.next_message()
        if not message:
            return
        parsed_message = json.loads(bytes(message))
        print(message)
        command = RefereeCommand(
            state=parsed_message['state'],
            commandForTeam=parsed_message['team'],
            isPartOfFieldLeft=parsed_message['is_left'],
        )
        self.records_writer.write(command)
