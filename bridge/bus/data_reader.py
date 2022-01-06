import typing

import attr

from bridge.bus import data_bus, Record


@attr.s(auto_attribs=True)
class DataReader:

    read_topic_name: str
    last_read_message_timestamp: float = 0

    def read_new(self) -> typing.List[typing.Any]:
        records = data_bus.read_from_timestamp(self.read_topic_name, self.last_read_message_timestamp)
        if records:
            self.last_read_message_timestamp = records[-1].timestamp
        return [r.content for r in records]

    def read_all(self) -> typing.List[Record]:
        return data_bus.read_all(self.read_topic_name)
