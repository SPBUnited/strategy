import typing
from datetime import datetime

import attr

from bridge.bus import data_bus, Record


@attr.s(auto_attribs=True)
class DataWriter:

    write_topic_name: str
    max_persisted_records_count: int

    def __attrs_post_init__(self):
        data_bus.register_topic(self.write_topic_name, self.max_persisted_records_count)

    def write(self, content: typing.Any):
        record = Record(content, datetime.now().timestamp())
        data_bus.write(self.write_topic_name, record)
