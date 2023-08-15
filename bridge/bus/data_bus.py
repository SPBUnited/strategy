import attr
import typing

from collections import deque

from bridge.bus.record import Record


@attr.s(auto_attribs=True)
class DataBus:
    def __attrs_post_init__(self):
        self.data: typing.Dict[str, deque[Record]] = {}

    def register_topic(self, topic_name: str, max_size: int) -> None:
        self.data[topic_name] = deque(maxlen=max_size)

    def write(self, topic_name: str, record: Record) -> None:
        self.data[topic_name].append(record)

    def read_all(self, topic_name) -> typing.List[Record]:
        data = self.data.get(topic_name)
        if not data:
            print(f"No data in topic {topic_name}")
            return []
        return list(data)

    def read_top(self, topic_name: str, count: int) -> typing.List[Record]:
        return self.read_all(topic_name)[-count:]

    def read_from_timestamp(self, topic_name: str, timestamp: float) -> typing.List[Record]:
        records = self.read_all(topic_name)
        valid_records = [r for r in records if r.timestamp > timestamp]
        return valid_records


data_bus = DataBus()
