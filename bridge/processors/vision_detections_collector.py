import attr

from bridge.bus import DataWriter
from bridge.common import config
from bridge.processors import BaseProcessor
from bridge.pb.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from bridge.zmq.receiver import ZmqReceiver


@attr.s(auto_attribs=True)
class VisionDetectionsCollector(BaseProcessor):

    max_records_to_persist: int = 30
    records_writer: DataWriter = attr.ib(init=False)
    receiver: ZmqReceiver = attr.ib(init=False)

    def __attrs_post_init__(self) -> None:
        self.records_writer = DataWriter(config.VISION_DETECTIONS_TOPIC, self.max_records_to_persist)
        self.receiver = ZmqReceiver(port=config.VISION_DETECTIONS_SUBSCRIBE_PORT)
        self._ssl_converter = SSL_WrapperPacket()

    async def process(self):
        message = self.receiver.next_message()
        if not message:
            return
        package = self._ssl_converter.FromString(message)
        self.records_writer.write(package)
        from datetime import datetime
        time = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        with open("tmp/vision_collector.txt", "a") as f:
            f.write(time + "\n")
