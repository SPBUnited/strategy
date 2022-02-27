import typing

import attr
import zmq


@attr.s(auto_attribs=True, kw_only=True)
class ZmqReceiver:

    port: int

    def __attrs_post_init__(self) -> None:
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)

        self.socket.connect(f"tcp://localhost:{self.port}")
        self.socket.subscribe("")

    def next_message(self) -> typing.Optional[typing.ByteString]:
        try:
            return self.socket.recv(flags=zmq.NOBLOCK)
        except zmq.ZMQError:
            return None
