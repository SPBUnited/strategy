import typing

import attr


@attr.s(auto_attribs=True)
class Record:
    content: typing.Any
    timestamp: float
