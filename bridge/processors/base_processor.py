import asyncio
import typing

import attr

from abc import ABC, abstractmethod


@attr.s(auto_attribs=True, kw_only=True)
class BaseProcessor(ABC):

    processing_pause: typing.Optional[int] = 1

    async def run(self) -> None:
        while True:
            self.process()
            if self.processing_pause:
                await asyncio.sleep(self.processing_pause)

    @abstractmethod
    def process(self) -> None:
        pass
