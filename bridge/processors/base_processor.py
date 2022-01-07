import asyncio
import logging
import typing

import attr

from abc import ABC, abstractmethod


logger = logging.getLogger(__name__)


@attr.s(auto_attribs=True, kw_only=True)
class BaseProcessor(ABC):

    processing_pause: typing.Optional[int] = 1

    async def run(self) -> None:
        logger.info(f"Running processor: {self.__class__.__name__}")
        while True:
            await self.process()
            if self.processing_pause:
                await asyncio.sleep(self.processing_pause)

    @abstractmethod
    async def process(self) -> None:
        pass
