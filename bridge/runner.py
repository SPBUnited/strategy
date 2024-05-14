import asyncio
import threading
import typing

import attr

from bridge.processors import BaseProcessor


@attr.s(auto_attribs=True, kw_only=True)
class Runner:
    processors: typing.List[BaseProcessor]

    def run(self):
        loop = asyncio.get_event_loop()
        t = threading.Thread(target=self.loop_in_thread, args=(loop,))
        t.start()

    async def run_tasks(self):
        tasks = [asyncio.create_task(processor.run()) for processor in self.processors]
        await asyncio.gather(*tasks)

    def loop_in_thread(self, loop):
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.run_tasks())


# loop = asyncio.get_event_loop()
# loop.create_task(matlab_controller.run())
# loop.create_task(collector.run())
# loop.run_forever()
#
# def loop_in_thread1(loop):
#     asyncio.set_event_loop(loop)
#     # loop.run_until_complete(run_all())
#     loop.run_until_complete(matlab_controller.run())
#
# loop2 = asyncio.get_event_loop()
# t2 = threading.Thread(target=loop_in_thread1, args=(loop1,))
# t2.start()
