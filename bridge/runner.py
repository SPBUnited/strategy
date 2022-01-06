import asyncio
import threading

from bridge.processors import VisionDetectionsCollector, RobotCommandsSender

from bridge.common import config
from bridge.processors.matlab_controller import MatlabController
from bridge.processors.referee_commands_collector import RefereeCommandsCollector

config.init_logging()

vision_collector = VisionDetectionsCollector()
referee_collector = RefereeCommandsCollector()
matlab_controller = MatlabController()
commands_sender = RobotCommandsSender()


async def run_all():
    tasks = []
    tasks.append(asyncio.create_task(vision_collector.run()))
    tasks.append(asyncio.create_task(referee_collector.run()))
    tasks.append(asyncio.create_task(matlab_controller.run()))
    tasks.append(asyncio.create_task(commands_sender.run()))
    await asyncio.gather(*tasks)


def loop_in_thread(loop):
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_all())


def do_run():
    loop = asyncio.get_event_loop()
    t = threading.Thread(target=loop_in_thread, args=(loop,))
    t.start()


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

