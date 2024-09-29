"""
Processor that creates the field
"""

import typing
from time import time

import attr
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.pb.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from strategy_bridge.processors import BaseProcessor

from bridge import const
from bridge.auxiliary import aux, fld

from scipy.optimize import minimize, dual_annealing
import numpy as np
from concurrent.futures import ThreadPoolExecutor
import bridge.strategy.accessories as acc


@attr.s(auto_attribs=True)
class FieldCreator(BaseProcessor):
    """class that creates the field"""

    processing_pause: typing.Optional[float] = 0.01
    reduce_pause_on_process_time: bool = False
    # commands_sink_reader: DataReader = attr.ib(init=False)
    # box_feedback_reader: DataReader = attr.ib(init=False)
    # field_writer: DataWriter = attr.ib(init=False)
    _ssl_converter: SSL_WrapperPacket = attr.ib(init=False)

    def __init__(self):
        self.points = None
        self.passes_writer = None
        self.field_reader = None
        self.field = None

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализация
        """
        super().initialize(data_bus)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.passes_writer = DataWriter(data_bus, const.PASSES_TOPIC, 20)
        self._ssl_converter = SSL_WrapperPacket()
        self.field = None

    def process_cell(self, cell):
        def wrp_fnc(x):
            point = aux.Point(x[0], x[1])
            return -acc.estimate_point(point, self.field.ball.get_pos(), self.field.enemies)

        tmp = aux.average_point(cell.peaks)
        res = minimize(
            wrp_fnc,
            np.array([tmp.x, tmp.y]),
            bounds=[(0, 4500), (-3000, 3000)],
            method="Nelder-Mead",
        )
        return res

    def process(self) -> None:
        """
        Метод обратного вызова процесса
        """

        self.field = self.field_reader.read_last()
        if self.field is None:
            return

        points = []

        cells = acc.get_cells(self.field.ball.get_pos(), self.field.enemies)

        with ThreadPoolExecutor(max_workers=1) as executor:
            futures = executor.map(self.process_cell, cells)

            for res in futures:
                if -res.get("fun") > _max:
                    _max = -res.get("fun")
                points.append(
                    (
                        aux.Point(res.get("x")[0], res.get("x")[1]),
                        aux.minmax(-res.get("fun"), -1, 1),
                    )
                )

        min_distance = 1000
        best = []

        for point in points:
            if all((point[0] - existing_point[0]).mag() >= min_distance for existing_point in best):
                best.append(point)

        best = sorted(best, key=lambda x: -x[1])

        self.passes_writer.write(best)
