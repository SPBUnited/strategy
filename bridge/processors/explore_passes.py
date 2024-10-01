"""
Processor that creates the field
"""

import typing
from typing import Any
from time import time

import attr
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.pb.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from strategy_bridge.processors import BaseProcessor

from bridge import const
from bridge.auxiliary import aux, fld
from bridge.auxiliary.cells_tools import Cell, get_cells

from random import random


from scipy.optimize import minimize, dual_annealing
import numpy as np
from concurrent.futures import ThreadPoolExecutor
import bridge.strategy.accessories as acc


@attr.s(auto_attribs=True)
class ExplorePasses(BaseProcessor):
    """class that creates the field"""

    processing_pause: typing.Optional[float] = 0.02
    reduce_pause_on_process_time: bool = False
    # commands_sink_reader: DataReader = attr.ib(init=False)
    # box_feedback_reader: DataReader = attr.ib(init=False)
    # field_writer: DataWriter = attr.ib(init=False)
    _ssl_converter: SSL_WrapperPacket = attr.ib(init=False)

    ally_color: const.Color = const.Color.BLUE

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализация
        """
        super().initialize(data_bus)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.passes_writer = DataWriter(data_bus, const.PASSES_TOPIC, 20)
        self._ssl_converter = SSL_WrapperPacket()
        self.field = fld.Field(self.ally_color)

    def process_cell(self, cell: Cell) -> Any:
        def wrp_fnc(x) -> float:
            point = aux.Point(x[0], x[1])
            return -acc.estimate_point(
                point,
                self.field.ball.get_pos(),
                self.field,
                [e.get_pos() for e in self.field.enemies],
            )

        # tmp = aux.average_point(cell.peaks)
        tmp = aux.Point(2250 * random(), 3000 * random() - 1500)
        res = minimize(
            wrp_fnc,
            np.array([tmp.x, tmp.y]),
            bounds=[(0, 2250), (-1500, 1500)],
            method="Nelder-Mead",
        )
        return res

    def process(self) -> None:
        """
        Метод обратного вызова процесса
        """

        new_field = self.field_reader.read_last()
        if new_field is not None:
            updated_field = new_field.content
            self.field.update_field(updated_field)
        else:
            return

        points = []

        _max = -100

        # cells = get_cells(  # ALARM DONT WORK FUCK
        #    self.field.ball.get_pos(),
        #    self.field,
        #    [e.get_pos() for e in self.field.enemies],
        # )

        with ThreadPoolExecutor(max_workers=1) as executor:
            futures = executor.map(self.process_cell, range(10))

            for res in futures:
                if -res.get("fun") > _max:
                    _max = -res.get("fun")
                points.append(
                    (
                        aux.Point(res.get("x")[0], res.get("x")[1]),
                        aux.minmax(-res.get("fun"), -1, 1),
                    )
                )

        min_distance = 30
        points = sorted(points, key=lambda x: -x[1])
        best = []

        for point in points:
            if all(
                (point[0] - existing_point[0]).mag() >= min_distance
                for existing_point in best
            ):
                best.append(point)

        best = sorted(best, key=lambda x: -x[1])

        self.passes_writer.write(best)
