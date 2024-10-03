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
from scipy.stats import qmc
import numpy as np
from concurrent.futures import ThreadPoolExecutor
import bridge.strategy.accessories as acc


@attr.s(auto_attribs=True)
class ExplorePasses(BaseProcessor):
    """class that creates the field"""

    processing_pause: typing.Optional[float] = 0.2
    reduce_pause_on_process_time: bool = True
    # commands_sink_reader: DataReader = attr.ib(init=False)
    # box_feedback_reader: DataReader = attr.ib(init=False)
    # field_writer: DataWriter = attr.ib(init=False)
    _ssl_converter: SSL_WrapperPacket = attr.ib(init=False)

    ally_color: const.Color = const.Color.YELLOW

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализация
        """
        super().initialize(data_bus)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.passes_writer = DataWriter(data_bus, const.PASSES_TOPIC, 20)
        self._ssl_converter = SSL_WrapperPacket()
        self.field = fld.Field(self.ally_color)
        self.best: tuple[aux.Point, float] = []

    def process_cell(self, point) -> Any:
        def wrp_fnc(x) -> float:
            point = aux.Point(x[0], x[1])
            return -acc.estimate_point(
                point,
                self.field.ball.get_pos(),
                self.field,
                [e.get_pos() for e in self.field.active_enemies()],
            )

        # tmp = aux.average_point(cell.peaks)
        # tmp = aux.Point(2250 * random(), 3000 * random() - 1500)
        res = minimize(
            wrp_fnc,
            point,
            bounds=[(-const.FIELD_WIDTH / 2, const.FIELD_WIDTH / 2), (-const.FIELD_HEIGH / 2, const.FIELD_HEIGH / 2)],
            method="Nelder-Mead",
        )
        return res

    def process(self) -> None:
        """
        Метод обратного вызова процесса
        """
        t = time()
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
        # tmp_data = [aux.Point(2000, 0), aux.Point(4000, 200), aux.Point(4000, -200)]
        # sampler = qmc.Halton(d=2)
        # start_points = sampler.random(5)
        #
        x_range = [-const.FIELD_WIDTH / 2, const.FIELD_WIDTH / 2]
        y_range = [-const.FIELD_HEIGH / 2, const.FIELD_HEIGH / 2]
        #
        # start_points[:, 0] = start_points[:, 0] * (x_range[1] - x_range[0]) + x_range[0]
        # start_points[:, 1] = start_points[:, 1] * (y_range[1] - y_range[0]) + y_range[0]

        a, b = 2, 1
        x = np.random.beta(a, b, size=5)

        # Масштабирование координат по оси x в заданный диапазон
        x = x * (x_range[1] - x_range[0]) + x_range[0]

        # Генерация координат по оси y с равномерным распределением
        y = np.random.uniform(low=y_range[0], high=y_range[1], size=5)

        # Объединяем координаты в массив точек
        start_points = np.vstack((x, y)).T

        additional_points = np.array([[point[0].x, point[0].y] for point in self.best])

        if len(additional_points) > 0:
            start_points = np.concatenate((start_points, additional_points), axis=0)

        with ThreadPoolExecutor(max_workers=1) as executor:
            futures = executor.map(self.process_cell, start_points)

            for res in futures:
                if -res.get("fun") > _max:
                    _max = -res.get("fun")
                points.append(
                    (
                        aux.Point(res.get("x")[0], res.get("x")[1]),
                        aux.minmax(-res.get("fun"), -1, 1),
                    )
                )

        min_distance = 500
        points = sorted(points, key=lambda x: -x[1])
        best = []

        for point in points:
            if all(
                    (point[0] - existing_point[0]).mag() >= min_distance
                    for existing_point in best
            ):
                best.append(point)

        # tmp = []
        # for p in start_points:
        #     tmp.append((aux.Point(p[0], p[1]), 1))

        self.passes_writer.write(best)

        self.best = best.copy()

        # print(time() - t)
