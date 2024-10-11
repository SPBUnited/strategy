"""
Processor that creates the field
"""

import typing
from concurrent.futures import ThreadPoolExecutor
from typing import Any

import attr
import numpy as np
from scipy.optimize import minimize
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.processors import BaseProcessor

import bridge.strategy.accessories as acc
from bridge import const, drawing
from bridge.auxiliary import aux, fld


@attr.s(auto_attribs=True)
class ExplorePasses(BaseProcessor):
    """class that creates the field"""

    processing_pause: typing.Optional[float] = 0.1
    reduce_pause_on_process_time: bool = True

    ally_color: const.Color = const.Color.BLUE

    def initialize(self, data_bus: DataBus) -> None:
        """Инициализация"""
        super().initialize(data_bus)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.passes_writer = DataWriter(data_bus, const.PASSES_TOPIC, 20)
        self.image_writer = DataWriter(data_bus, const.IMAGE_TOPIC, 20)

        self.field = fld.Field(self.ally_color)
        self.image = drawing.Image(drawing.ImageTopic.PASSES)
        self.best: list[tuple[aux.Point, float]] = []

        a, b = 2, 1
        x = np.random.beta(a, b, size=5)
        x_range = [-const.FIELD_DX, const.FIELD_DX]
        y_range = [-const.FIELD_DY, const.FIELD_DY]

        x = x * (x_range[1] - x_range[0]) + x_range[0]

        y = np.random.uniform(low=y_range[0], high=y_range[1], size=5)

        self.start_points = np.vstack((x, y)).T

    def process_cell(self, point: tuple[float, float]) -> Any:
        """surf to local minimum from point"""

        def wrp_fnc(x: tuple[float, float]) -> float:
            point = aux.Point(x[0], x[1])
            return -acc.estimate_point(
                self.field,
                point,
                self.field.ball.get_pos(),
            )

        # tmp = aux.average_point(cell.peaks)
        # tmp = aux.Point(2250 * random(), 3000 * random() - 1500)
        res = minimize(
            wrp_fnc,
            point,
            bounds=[
                (-const.FIELD_DX, const.FIELD_DX),
                (-const.FIELD_DY, const.FIELD_DY),
            ],
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

        points: list[tuple[aux.Point, float]] = []

        _max = -100

        # cells = get_cells(  # ALARM DONT WORK FUCK   (;-;)
        #    self.field.ball.get_pos(),                 /|\
        #    self.field,                                / \
        #    [e.get_pos() for e in self.field.enemies],
        # )
        # tmp_data = [aux.Point(2000, 0), aux.Point(4000, 200), aux.Point(4000, -200)]
        # sampler = qmc.Halton(d=2)
        # start_points = sampler.random(5)
        #

        #
        # start_points[:, 0] = start_points[:, 0] * (x_range[1] - x_range[0]) + x_range[0]
        # start_points[:, 1] = start_points[:, 1] * (y_range[1] - y_range[0]) + y_range[0]

        start_points = self.start_points

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
                        aux.minmax(-res.get("fun"), -2, 1),
                    )
                )

        min_distance = 500
        points = sorted(points, key=lambda x: -x[1])
        best: list[tuple[aux.Point, float]] = []

        for point in points:
            if all((point[0] - existing_point[0]).mag() >= min_distance for existing_point in best):
                best.append(point)

        # tmp = []
        # for p in start_points:
        #     tmp.append((aux.Point(p[0], p[1]), 1))

        self.passes_writer.write(best)

        self.best = best.copy()

        for p in best:
            if p[1] > 0.5:
                color = (int(255 * 2 * (1 - p[1])), 255, 0)
            elif p[1] > 0:
                color = (255, int(255 * 2 * p[1]), 0)
            else:
                color = (int(255 * (1 + p[1] / 2)), 0, 0)
            self.image.draw_dot(p[0], color, 65)

        self.image_writer.write(self.image)
        self.image.clear()

        # print(time() - t)
