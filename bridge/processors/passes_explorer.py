"""
Processor that creates the field
"""

import typing
from time import time

import attr
import numpy as np
from scipy.optimize import minimize
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.processors import BaseProcessor

import bridge.strategy.accessories as acc
from bridge import const, drawing
from bridge.auxiliary import aux, fld

NUM_SENDING_POINTS = 3
NUM_SAVING_POINTS = 6
DISTANCE_BETWEEN_POINTS = 500


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
        self.passes_writer = DataWriter(data_bus, const.PASSES_TOPIC, 1)
        self.image_writer = DataWriter(data_bus, const.IMAGE_TOPIC, 20)

        self.field = fld.Field(self.ally_color)
        self.image = drawing.Image(drawing.ImageTopic.PASSES)
        self.best: list[tuple[aux.Point, float]] = []

    def process_cell(self, point: tuple[float, float]) -> tuple[aux.Point, float]:
        """surf to local minimum from point"""

        def wrp_fnc(x: tuple[float, float]) -> float:
            point = aux.Point(x[0], x[1])
            return -self.estimate(point)

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
        return (aux.Point(res.get("x")[0], res.get("x")[1]), -res.get("fun"))

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

        points: list[tuple[aux.Point, float]] = []

        # cells = get_cells(  # ALARM DONT WORK FUCK   (;-;)
        #    self.field.ball.get_pos(),                 /|\
        #    self.field,                                / \
        #    [e.get_pos() for e in self.field.enemies],
        # )
        # tmp_data = [aux.Point(2000, 0), aux.Point(4000, 200), aux.Point(4000, -200)]
        # sampler = qmc.Halton(d=2)
        # start_points = sampler.random(5)

        # start_points[:, 0] = start_points[:, 0] * (x_range[1] - x_range[0]) + x_range[0]
        # start_points[:, 1] = start_points[:, 1] * (y_range[1] - y_range[0]) + y_range[0]

        start_points = get_start_points()
        for point in start_points:
            point_est = self.process_cell(point)
            points.append(point_est)

        old_points = [point[0] for point in self.best]
        for point in old_points:
            point_est = self.process_cell((point.x, point.y))
            old_est = self.estimate(point)
            if point_est[1] > old_est:
                new_point = aux.lerp(point, point_est[0], 0.5)
                points.append((new_point, self.estimate(new_point)))
            else:
                points.append((point, old_est))

        # with ThreadPoolExecutor(max_workers=1) as executor: #кажется эта штука работает слишком долго
        #     futures = executor.map(self.process_cell, start_points)

        #     for res in futures:
        #         points.append(
        #             (
        #                 aux.Point(res.get("x")[0], res.get("x")[1]),
        #                 aux.minmax(-res.get("fun"), -2, 1),
        #             )
        #         )

        points = sorted(points, key=lambda x: -x[1])
        best: list[tuple[aux.Point, float]] = []

        for point in points:
            if aux.dist(point[0], aux.closest_point_on_line(self.field.ball.get_pos(), self.field.enemy_goal.center, point[0])) < 2 * const.ROBOT_R:
                continue
            if all(aux.dist(point[0], existing_point[0]) >= DISTANCE_BETWEEN_POINTS for existing_point in best):
                best.append(point)

        best = best[:NUM_SAVING_POINTS]

        sended_best = best[:NUM_SENDING_POINTS]
        self.passes_writer.write(sended_best)

        self.best = best.copy()
        for p in sended_best:
            self.image.draw_dot(p[0], (255, 255, 255), 70)
        for p in best:
            est = aux.minmax(p[1], -2, 1)
            if est > 0.5:
                color = (int(255 * 2 * (1 - est)), 255, 0)
            elif est > 0:
                color = (255, int(255 * 2 * est), 0)
            else:
                color = (int(255 * (1 + est / 2)), 0, 0)
            self.image.draw_dot(p[0], color, 65)

        self.image_writer.write(self.image)
        self.image.clear()

        print("Passes long:", time() - t)

    def estimate(self, point: aux.Point) -> float:
        """estimate pass to point"""
        return acc.estimate_point(self.field, point, self.field.ball.get_pos())


def get_start_points() -> np.ndarray:
    """get random points to start search"""
    a, b = 2, 1
    x = np.random.beta(a, b, size=5)
    x_range = [-const.FIELD_DX, const.FIELD_DX]
    y_range = [-const.FIELD_DY, const.FIELD_DY]

    x = x * (x_range[1] - x_range[0]) + x_range[0]

    y = np.random.uniform(low=y_range[0], high=y_range[1], size=5)

    return np.vstack((x, y)).T
