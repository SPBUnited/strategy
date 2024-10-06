from typing import Any, Optional

from bridge import const
from bridge.auxiliary import aux, fld


class Cell:
    def __init__(self, peaks: list[aux.Point]) -> None:
        self.peaks: list[aux.Point] = peaks

    def __eq__(self, obj: Any) -> bool:
        try:
            return self.peaks == obj.peaks
        except TypeError:
            return False

    def print(self) -> None:
        """print all peaks of cell"""
        print("cell peaks:")
        for peak in self.peaks:
            print("\t", peak)

    def get_line_intersections(
        self, line_start: aux.Point, line_end: aux.Point, is_inf: str = "L"
    ) -> list[tuple[aux.Point, int]]:
        """
        returns a list of intersections and numbers of crossed edges
        line_start, line_end - points on line
        is_inf: "S" - segment, "R" - ray, "L" - line
        """
        intersections: list[tuple[aux.Point, int]] = []
        for index, _ in enumerate(self.peaks):
            inter = aux.get_line_intersection(
                self.peaks[index - 1],
                self.peaks[index],
                line_start,
                line_end,
                "S" + is_inf,
            )
            if inter is not None and (len(intersections) == 0 or inter != intersections[-1][0]):  # if intersection is peak
                intersections.append((inter, index))

        if intersections and intersections[0][0] == intersections[-1][0]:  # if intersection is peak
            intersections.pop()
        return intersections

    def intersect_cell(self, line_start: aux.Point, line_end: aux.Point, is_inf: str = "L") -> Optional["Cell"]:
        """
        if the line intersects a cell, changes it and returns a new one (only convex cells)
        is_inf: "S" - segment, "R" - ray, "L" - line
        """
        inter_idx = self.get_line_intersections(line_start, line_end, is_inf)
        if len(inter_idx) < 2:
            return None
        inter1, idx1 = inter_idx[0]
        inter2, idx2 = inter_idx[1]
        if inter1 == inter2:
            return None

        new_cell_peaks = self.peaks[idx1:idx2]

        if new_cell_peaks[-1] != inter2:
            new_cell_peaks += [inter2]
        if new_cell_peaks[0] != inter1:
            new_cell_peaks += [inter1]

        new_cell = Cell(new_cell_peaks)

        updated_peaks = self.peaks[idx2:] + self.peaks[0:idx1]  # + [inter1, inter2]

        if updated_peaks[-1] != inter1:
            updated_peaks += [inter1]
        if updated_peaks[0] != inter2:
            updated_peaks += [inter2]

        self.peaks = updated_peaks

        return new_cell

    def crop_cell(
        self,
        line_start: aux.Point,
        line_end: aux.Point,
        side_to_delete: int,
        is_inf: str = "L",
    ) -> bool:
        """
        crop the cell with line
        side_to_delete: -1 - delete left part, 1 - delete right part (to look relative to the direction of the vector)
        return True if cell been cropped, False if can't crop cell (line don't intersect cell)
        is_inf: "S" - segment, "R" - ray, "L" - line
        """
        cropped_part = self.intersect_cell(line_start, line_end, is_inf)
        if cropped_part:
            center = aux.average_point(self.peaks)
            sign = aux.sign(aux.vec_mult((center - line_start), (line_end - line_start)))
            if sign != side_to_delete:
                self.peaks = cropped_part.peaks
            return True
        else:
            return False


def get_cells(kick_point: aux.Point, field: fld.Field, enemies: list[aux.Point] = []) -> list[Cell]:
    left_top_cell = aux.Point(0, const.FIELD_DY)
    right_top_cell = aux.Point(const.FIELD_DX, const.FIELD_DY)
    right_down_cell = aux.Point(const.FIELD_DX, -const.FIELD_DY)
    left_down_cell = aux.Point(0, -const.FIELD_DY)

    cells = [Cell([left_top_cell, right_top_cell, right_down_cell, left_down_cell])]

    for enemy in enemies:
        new_cells = []
        for cell in cells:
            new_cell = cell.intersect_cell(enemy, field.enemy_goal.center)
            if new_cell:
                new_cells.append(new_cell)
        cells += new_cells

        new_cells = []
        cells_to_delete: list[int] = []

        for idx, cell in enumerate(cells):
            vec = (enemy - kick_point).unity()

            tangents = aux.get_tangent_points(enemy, kick_point, const.ROBOT_R)
            if len(tangents) < 2:
                continue

            side = aux.sign(aux.vec_mult((tangents[0] - kick_point), (enemy - kick_point)))

            new_cell = cell.intersect_cell(enemy - vec, enemy, "R")
            if new_cell:
                is_cropped = cell.crop_cell(kick_point, tangents[0], side, "R")
                if not is_cropped:
                    is_cropped = cell.crop_cell(kick_point, tangents[1], -side, "R")
                    if not is_cropped:
                        cells_to_delete = [idx] + cells_to_delete
                        # порядок важен, т к при удалении меняются индексы

                is_cropped = new_cell.crop_cell(kick_point, tangents[0], side, "R")
                if not is_cropped:
                    is_cropped = new_cell.crop_cell(kick_point, tangents[1], -side, "R")

                if is_cropped:
                    new_cells.append(new_cell)
            else:
                vec0 = tangents[0] - kick_point
                cell.crop_cell(tangents[0] - vec0, tangents[0], side, "R")
                vec1 = tangents[1] - kick_point
                cell.crop_cell(tangents[1] - vec1, tangents[1], -side, "R")

        for idx in cells_to_delete:  # NOTE
            cells.pop(idx)

        cells += new_cells

    return cells
