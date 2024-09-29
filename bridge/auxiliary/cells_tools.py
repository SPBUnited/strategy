import math
from typing import Optional, Any

import bridge.auxiliary as aux

class Peak(aux.Point):
    def __init__(self, pos: aux.Point) -> None:
        super().__init__(pos.x, pos.y)
        self.neighbors: list[Cell] = []

    def __eq__(self, p: Any) -> bool:
        try:
            return abs(self.x - p.x) < 0.1 and abs(self.y - p.y) < 0.1
        except TypeError:
            return False

    def __str__(self) -> str:
        return f"x = {self.x:.2f},\ty = {self.y:.2f}"

    def add_neighbor_(self, new_neighbor: "Cell") -> None:
        """add new neighbor to peak"""
        if new_neighbor not in self.neighbors:
            self.neighbors.append(new_neighbor)

    def remove_neighbor_(self, neighbor_to_remove: "Cell") -> None:
        """remove one neighbor for peak"""
        if neighbor_to_remove in self.neighbors:
            self.neighbors.remove(neighbor_to_remove)


class Cell:
    def __init__(self, peaks: list[Peak]) -> None:
        self.peaks: list[Peak] = peaks

        for peak in self.peaks:
            peak.add_neighbor_(self)

    def __eq__(self, obj: Any) -> bool:
        try:
            return self.peaks == obj.peaks
        except TypeError:
            return False

    def add_peak(self, peak: Peak) -> None:
        """add new peak and update peak's neighbors"""
        self.peaks.append(peak)
        peak.add_neighbor_(self)

        for neighbor in self.get_all_neighbors():  # maybe side_neighbors is enough
            neighbors_peaks = neighbor.peaks
            for i, _ in enumerate(neighbors_peaks):
                if (
                    aux.is_point_on_line(
                        peak, neighbors_peaks[i - 1], neighbors_peaks[i], "S"
                    )
                    and peak not in neighbor.peaks
                ):
                    peak.add_neighbor_(neighbor)
                    neighbor.paste_new_peak(peak)
                    break

    def remove_peak(self, peak: Peak) -> None:
        """remove peak from cell"""
        self.peaks.remove(peak)
        peak.remove_neighbor_(self)

    def paste_new_peak(self, new_peak: Peak) -> bool:
        """paste new peak in right position"""
        for i, _ in enumerate(self.peaks):
            if aux.is_point_on_line(new_peak, self.peaks[i - 1], self.peaks[i], "S"):
                self.peaks = self.peaks[i:] + self.peaks[:i]
                self.add_peak(new_peak)
                return True
        return False

    def get_all_neighbors(self) -> list["Cell"]:
        """return list of neighbors of cell"""
        neighbors: list[Cell] = []
        for peak in self.peaks:
            for neighbor in peak.neighbors:
                if neighbor not in neighbors:
                    neighbors += [neighbor]

        return neighbors

    def get_peak_neighbors(self) -> list["Cell"]:
        """return list of neighbors (by only one peak) of cell"""
        neighbors_by_peak: list["Cell"] = []
        neighbors_by_side: list["Cell"] = []
        for peak in self.peaks:
            for neighbor in peak.neighbors:
                if neighbor not in (neighbors_by_peak + neighbors_by_side):
                    neighbors_by_peak.append(neighbor)
                elif neighbor in neighbors_by_peak:
                    neighbors_by_peak.remove(neighbor)
                    neighbors_by_side.append(neighbor)

        return neighbors_by_peak

    def get_side_neighbors(self) -> list["Cell"]:
        """return list of neighbors (by side) of cell"""
        neighbors_by_peak: list["Cell"] = []
        neighbors_by_side: list["Cell"] = []
        for peak in self.peaks:
            for neighbor in peak.neighbors:
                if neighbor != self:
                    if neighbor not in (neighbors_by_peak + neighbors_by_side):
                        neighbors_by_peak.append(neighbor)
                    elif neighbor in neighbors_by_peak:
                        neighbors_by_peak.remove(neighbor)
                        neighbors_by_side.append(neighbor)

        return neighbors_by_side

    def print(self) -> None:
        """print all peaks of cell"""
        print("cell peaks:")
        for peak in self.peaks:
            print("\t", peak, "\t", len(peak.neighbors))
        print(
            f"cell neighbors : peak {len(self.get_peak_neighbors())}, side {len(self.get_side_neighbors())}"
        )

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
            if inter is not None and (
                len(intersections) == 0
                or inter != intersections[-1][0]  # if intersection is peak
            ):
                intersections.append((inter, index))

        if (
            intersections and intersections[0][0] == intersections[-1][0]
        ):  # if intersection is peak
            intersections.pop()
        return intersections

    def intersect_cell(
        self, line_start: aux.Point, line_end: aux.Point, is_inf: str = "L"
    ) -> Optional["Cell"]:
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

        new_cell = Cell(self.peaks[idx2:] + self.peaks[:idx1])

        for peak in self.peaks[:idx1] + self.peaks[idx2:]:
            self.remove_peak(peak)

        if new_cell.peaks[-1] == inter1:
            new_peak1 = new_cell.peaks[-1]
        elif self.peaks[0] == inter1:
            new_peak1 = self.peaks[0]
        else:
            new_peak1 = Peak(inter1)

        if self.peaks[-1] == inter2:
            new_peak2 = self.peaks[-1]
        elif new_cell.peaks[0] == inter2:
            new_peak2 = new_cell.peaks[0]
        else:
            new_peak2 = Peak(inter2)

        if new_cell.peaks[-1] != new_peak1:
            new_cell.add_peak(new_peak1)
        if new_cell.peaks[0] != new_peak2:
            new_cell.add_peak(new_peak2)

        if self.peaks[-1] != new_peak2:
            self.add_peak(new_peak2)
        if self.peaks[0] != new_peak1:
            self.add_peak(new_peak1)

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
            sign = aux.sign(
                aux.vec_mult((center - line_start), (line_end - line_start))
            )
            if sign != side_to_delete:
                for peak in self.peaks.copy():
                    self.remove_peak(peak)
                self.peaks = cropped_part.peaks
            else:
                for peak in cropped_part.peaks.copy():
                    cropped_part.remove_peak(peak)
            return True
        else:
            return False