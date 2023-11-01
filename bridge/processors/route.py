"""
Класс, описывающий текущий маршрут робота
"""


import bridge.processors.auxiliary as aux
import bridge.processors.robot as robot
import bridge.processors.waypoint as wp


class Route:
    """
    Класс описание произвольного маршрута
    """

    def __init__(self, rbt: robot.Robot):
        """
        Конструктор
        """
        self._robot = [wp.Waypoint(rbt.get_pos(), rbt._angle, wp.WType.T_ROBOT)]
        self._destination = [wp.Waypoint(aux.GRAVEYARD_POS, 0, wp.WType.T_GRAVEYARD)]
        self._routewp: list[wp.Waypoint] = []
        # self.__route = [*self.robot, *self.__routewp, *self.__destination]

    def update(self, rbt: robot.Robot) -> None:
        """
        Обновить маршрут

        Обновляет текущее положение робота в маршрутной карте
        """
        self._robot = [wp.Waypoint(rbt.get_pos(), rbt.get_angle(), wp.WType.T_ROBOT)]
        # if self.getNextVec().mag() < const.VANISH_DIST and \
        #     len(self._routewp) >= 1:
        #     del self._routewp[0]

    def clear(self) -> None:
        """
        Очистить промежуточные точки маршрута
        """
        self._routewp = []

    def __get_route(self) -> list[wp.Waypoint]:
        """
        Получить маршрут в виде списка путевых точек
        """
        return [*self._robot, *self._routewp, *self._destination]

    def set_dest_wp(self, dest: wp.Waypoint) -> None:
        """
        Задать конечную точку
        """
        self._destination = [dest]

    def get_dest_wp(self) -> wp.Waypoint:
        """
        Получить конечную точку
        """
        return self._destination[0]

    def get_next_wp(self) -> wp.Waypoint:
        """
        Получить следующую путевую точку
        """
        return self.__get_route()[1]

    def get_next_segment(self) -> list[wp.Waypoint]:
        """
        Получить следующий сегмент маршрута в виде списка двух точек
        """
        return self.__get_route()[0:1]

    def get_next_vec(self) -> aux.Point:
        """
        Получить следующий сегмент маршрута в виде вектора
        """
        return self.__get_route()[1].pos - self.__get_route()[0].pos

    def get_next_angle(self) -> float:
        """
        Получить угол следующей путевой точки
        """
        return self.__get_route()[1].angle

    def get_next_type(self) -> wp.WType:
        """
        Получить тип следующей путевой точки
        """
        return self.__get_route()[1].type

    def insert_wp(self, wpt: wp.Waypoint) -> None:
        """
        Вставить промежуточную путевую точку в начало маршрута
        """
        self._routewp.insert(0, wpt)

    def is_used(self) -> bool:
        """
        Определить, используется ли маршрут
        """
        return self._destination[0].type != wp.WType.T_GRAVEYARD

    def get_length(self) -> float:
        """
        Получить длину маршрута
        """
        dist = 0.0
        last_wp_pos = self._robot[0].pos
        for wpt in self.__get_route():
            if wpt.type == wp.WType.S_BALL_GO or wpt.type == wp.WType.S_BALL_KICK or wpt.type == wp.WType.S_BALL_GRAB:
                break
            dist += (wpt.pos - last_wp_pos).mag()
            last_wp_pos = wpt.pos
        return dist

    def __str__(self) -> str:
        strin = "ROUTE: "
        for wpt in self.__get_route():
            strin += " ->\n" + str(wpt)
        # for wp in [*self.robot, *self.__routewp, *self.__destination]:
        #     strin += " -> " + str(wp)
        return strin
