"""
Модуль описания структуры Field для хранения информации об объектах на поле (роботы и мяч)
"""
import bridge.processors.entity as entity
import bridge.processors.auxiliary as aux
import bridge.processors.const as const

class Field:
    """
    Конструктор
    Инициализирует все нулями
    
    @todo Сделать инициализацию реальными параметрами для корректного
    определения скоростей и ускорений в первые секунды
    """
    def __init__(self) -> None:
        self.ball = entity.Entity(aux.Point(0,0), 0, const.BALL_R)
        self.b_team = [ entity.Entity(aux.Point(0,0), 0, const.ROBOT_R) for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        self.y_team = [ entity.Entity(aux.Point(0,0), 0, const.ROBOT_R) for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]

    """
    Обновить положение мяча
    !!! Вызывать один раз за итерацию с постоянной частотой !!!
    """
    def updateBall(self, pos):
        self.ball.update(pos, 0)

    """
    Обновить положение робота синей команды
    !!! Вызывать один раз за итерацию с постоянной частотой !!!
    """
    def updateBluRobot(self, idx, pos, angle):
        self.b_team[idx].update(pos, angle)

    """
    Обновить положение робота желтой команды
    !!! Вызывать один раз за итерацию с постоянной частотой !!!
    """
    def updateYelRobot(self, idx, pos, angle):
        self.y_team[idx].update(pos, angle)

    """
    Получить объект мяча

    @return Объект entity.Entity
    """
    def getBall(self):
        return self.ball

    """
    Получить массив роботов синей команды

    @return Массив entity.Entity[]
    """
    def getBluTeam(self):
        return self.b_team

    """
    Получить массив роботов желтой команды

    @return Массив entity.Entity[]
    """
    def getYelTeam(self):
        return self.y_team
