"""
Модуль описания структуры Field для хранения информации об объектах на поле (роботы и мяч)
"""
import bridge.processors.entity as entity
import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.robot as robot

class Goal:
    """
    Структура, описывающая ключевые точки ворот
    """
    def __init__(self, goal_dx, goal_dy) -> None:
        self.center = aux.Point(goal_dx, 0)
        self.up = aux.Point(goal_dx, goal_dy)
        self.down = aux.Point(goal_dx, -goal_dy)
        self.eye_forw = aux.Point(-aux.sign(goal_dx), 0)
        self.eye_up = aux.Point(0, aux.sign(goal_dy))

class Field:
    """
    Конструктор
    Инициализирует все нулями
    
    @todo Сделать инициализацию реальными параметрами для корректного
    определения скоростей и ускорений в первые секунды
    """
    def __init__(self, ally_color = 'b') -> None:
        self.ball = entity.Entity(const.GRAVEYARD_POS, 0, const.BALL_R)
        self.b_team = [ robot.Robot(const.GRAVEYARD_POS, 0, const.ROBOT_R, 'b', i) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]
        self.y_team = [ robot.Robot(const.GRAVEYARD_POS, 0, const.ROBOT_R, 'y', i) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]
        self.all_bots = [*self.b_team, *self.y_team]
        self.y_goal = Goal(const.GOAL_DX, const.GOAL_DY)
        self.b_goal = Goal(-const.GOAL_DX, const.GOAL_DY)

        if ally_color == 'b':
            self.allies = [*self.b_team]
            self.ally_goal = self.b_goal
            self.enemies = [*self.y_team]
            self.enemy_goal = self.y_goal
        elif ally_color == 'y':
            self.allies = [*self.y_team]
            self.ally_goal = self.y_goal
            self.enemies = [*self.b_team]
            self.enemy_goal = self.b_goal

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
    def updateBluRobot(self, idx, pos, angle, t):
        self.b_team[idx].update(pos, angle, t)

    """
    Обновить положение робота желтой команды
    !!! Вызывать один раз за итерацию с постоянной частотой !!!
    """
    def updateYelRobot(self, idx, pos, angle, t):
        self.y_team[idx].update(pos, angle, t)

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
