import sys

sys.path.insert(0, '..')

import time
import math
import bridge.processors.const as const
import bridge.processors.robot as robot
import bridge.processors.auxiliary as aux
import bridge.processors.field as field
import bridge.processors.router as router
import bridge.processors.strategy as strategy
import bridge.processors.waypoint as wp
import bridge.processors.drawer as drw



if __name__ == '__main__':
    
    field = field.Field()
    router = router.Router()
    strategy = strategy.Strategy()

    while True:

        field.updateYelRobot(0, field.y_goal - aux.Point(500, 200), 0, 0)        
        field.updateBall(field.y_goal - aux.Point(1500, 1500))
        field.ball.vel = aux.rotate(aux.Point(1200, 0), (time.time()/10)%(math.pi/2))
        field.draw()

        waypoints = strategy.process(field)
        drw.Drawer().drawAbsVec(field.y_team[0].pos, waypoints[0].pos)

        drw.Drawer().flip()

        pass