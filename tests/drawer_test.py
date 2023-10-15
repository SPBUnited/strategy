# import time
# import math

# from context import field, router, strategy, aux, drw



# if __name__ == '__main__':
    
#     field = field.Field()
#     router = router.Router()
#     strategy = strategy.Strategy()

#     while True:

#         field.updateYelRobot(0, field.y_goal - aux.Point(500, 200), 0, 0)        
#         field.updateBall(field.y_goal - aux.Point(1500, 1500))
#         field.ball._vel = aux.rotate(aux.Point(1200, 0), (time.time()/10)%(math.pi/2))
#         field.draw()

#         waypoints = strategy.process(field)
#         drw.Drawer().drawAbsVec(field.y_team[0].getPos(), waypoints[0].pos)

#         drw.Drawer().flip()

#         pass