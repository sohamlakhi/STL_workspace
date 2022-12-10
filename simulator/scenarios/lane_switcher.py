

class LaneSwitcher:
    def __init__(self, frenet):
        self.frenet = frenet
        self.state = 0
        self.transition = False

    def plan(self, gposition1, gposition2, planner1, planner2):

            gap = self.frenet.gap(gposition1, gposition2, unwrap = True)
        
            if self.state == 0:
                if gap > -2.0:
                    #switch waypoints using planner object
                    self.state = 1

            elif self.state == 1:
                if gap > 30.0:
                    #switch lanes again
                    self.state == 0        
            
"""
debug new gap operator
debug adding new tracks

read your specs
get setup with breach/staliro
write signal baggers
"""