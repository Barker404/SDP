from models import *
from collisions import *
from strategies import *
from utilities import *


class Planner:
    
    # The planner assigns the robots a state and strategy
    #
    # Each robot (attacker, defender) has a state:
    # attacker: defence, grab, score, catch
    # defender: defence, grab, pass
    #
    # Based on these they choose one of a number of strategies from strategies.py
    # Decided in the (large) plan() method
    #
    # NOTE: Strategies have their own (sub-)state
    # This is how far through the strategy they are
    # e.g. when grabing ball, we must -prepare-, -go to the ball-, -grab- it, and be -finished-

    def __init__(self, our_side, pitch_num, isPenalty=False):
        self._world = World(our_side, pitch_num)

        self._defender_strategies = { 'pass'   : [SimplePass],
                                      'defend' : [SimpleBlock],
                                      'penalty' : [DefenderPenalty]
                                      }
        
        if isPenalty:
            self._defender_state = 'penalty'
            print "penalty"
        else:
            self._defender_state = 'pass'
            print "pass"


        self._defender_current_strategy = self.choose_defender_strategy(self._world)

    # Choose the first strategy in the applicable list.
    def choose_defender_strategy(self, world):
        next_strategy = self._defender_strategies[self._defender_state][0]
        return next_strategy(world)

    @property
    def defender_strat_state(self):
        return self._defender_current_strategy.current_state

    @property
    def defender_state(self):
        return self._defender_state

    @defender_state.setter
    def defender_state(self, new_state):
        assert new_state in ['defend', 'pass']
        self._defender_state = new_state

    def update_world(self, position_dictionary):
        self._world.update_positions(position_dictionary)

    def plan(self, robot='attacker'):
        assert robot == 'defender'

        if self._defender_state == 'penalty':
            return self._defender_current_strategy.generate()
            
        # Check strategy changes
        if self._world._our_side == 'right':
            zone = self._world.pitch.zones[3]
        else:
            zone = self._world.pitch.zones[0]
        box = zone.boundingBox()

        ball_in_zone = (box[0] <= self._world.ball.x <= box[1] and 
            box[2] <= self._world.ball.y <= box[3])

        if (((self._world.ball.angle > (5.0/4.0)*pi or 
                self._world.ball.angle < (3.0/4.0)*pi or 
                self._world.ball.velocity < BALL_VELOCITY) and 
                ball_in_zone) or
                self._world.our_defender.has_ball(self._world.ball) or
                self._world.our_defender.can_catch_ball(self._world.ball)):
            if self._defender_state != 'pass':
                self._defender_state = 'pass'
                self._defender_current_strategy = self.choose_defender_strategy(self._world)
        else:
            if self._defender_state != 'defend':
                self._defender_state = 'defend'
                self._defender_current_strategy = self.choose_defender_strategy(self._world)

        return self._defender_current_strategy.generate()
