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
        # LB: Magic numbers!
        # These should surely be constants in models.py?
        # Also need to make sure grabber area is consistent with our robot
        self._world.our_defender.catcher_area = {'width' : 25, 'height' : 18, 'front_offset' : 12} #10
        self._world.our_attacker.catcher_area = {'width' : 25, 'height' : 18, 'front_offset' : 14}

        self._attacker_strategies = { 'pass'   : [SimplePass],
                                      'defend' : [SimpleBlock]}
        self._defender_strategies = { 'pass'   : [SimplePass],
                                      'defend' : [SimpleBlock],
                                      'penalty' : [DefenderPenalty]
                                      }

        # self._attacker_strategies = {'defence' : [AttackerDefend],
        #                              'grab' : [AttackerGrab, AttackerGrabCareful],
        #                              'score' : [AttackerDriveByTurn, AttackerDriveBy, AttackerTurnScore, AttackerScoreDynamic],
        #                              'catch' : [AttackerPositionCatch, AttackerCatch]}

        # self._defender_strategies = {'defence' : [DefenderDefence, DefenderPenalty],
        #                              'grab' : [DefenderGrab],
        #                              'pass' : [DefenderBouncePass]}

        self._attacker_state = 'defend'
        
        if isPenalty:
            self._defender_state = 'penalty'
            print "penalty"
        else:
            self._defender_state = 'pass'
            print "pass"


        self._defender_current_strategy = self.choose_defender_strategy(self._world)
        self._attacker_current_strategy = self.choose_attacker_strategy(self._world)

    # LB: Only chooses the first possible strategy? Is this correct?
    # Provisional. Choose the first strategy in the applicable list.
    def choose_attacker_strategy(self, world):
        next_strategy = self._attacker_strategies[self._attacker_state][0]
        return next_strategy(world)

    # Provisional. Choose the first strategy in the applicable list.
    def choose_defender_strategy(self, world):
        next_strategy = self._defender_strategies[self._defender_state][0]
        return next_strategy(world)

    @property
    def attacker_strat_state(self):
        return self._attacker_current_strategy.current_state

    @property
    def defender_strat_state(self):
        return self._defender_current_strategy.current_state

    @property
    def attacker_state(self):
        return self._attacker_state

    @attacker_state.setter
    def attacker_state(self, new_state):
        assert new_state in ['defend', 'pass', 'penalty']
        self._attacker_state = new_state

    @property
    def defender_state(self):
        return self._defender_state

    @defender_state.setter
    def defender_state(self, new_state):
        assert new_state in ['defend', 'pass']
        self._defender_state = new_state

    def update_world(self, position_dictionary):
        self._world.update_positions(position_dictionary)

    # LB: We could split up this big method
    # I also don't think robot should have a default - to avoid forgetting the parameter
    # But we need to check if it is used anywhere without the parameter
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


        # Old Stuff
        our_defender = self._world.our_defender
        our_attacker = self._world.our_attacker
        their_defender = self._world.their_defender
        their_attacker = self._world.their_attacker
        ball = self._world.ball
        if robot == 'defender':
            # If the ball is in their attacker zone:
            if self._world.pitch.zones[their_attacker.zone].isInside(ball.x, ball.y):
                # If the bal is not in the defender's zone, the state should always be 'defend'.
                if not self._defender_state == 'defence':
                    self._defender_state = 'defence'
                    self._defender_current_strategy = self.choose_defender_strategy(self._world)
                return self._defender_current_strategy.generate()

            # We have the ball in our zone, so we grab and pass:
            elif self._world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y):
                # Check if we should switch from a grabbing to a scoring strategy.
                if  self._defender_state == 'grab' and self._defender_current_strategy.current_state == 'GRABBED':
                    self._defender_state = 'pass'
                    self._defender_current_strategy = self.choose_defender_strategy(self._world)

                # Check if we should switch from a defence to a grabbing strategy.
                elif self._defender_state == 'defence':
                    self._defender_state = 'grab'
                    self._defender_current_strategy = self.choose_defender_strategy(self._world)

                elif self._defender_state == 'pass' and self._defender_current_strategy.current_state == 'FINISHED':
                    self._defender_state = 'grab'
                    self._defender_current_strategy = self.choose_defender_strategy(self._world)

                return self._defender_current_strategy.generate()
            # Otherwise, chillax:
            else:

                return do_nothing()

        else:
            # If the ball is in their defender zone we defend:
            if self._world.pitch.zones[their_defender.zone].isInside(ball.x, ball.y):
                if not self._attacker_state == 'defence':
                    self._attacker_state = 'defence'
                    self._attacker_current_strategy = self.choose_attacker_strategy(self._world)
                return self._attacker_current_strategy.generate()

            # If ball is in our attacker zone, then grab the ball and score:
            elif self._world.pitch.zones[our_attacker.zone].isInside(ball.x, ball.y):

                # Check if we should switch from a grabbing to a scoring strategy.
                if self._attacker_state == 'grab' and self._attacker_current_strategy.current_state == 'GRABBED':
                    self._attacker_state = 'score'
                    self._attacker_current_strategy = self.choose_attacker_strategy(self._world)

                elif self._attacker_state == 'grab':
                    # Switch to careful mode if the ball is too close to the wall.
                    if abs(self._world.ball.y - self._world.pitch.height) < 0 or abs(self._world.ball.y) < 0:
                        if isinstance(self._attacker_current_strategy, Milestone2Attacker):
                            self._attacker_current_strategy = Milestone2AttackerCareful(self._world)
                    else:
                        if isinstance(self._attacker_current_strategy, Milestone2AttackerCareful):
                            self._attacker_current_strategy = Milestone2Attacker(self._world)

                # Check if we should switch from a defence to a grabbing strategy.
                elif self._attacker_state in ['defence', 'catch'] :
                    self._attacker_state = 'grab'
                    self._attacker_current_strategy = self.choose_attacker_strategy(self._world)

                elif self._attacker_state == 'score' and self._attacker_current_strategy.current_state == 'FINISHED':
                    self._attacker_state = 'grab'
                    self._attacker_current_strategy = self.choose_attacker_strategy(self._world)

                return self._attacker_current_strategy.generate()
            # If the ball is in our defender zone, prepare to catch the passed ball:
            elif self._world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y) or \
                 self._attacker_state == 'catch':
                 # self._world.pitch.zones[their_attacker.zone].isInside(ball.x, ball.y):
                if not self._attacker_state == 'catch':
                    self._attacker_state = 'catch'
                    self._attacker_current_strategy = self.choose_attacker_strategy(self._world)
                return self._attacker_current_strategy.generate()
            else:
                return calculate_motor_speed(0, 0)
