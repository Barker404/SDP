from utilities import *
import math
from random import randint
import time
from Polygon.cPolygon import Polygon

DEFAULT_KICK_POWER = 70

class Strategy(object):

    PRECISE_BALL_ANGLE_THRESHOLD = math.pi / 15.0
    UP, DOWN = 'UP', 'DOWN'

    def __init__(self, world, states):
        self.world = world
        self.states = states
        self._current_state = states[0]

    @property
    def current_state(self):
        return self._current_state

    @current_state.setter
    def current_state(self, new_state):
        assert new_state in self.states
        self._current_state = new_state

    def reset_current_state(self):
        self.current_state = self.states[0]

    def is_last_state(self):
        return self._current_state == self.states[-1]

    def generate(self):
        return self.NEXT_ACTION_MAP[self.current_state]()


class SimplePass(Strategy):
    # For controlling _defender_

    (PREPARE, GET_BALL, AVOID, ALIGN_HORIZ, ALIGN_MID_FAR, 
        ALIGN_STRAIGHT, STOP, SHOOT, WAIT) = \
        ('PREPARE', 'GET_BALL', 'AVOID', 'ALIGN_HORIZ', 'ALIGN_MID_FAR', 
            'ALIGN_STRAIGHT', 'SHOOT', 'STOP', 'WAIT')
    STATES = [PREPARE, GET_BALL, AVOID, ALIGN_HORIZ, ALIGN_MID_FAR, 
        ALIGN_STRAIGHT, SHOOT, STOP, WAIT]

    def __init__(self, world):
        super(SimplePass, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.PREPARE: self.prepare,
            self.GET_BALL: self.get_ball,
            self.AVOID: self.avoid,
            self.ALIGN_HORIZ: self.align_horiz,
            self.ALIGN_MID_FAR: self.align_mid_far,
            self.ALIGN_STRAIGHT: self.align_straight,
            self.STOP: self.stop,
            self.SHOOT: self.shoot,
            self.WAIT: self.wait
        }

        self.TIME_LIMIT = 8
        self.SPACE_THRESHOLD = 60

        self.our_attacker = self.world.our_attacker
        self.our_defender = self.world.our_defender
        self.their_attacker = self.world.their_attacker
        self.ball = self.world.ball

        self.catchTime = -1
        self.clockwise = True

    def prepare(self):
        self.current_state = self.GET_BALL
        if self.our_defender.catcher == 'closed':
            self.our_defender.catcher = 'open'
            return open_catcher()
        else:
            return do_nothing()

    def get_ball(self):

        # path = self.world.our_defender.get_pass_path(self.world.our_attacker)
        # BLOCKED = path.overlaps(Polygon(self.world.their_attacker.get_polygon()))
        # print BLOCKED;

        displacement, angle = self.our_defender.get_direction_to_point(self.ball.x, self.ball.y)
        if self.our_defender.can_catch_ball(self.ball):
            self.current_state = self.AVOID
            self.catchTime = time.clock()
            self.our_defender.catcher = 'closed'
            return grab_ball()
        else:
            if (displacement > 40):
                return calculate_motor_speed(displacement, angle, careful=True)
            else:
                return calculate_motor_speed_for_catch(displacement, angle, careful=True)

    def avoid(self):

        # Check we aren't out of time
        if time.clock() - self.catchTime > self.TIME_LIMIT:
            self.current_state = self.ALIGN_MID_FAR
            return do_nothing()

        bottom_split = self.world.pitch.height/3
        top_split = bottom_split*2
        midpont = self.world.pitch.height/2


        if abs(self.their_attacker.y - self.our_defender.y) > self.SPACE_THRESHOLD:
            
            path = self.world.our_defender.get_pass_path(self.world.our_attacker)
            BLOCKED = path.overlaps(Polygon(self.world.their_attacker.get_polygon()))

            print BLOCKED;

            if(BLOCKED):
                self.current_state = self.ALIGN_STRAIGHT
            else:
                self.current_state = self.ALIGN_HORIZ
            return do_nothing()
        else:
            if self.their_attacker.y < bottom_split:
                # Bottom blocked
                pointY = self.world.pitch.height
            elif self.their_attacker.y > top_split:
                # Top blocked
                pointY = 0
            else:
                # Go whichever way is closest
                if self.our_defender.y > midpont:
                    pointY = self.world.pitch.height
                else:
                    pointY = 0

            if self.world._our_side == 'right':
                pointX = 448
            else:
                pointX = 70

            displacement, angle = self.our_defender.get_direction_to_point(pointX, pointY)
            return calculate_motor_speed(displacement, angle)

    def align_horiz(self):
        # aim horizontally
       
        angle = self.our_defender.get_rotation_to_point(self.world.our_attacker.x, self.world.our_attacker.y)

        action = calculate_motor_speed(None, angle, careful=True)
        
        if action['left_motor'] > 0:
            self.clockwise = True
        elif action['left_motor'] < 0:
            self.clockwise = False
        else:
            self.current_state = self.STOP
            self.stopTime = time.clock()

        return action
        
    def align_mid_far(self):
        # aligns to middle of far away wall to do a bounce pass
        x_aim = self.world.pitch.width/2
        y_aim = 0

        if (self.our_defender.y < self.world.pitch.height/2):
            y_aim = self.world.pitch.height

        angle = self.our_defender.get_rotation_to_point(x_aim, y_aim)
        
        action = calculate_motor_speed(None, angle, careful=True)
        
        if action['left_motor'] > 0:
            self.clockwise = True
        elif action['left_motor'] < 0:
            self.clockwise = False
        else:
            self.current_state = self.STOP
            self.stopTime = time.clock()
        
        return action

    def align_straight(self):
        # aligns perpendicular because path is blocked

        angle = self.our_defender.get_rotation_to_point(self.world.our_attacker.x, self.world.our_defender.y)

        action = calculate_motor_speed(None, angle, careful=True)
        
        if action['left_motor'] > 0:
            self.clockwise = True
        elif action['left_motor'] < 0:
            self.clockwise = False
        else:
            self.current_state = self.STOP
            self.stopTime = time.clock()
        
        return action

    def stop(self):
        # Fires the motors for a short burst in the opposite direction from the turn
        # Counteracts the delay in stopping after the command is sent
        timeNow = time.clock()
        if (timeNow - self.stopTime > 0.3):
            self.shootReadyTime = time.clock()
            self.current_state = self.SHOOT
            return do_nothing()
        else:
            speed = TURNING_SPEED_CAREFUL

            if self.clockwise:
                action = {'left_motor':-speed, 'right_motor':speed}
            else:
                action = {'left_motor':speed, 'right_motor':-speed}
            return action

    def shoot(self):
        currentTime = time.clock()
        if (currentTime - self.shootReadyTime) > 0.2:
            self.current_state = self.WAIT
            self.shootTime = currentTime
            self.our_defender.catcher = 'open'
            return kick_ball(DEFAULT_KICK_POWER)
        else:
            return do_nothing()

    def wait(self):
        currentTime = time.clock()
        if (currentTime - self.shootTime) > 0.2:
            self.current_state = self.GET_BALL
        return do_nothing()


class SimpleBlock(Strategy):
    # For controlling _defender_

    PREPARE, FOLLOW = \
        'PREPARE', 'FOLLOW'
    STATES = [PREPARE, FOLLOW]

    def __init__(self, world):
        super(SimpleBlock, self).__init__(world, self.STATES)
        self.NEXT_ACTION_MAP = {
            self.PREPARE: self.prepare,
            self.FOLLOW: self.follow
        }

        if self.world._our_side == 'left':
            self.min_x = 60
            self.max_x = 90
        else:
            self.min_x = 450
            self.max_x = 470

        self.our_attacker = self.world.our_attacker
        self.their_attacker = self.world.their_attacker
        self.our_defender = self.world.our_defender
        self.ball = self.world.ball

    def prepare(self):
        self.current_state = self.FOLLOW
        if self.our_attacker.catcher != 'open':
            self.our_attacker.catcher = 'open'
            return open_catcher()
        else:
            return do_nothing()

    def follow(self):

        x_aim = min(self.max_x, max(self.min_x, self.our_defender.x))

        predicted_y = None
        # Predict where they are aiming.
        if self.ball.velocity > BALL_VELOCITY:
            predicted_y = predict_y_intersection(self.world, x_aim, self.ball, bounce=False)

        if predicted_y is None:
            predicted_y = self.ball.y
            predicted_y = min(max(predicted_y, 90), self.world._pitch.height - 90)

        displacement, angle = self.our_defender.get_direction_to_point(x_aim, predicted_y)
        action = calculate_motor_speed_defence(displacement, angle, backwards_ok=True)

        return action

class DefenderPenalty(Strategy):


    STAY, DEFEND_GOAL, STOP = \
        'STAY','DEFEND_GOAL', 'STOP'
    STATES = [STAY, DEFEND_GOAL, STOP]
    LEFT, RIGHT = 'left', 'right'
    SIDES = [LEFT, RIGHT]


    def __init__(self, world):
        super(DefenderPenalty, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.STAY: self.stay,
            self.DEFEND_GOAL: self.defend_goal,
            self.STOP: self.stop,
        }

        self.their_attacker = self.world.their_attacker
        self.our_defender = self.world.our_defender
        self.ball = self.world.ball
        self.was_shot = False

    def stay(self):
        kicker_threshold = 3
        if self.ball.velocity > kicker_threshold and self.ball.velocity<50 :
            self.current_state = self.DEFEND_GOAL
            return do_nothing()
        else:
            return do_nothing()



    def defend_goal(self):

        if self.our_defender.can_remotely_defend_ball(self.ball):
            self.our_defender.catcher = 'closed'
            self.current_state = self.STOP
            return grab_ball()

        predicted_y = None
        # Predict where they are aiming.
        if self.ball.velocity > BALL_VELOCITY:
            predicted_y = predict_y_intersection(self.world, self.our_defender.x, self.ball, bounce=False)

        if predicted_y is not None:
            displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x,
                                                                           predicted_y - 7*math.sin(self.our_defender.angle))
            action = calculate_motor_speed(displacement, angle, backwards_ok=True)
        else:
            y = self.ball.y
            y = max([y, 60])
            y = min([y, self.world._pitch.height - 60])
            displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x, y)
            action = calculate_motor_speed(displacement, angle, backwards_ok=True)

        return action

    def stop(self):
        return do_nothing();
