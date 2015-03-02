from utilities import *
import math
from random import randint
import time

DEFAULT_KICK_POWER = 80

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


class Milestone3Catch(Strategy):
    # For controlling _defender_

    # (Check if enemy defender on pitch?)
    # (If so, skip to turning to face vertically (face attacker?))
    # Turn to be facing verically
    # Stay in line with other robot
    # When it stops, we open catcher
    # Then turn to receive
    # Should be possible to go back to following from here
    # Grab when the ball is near enough
    # (If grab fails, try to pick up ball?)

    PREPARE, ALIGN_DOWN, FOLLOW, PREPARE_CATCH, ALIGN_PASS, WAIT = 'PREPARE', 'ALIGN_DOWN', 'FOLLOW', 'PREPARE_CATCH', 'ALIGN_PASS', 'WAIT'
    STATES = [PREPARE, ALIGN_DOWN, FOLLOW, PREPARE_CATCH, ALIGN_PASS, WAIT]

    def __init__(self, world):
        super(Milestone3Catch, self).__init__(world, self.STATES)
        self.NEXT_ACTION_MAP = {
            self.PREPARE: self.prepare,
            self.ALIGN_DOWN: self.align_down,
            self.FOLLOW: self.follow,
            self.PREPARE_CATCH: self.prepare_catch,
            self.ALIGN_PASS: self.align_pass
        }

        self.our_attacker = self.world.our_attacker
        self.their_attacker = self.world.their_attacker
        self.our_defender = self.world.our_defender
        self.ball = self.world.ball


    def prepare(self):
        self.current_state = self.ALIGN_DOWN
        return do_nothing()

    def align_down(self):
        # This could just check our_defender.angle
        angle = self.our_defender.get_rotation_to_point(self.our_defender.x, 0)
        action = calculate_motor_speed(None, angle, careful=True)
        if action['left_motor'] == 0 and action['right_motor'] == 0:
            self.current_state = self.FOLLOW
            return do_nothing()
        else:
            return action

    def follow(self):
        if in_line(self.our_defender, self.our_attacker):
            self.current_state = self.PREPARE_CATCH
            return do_nothing()
        else:
            displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x,
                                                                           self.our_attacker.y)
            return calculate_motor_speed(displacement, angle, careful=True, backwards_ok=True)

    def prepare_catch(self):
        self.current_state = self.ALIGN_PASS
        self.our_attacker.catcher = 'open'
        return open_catcher()

    def align_pass(self):
        # Check to move onto grab/chase ball if time
        if in_line(self.our_defender, self.our_attacker):
            angle = self.our_defender.get_rotation_to_point(self.our_attacker.x,
                                                            self.our_attacker.y)
            action = calculate_motor_speed(None, angle, careful=True, backwards_ok=True)
            return action
        else:
            self.current_state = self.ALIGN_DOWN
            self.our_attacker.catcher = 'closed'
            return grab_ball()


class Milestone3Kick(Strategy):
    # For controlling _defender_

    # Check if enemy defender on pitch
    # If so, find point where we can shoot past
    # Move to this point
    # (Maybe move while pass would be blocked? - need to choose direction)
    # Stop
    # Turn to face other robot (continuously)
    # Once it has stopped moving (including turning) pass
    # (After waiting a short moment?)
    # (Does current world state include stationary rotation?)
    # (Might need to store direction locally)

    # TODO = 'TODO'
    # STATES = [TODO]

    # def __init__(self, world):
    #     super(Milestone3Kick, self).__init__(world, self.STATES)
    #     self.NEXT_ACTION_MAP = {
    #         self.TODO: self.todo
    #     }
        
    #     self.our_attacker = self.world.our_attacker
    #     self.their_attacker = self.world.their_attacker
    #     self.our_defender = self.world.our_defender
    #     self.ball = self.world.ball

    # def todo(self):
    #     print "running"

    PREPARE, GO_TO_BALL, GRAB_BALL, ALIGN, SHOOT, FINISH = \
        'PREPARE', 'GO_TO_BALL', 'GRAB_BALL', 'ALIGN', 'SHOOT', 'FINISH'
    STATES = [PREPARE, GO_TO_BALL, GRAB_BALL, ALIGN, SHOOT, FINISH]

    def __init__(self, world):
        super(Milestone3Kick, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.PREPARE: self.prepare,
            self.GO_TO_BALL: self.position,
            self.GRAB_BALL: self.grab,
            self.ALIGN: self.align,
            self.SHOOT: self.shoot,
            self.FINISH: self.finish
        }

        self.our_attacker = self.world.our_attacker
        self.our_defender = self.world.our_defender
        self.ball = self.world.ball

    def prepare(self):
        self.current_state = self.GO_TO_BALL
        if self.our_defender.catcher == 'closed':
            self.our_defender.catcher = 'open'
            return open_catcher()
        else:
            return do_nothing()

    def position(self):
        displacement, angle = self.our_defender.get_direction_to_point(self.ball.x, self.ball.y)
        if self.our_defender.can_catch_ball(self.ball):
            # self.current_state = self.GRAB_BALL
            return {}
        else:
            return calculate_motor_speed(None, angle, careful=True)

    def grab(self):
        if self.our_defender.has_ball(self.ball):
            self.current_state = self.ALIGN
            return do_nothing()
        else:
            self.our_defender.catcher = 'closed'
            return grab_ball()

    def align(self):
        displacement, angle = self.our_defender.get_direction_to_point(self.world.their_goal.x, self.world.their_goal.y)
        action = calculate_motor_speed(None, angle, careful=True)
        if action['left_motor'] == 0 and action['right_motor'] == 0:
            action = calculate_motor_speed(None, angle, careful=True)
            if action['left_motor'] == 0 and action['right_motor'] == 0:
                self.current_state = self.SHOOT
                return do_nothing()
            else:
                return action
        else:
            return action
    
    def shoot(self):
        self.our_defender.catcher = 'open'
        self.current_state = self.FINISH
        return kick_ball(DEFAULT_KICK_POWER)

    
    def finish(self):
        return do_nothing()

class Milestone2Attacker(Strategy):

    PREPARE, GO_TO_BALL, GRAB_BALL, ALIGN, SHOOT, FINISH = \
        'PREPARE', 'GO_TO_BALL', 'GRAB_BALL', 'ALIGN', 'SHOOT', 'FINISH'
    STATES = [PREPARE, GO_TO_BALL, GRAB_BALL, ALIGN, SHOOT, FINISH]

    def __init__(self, world):
        super(Milestone2Attacker, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.PREPARE: self.prepare,
            self.GO_TO_BALL: self.position,
            self.GRAB_BALL: self.grab,
            self.ALIGN: self.align,
            self.SHOOT: self.shoot,
            self.FINISH: self.finish
        }

        self.our_attacker = self.world.our_attacker
        self.ball = self.world.ball

    def prepare(self):
        self.current_state = self.GO_TO_BALL
        if self.our_attacker.catcher == 'closed':
            self.our_attacker.catcher = 'open'
            return open_catcher()
        else:
            return do_nothing()

    def position(self):
        displacement, angle = self.our_attacker.get_direction_to_point(self.ball.x, self.ball.y)
        if self.our_attacker.can_catch_ball(self.ball):
            self.current_state = self.GRAB_BALL
            return {}
        else:
            return calculate_motor_speed(displacement, angle, careful=True)

    def grab(self):
        if self.our_attacker.has_ball(self.ball):
            self.current_state = self.ALIGN
            return do_nothing()
        else:
            self.our_attacker.catcher = 'closed'
            return grab_ball()

    def align(self):
        displacement, angle = self.our_attacker.get_direction_to_point(self.world.their_goal.x, self.world.their_goal.y)
        action = calculate_motor_speed(None, angle, careful=True)
        if action['left_motor'] == 0 and action['right_motor'] == 0:
            action = calculate_motor_speed(None, angle, careful=True)
            if action['left_motor'] == 0 and action['right_motor'] == 0:
                self.current_state = self.SHOOT
                return do_nothing()
            else:
                return action
        else:
            return action
    
    def shoot(self):
        self.our_attacker.catcher = 'open'
        self.current_state = self.FINISH
        return kick_ball(DEFAULT_KICK_POWER)

    
    def finish(self):
        return do_nothing()

class Milestone2Defender(Strategy):

    STAY, TURN, DEFEND_GOAL, POSITION, GRAB_BALL, ALIGN, SHOOT, FINISH = \
        'STAY', 'TURN', 'DEFEND_GOAL', 'POSITION', 'GRAB_BALL', 'ALIGN', 'SHOOT', 'FINISH'
    STATES = [STAY, TURN, DEFEND_GOAL, POSITION, GRAB_BALL, ALIGN, SHOOT, FINISH]
    LEFT, RIGHT = 'left', 'right'
    SIDES = [LEFT, RIGHT]

    GOAL_ALIGN_OFFSET = 60

    def __init__(self, world):
        super(Milestone2Defender, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.STAY: self.stay,
            self.TURN: self.turn,
            self.DEFEND_GOAL: self.defend_goal,
            self.POSITION: self.position,
            self.GRAB_BALL: self.grab_ball,
            self.ALIGN: self.align,
            self.SHOOT: self.shoot,
            self.FINISH: self.finish
        }

        self.our_goal = self.world.our_goal
        self.their_attacker = self.world.their_attacker
        self.our_defender = self.world.our_defender
        self.ball = self.world.ball

    def stay(self):
        kicker_threshold = 3
        if self.ball.velocity > kicker_threshold and self.ball.velocity<50 :
            self.current_state = self.DEFEND_GOAL
            return do_nothing()
        else:
            return do_nothing()

    def turn(self):        
        predicted_y = None
        # Predict where they are aiming.
        if self.ball.velocity > BALL_VELOCITY:
            predicted_y = predict_y_intersection(self.world, self.our_defender.x, self.ball, bounce=False)

        if predicted_y is None:
            predicted_y = self.ball.y

        print predicted_y

        displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x + 30, predicted_y)

        print angle

        action = calculate_motor_speed(None, angle)

        if action['left_motor'] == 0 and action['right_motor'] == 0:
            self.current_state = self.DEFEND_GOAL
            return do_nothing()
        else:
            return action

    def defend_goal(self):
        """
        Run around, blocking shots.
        """

        # LB: IMPORTANT - only works from one side of pitch

        # Move to pick up the ball if it has stopped moving roughly towards you
        # or if it has stopped moving altogether
        # and it is close to us
        if ((self.ball.angle > (5.0/4.0)*pi or 
                self.ball.angle < (3.0/4.0)*pi or 
                self.ball.velocity < BALL_VELOCITY) and 
                abs(self.ball.x - self.our_defender.x) < 80):
            self.current_state = self.POSITION
            return do_nothing()

        x_aim = self.our_defender.x

        predicted_y = None
        # Predict where they are aiming.
        if self.ball.velocity > BALL_VELOCITY:
            predicted_y = predict_y_intersection(self.world, x_aim, self.ball, bounce=False)

        print predicted_y
        if predicted_y is not None:
            displacement, angle = self.our_defender.get_direction_to_point(x_aim,
                                                                           predicted_y - 7*math.sin(self.our_defender.angle))
            action = calculate_motor_speed(displacement, angle, backwards_ok=True)
        else:
            y = self.ball.y
            y = max([y, 60])
            y = min([y, self.world._pitch.height - 60])
            displacement, angle = self.our_defender.get_direction_to_point(x_aim, y)
            action = calculate_motor_speed(displacement, angle, backwards_ok=True)

        return action

    def position(self):
        displacement, angle = self.our_defender.get_direction_to_point(self.ball.x, self.ball.y)
        if self.our_defender.can_catch_ball(self.ball):
            self.current_state = self.GRAB_BALL
            action = {}
        else:
            action = calculate_motor_speed(displacement, angle, careful=True)

        return action

    # LB: In the milestone we saw the robot fail to grab the ball at this state 
    # and end up looping here constantly grabbing the ball
    # Need to tighten up strategy to avoid this situations
    def grab_ball(self):
        if self.our_defender.has_ball(self.ball):
            self.current_state = self.ALIGN
            return do_nothing()
        else:
            self.our_defender.catcher = 'closed'
            return grab_ball()

    def align(self):
        displacement, angle = self.our_defender.get_direction_to_point(self.world.their_goal.x, self.world.their_goal.y)
        action = calculate_motor_speed(None, angle, careful=True)
        if action['left_motor'] == 0 and action['right_motor'] == 0:
            self.current_state = self.SHOOT
            return do_nothing()
        else:
            return action
    
    def shoot(self):
        self.our_defender.catcher = 'open'
        self.current_state = self.FINISH
        return kick_ball(DEFAULT_KICK_POWER)

    
    def finish(self):
        return do_nothing()



    def get_alignment_position(self, side):
        """
        Given the side, find the x coordinate of where we need to align to initially.
        """
        assert side in self.SIDES
        if side == self.LEFT:
            return self.world.our_goal.x + self.GOAL_ALIGN_OFFSET
        else:
            return self.world.our_goal.x - self.GOAL_ALIGN_OFFSET


class DefenderPenalty(Strategy):


    DEFEND_GOAL = 'DEFEND_GOAL'
    STATES = [DEFEND_GOAL]
    LEFT, RIGHT = 'left', 'right'
    SIDES = [LEFT, RIGHT]


    def __init__(self, world):
        super(DefenderPenalty, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.DEFEND_GOAL: self.defend_goal
        }

        self.their_attacker = self.world.their_attacker
        self.our_defender = self.world.our_defender
        self.ball = self.world.ball


    def defend_goal(self):
        """
        Run around, blocking shots.
        """
        # Predict where they are aiming.
        if self.ball.velocity > BALL_VELOCITY:
            predicted_y = predict_y_intersection(self.world, self.our_defender.x, self.ball, bounce=False)

        if self.ball.velocity <= BALL_VELOCITY or predicted_y is None: 
            predicted_y = predict_y_intersection(self.world, self.our_defender.x, self.their_attacker, bounce=False)

        if predicted_y is not None:
            displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x,
                                                                           predicted_y - 7*math.sin(self.our_defender.angle))
            return calculate_motor_speed(displacement, angle, backwards_ok=True)
        else:
            y = self.ball.y
            y = max([y, 60])
            y = min([y, self.world._pitch.height - 60])
            displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x, y)
            return calculate_motor_speed(displacement, angle, backwards_ok=True)


class DefenderGrab(Strategy):

    DEFEND, GO_TO_BALL, GRAB_BALL, GRABBED = 'DEFEND', 'GO_TO_BALL', 'GRAB_BALL', 'GRABBED'
    STATES = [DEFEND, GO_TO_BALL, GRAB_BALL, GRABBED]

    def __init__(self, world):
        super(DefenderGrab, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.DEFEND: self.defend,
            self.GO_TO_BALL: self.position,
            self.GRAB_BALL: self.grab,
            self.GRABBED: do_nothing
        }

        self.our_defender = self.world.our_defender
        self.ball = self.world.ball

    def defend(self):
        '''
        If the ball is heading towards our goal at high velocity then don't go directly into
        grabbing mode once the ball enters our zone. Try to match it's y-coordinate as fast as possible.
        '''
        if self.ball.velocity > BALL_VELOCITY:
            predicted_y = predict_y_intersection(self.world, self.our_defender.x, self.ball, bounce=True)

            if predicted_y is not None:
                displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x,
                                                                               predicted_y - 7*math.sin(self.our_defender.angle))
                return calculate_motor_speed(displacement, angle, backwards_ok=True)
        
        self.current_state = self.GO_TO_BALL
        return do_nothing()

    def position(self):
        displacement, angle = self.our_defender.get_direction_to_point(self.ball.x, self.ball.y)
        if self.our_defender.can_catch_ball(self.ball):
            self.current_state = self.GRAB_BALL
            return do_nothing()
        else:
            return calculate_motor_speed(displacement, angle, careful=True)

    def grab(self):
        if self.our_defender.has_ball(self.ball):
            self.current_state = self.GRABBED
            return do_nothing()
        else:
            self.our_defender.catcher = 'closed'
            return grab_ball()


class AttackerScoreDynamic(Strategy):
    """
    Goal scoring tactic. Assumes it will be executed when the robot has grabbed the ball.

    Outline:
        1) Position the robot closer to the border line.
        2) Move to aim into one corner of the goal.
        3) Re-rotate and aim into the other corner
        4) Shoot

    Effectivness:
        * Only effective if their attacker is not standing on the white line
          close to us. They need to be at least 40px (the side facing us) from
          the division line between defender and attacker.
        * If opponent's grabber is extended we may not get any leavway for scoring.
          This assumes that they effectively predict direction and optimize for
          maximum blocking area.

    Maths:
        * When the opponent is defending ideally, we have about 15 degrees leaveway to
          score
        * Each ~5 pixels away from the ideal position we lose 2 degrees
            - Imprecision of 15px results in highly unprobable score in CONFUSE1.
            - Probability of scoring increases in CONFUSE2
        * Size of their grabber in extended position is not factored in

    TODO:
        * Finish implementing
        * After CONFUSE1, check if we have a clear shot at the goal and shoot
            - Defender's velocity should be taken into consideration
                - if velocity high, we are better off pulling off the CONFUSE2 part
                - if low, best to try to shoot as opponent's vision/delay may not pickup the trick
        * Attempt to pick sides based on their robot velocity as well
        * Contigency
            - If both CONFUSE1 and CONFUSE2 fail, we may switch strategies or resort to a shot
              either UP or DOWN, based on their position.
    """
    GRABBED, POSITION = 'GRABBED', 'POSITION'
    CONFUSE1, CONFUSE2, SHOOT = 'CONFUSE1', 'CONFUSE2', 'SHOOT'
    STATES = [GRABBED, POSITION, CONFUSE1, CONFUSE2, SHOOT]

    UP, DOWN = 'UP', 'DOWN'
    GOAL_SIDES = [UP, DOWN]

    SHOOTING_X_OFFSET = 85
    GOAL_CORNER_OFFSET = 55

    def __init__(self, world):
        super(AttackerScoreDynamic, self).__init__(world, self.STATES)
        # Map states into functions
        self.NEXT_ACTION_MAP = {
            self.GRABBED: self.position,
            self.POSITION: self.confuse_one,
            self.CONFUSE1: self.confuse_two,
            self.CONFUSE2: self.shoot,
            self.SHOOT: self.shoot
        }

        self.our_attacker = self.world.our_attacker
        self.their_defender = self.world.their_defender

        # Find the position to shoot from and cache it
        self.shooting_pos = self._get_shooting_coordinates(self.our_attacker)

        # Remember which side we picked first
        self.fake_shoot_side = None

    def generate(self):
        """
        Pick an action based on current state.
        """
        print 'BALL', self.world.ball
        return self.NEXT_ACTION_MAP[self.current_state]()

    def position(self):
        """
        Position the robot in the middle close to the goal. Angle does not matter.
        Executed initially when we've grabbed the ball and want to move.
        """
        ideal_x, ideal_y = self.shooting_pos
        distance, angle = self.our_attacker.get_direction_to_point(ideal_x, ideal_y)

        if has_matched(self.our_attacker, x=ideal_x, y=ideal_y):
            # We've reached the POSITION state.
            self.current_state = self.POSITION
            return self.confuse_one()

        # We still need to drive
        return calculate_motor_speed(distance, angle)

    def confuse_one(self):
        """
        Pick a side and aim at it. Executed when we've reached the POSITION state.
        """
        # Initialize fake shoot side if not available
        if self.fake_shoot_side is None:
            self.fake_shoot_side = self._get_fake_shoot_side(self.their_defender)

        target_x = self.world.their_goal.x
        target_y = self._get_goal_corner_y(self.fake_shoot_side)

        print 'SIDE:', self.fake_shoot_side

        print 'TARGET_Y', target_y
        print 'STATE:', self.current_state

        distance, angle = self.our_attacker.get_direction_to_point(target_x, target_y)

        print 'DIRECTION TO POINT', distance, angle

        if has_matched(self.our_attacker, angle=angle):
            # TODO: Shoot if we have a clear shot and the oppononet's velocity is favourable for us
            y = self.their_defender.y
            middle = self.world.pitch.height / 2

            opp_robot_side = self._get_fake_shoot_side(self.their_defender)
            if opp_robot_side != self.fake_shoot_side:
                # We've finished CONFUSE1
                self.current_state = self.CONFUSE1
                return self.confuse_two()
            else:
                return calculate_motor_speed(0, 0)

        # Rotate on the spot
        return calculate_motor_speed(None, angle)

    def confuse_two(self):
        """
        Rotate to the other side and make them go 'Wow, much rotate'.
        """
        other_side = self._get_other_side(self.fake_shoot_side)
        print 'OTHER SIDE:', other_side
        # Determine targets
        target_x = self.world.their_goal.x
        target_y = self._get_goal_corner_y(other_side)

        print 'OTHER SIDE TARGET Y', target_y

        angle = self.our_attacker.get_rotation_to_point(target_x, target_y)

        print 'OTHER SIDE ANGLE:', angle

        if has_matched(self.our_attacker, angle=angle, angle_threshold=self.PRECISE_BALL_ANGLE_THRESHOLD):
            # We've finished CONFUSE2
            self.current_state = self.SHOOT
            return self.shoot()
            # pass
            pass

        # Rotate on the spot
        return calculate_motor_speed(None, angle, careful=True)

    def shoot(self):
        """
        Kick.
        """
        self.current_state = self.SHOOT
        return kick_ball(DEFAULT_KICK_POWER)

    def _get_shooting_coordinates(self, robot):
        """
        Retrive the coordinates to which we need to move before we set up the confuse shot.
        """
        zone_index = robot.zone
        zone_poly = self.world.pitch.zones[zone_index][0]

        # Find the x coordinate of where we need to go
        # Find which function to use, min for us on the right, max otherwise
        f = max if zone_index == 2 else min
        x = int(f(zone_poly, key=lambda z: z[0])[0])

        # Offset x to be a wee bit inside our zone
        x = x - self.SHOOTING_X_OFFSET if zone_index == 2 else x + self.SHOOTING_X_OFFSET

        # y is simply middle of the pitch
        y = self.world.pitch.height / 2

        return (x, y)

    def _get_fake_shoot_side(self, robot):
        """
        Compare the location of their robot with the middle to pick the first side
        """
        y = robot.y
        middle = self.world.pitch.height / 2
        return self.UP if y < middle else self.DOWN

    def _get_other_side(self, side):
        """
        Determine the other side to rotate to based on the CONFUSE1 side.
        """
        assert side in self.GOAL_SIDES
        return self.UP if side == self.DOWN else self.DOWN

    def _get_goal_corner_y(self, side):
        """
        Get the coordinates of where to aim / shoot.
        """
        assert side in self.GOAL_SIDES
        if side == self.UP:
            # y coordinate of the goal is DOWN, offset by the width
            return self.world.their_goal.y + self.world.their_goal.width / 2 - int(self.GOAL_CORNER_OFFSET * 1.5)
        return self.world.their_goal.y - self.world.their_goal.width / 2 + self.GOAL_CORNER_OFFSET + 20


class AttackerTurnScore(Strategy):
    """
    Move up and down the opponent's goal line and suddenly turn 90 degrees and kick if the
    path is clear.
    """

    UNALIGNED, POSITION, KICK, FINISHED = 'UNALIGNED', 'POSITION', 'KICK', 'FINISHED'
    STATES = [UNALIGNED, POSITION, KICK, FINISHED]

    def __init__(self, world):
        super(AttackerTurnScore, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.UNALIGNED: self.align,
            self.POSITION: self.position,
            self.KICK: self.kick,
            self.FINISHED: do_nothing
        }

        self.their_goal = self.world.their_goal
        self.our_attacker = self.world.our_attacker
        self.their_defender = self.world.their_defender

        # Distance that the attacker should keep from its boundary.
        self.offset = 60

        # Opponent's goal edge where our attacker is currently heading.
        self.point = 0

    def align(self):
        '''
        Go to the boundary of the attacker's zone and align with the center
        of the goal line.
        '''
        ideal_x = self._get_alignment_x()
        ideal_y = self.their_goal.y

        if has_matched(self.our_attacker, x=ideal_x, y=ideal_y):
            self.current_state = self.POSITION
            return do_nothing()
        else:
            distance, angle = self.our_attacker.get_direction_to_point(ideal_x, ideal_y)
            return calculate_motor_speed(distance, angle)

    def position(self):
        '''
        Go up an down the goal line waiting for the first opportunity to shoot.
        '''
        our_attacker = self.our_attacker
        # Check if we have a clear shot
        if not is_attacker_shot_blocked(self.world, self.our_attacker, self.their_defender) and \
               (abs(our_attacker.angle - math.pi / 2) < math.pi / 20 or \
               abs(our_attacker.angle - 3*math.pi/2) < math.pi / 20):
            self.current_state = self.KICK
            return self.kick()

        else:
            # If our shot is blocked, continue moving up and down the goal line.
            # We want the center of the robot to be inside the goal line.
            goal_width = self.their_goal.width/2
            goal_edges = [self.their_goal.y - goal_width + 10,
                          self.their_goal.y + goal_width - 10]
            ideal_x = self.our_attacker.x
            ideal_y = goal_edges[self.point]

            if has_matched(self.our_attacker, x=self.our_attacker.x, y=ideal_y):
                # Go to the other goal edge
                self.point = 1 - self.point
                ideal_y = goal_edges[self.point]

            distance, angle = self.our_attacker.get_direction_to_point(ideal_x, ideal_y)
            return calculate_motor_speed(distance, angle, backwards_ok=True)

    def kick(self):
        # Decide the direction of the right angle turn, based on our position and
        # side on the pitch.
        if self.world._our_side == 'left':
            if self.our_attacker.angle > 0 and self.our_attacker.angle < math.pi:
                orientation = -1
            else:
                orientation = 1
        else:
            if self.our_attacker.angle > 0 and self.our_attacker.angle < math.pi:
                orientation = 1
            else:
                orientation = -1

        self.current_state = self.FINISHED
        return turn_shoot(orientation)

    def _get_alignment_x(self):
        # Get the polygon of our attacker's zone.
        zone = self.our_attacker.zone
        assert zone in [1,2]
        zone_poly = self.world.pitch.zones[zone][0]

        # Choose the appropriate function to determine the borderline of our
        # attacker's zone facing the opponent's goal.
        side = {1: min, 2: max}
        f = side[zone]

        # Get the x coordinate that our attacker needs to match.
        sign = {1: 1, 2: -1}
        boundary_x = int(f(zone_poly, key=lambda z: z[0])[0]) + sign[zone]*self.offset
        return boundary_x

