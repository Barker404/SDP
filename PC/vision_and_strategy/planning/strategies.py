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

    PREPARE, GET_BALL, AVOID, ALIGN_HORIZ, ALIGN_MID_FAR, ALIGN_STRAIGHT, SHOOT, WAIT = \
        'PREPARE', 'GET_BALL', 'AVOID', 'ALIGN_HORIZ', 'ALIGN_MID_FAR', 'ALIGN_STRAIGHT', 'SHOOT', 'WAIT'
    STATES = [PREPARE, GET_BALL, AVOID, ALIGN_HORIZ, ALIGN_MID_FAR, ALIGN_STRAIGHT, SHOOT, WAIT]

    def __init__(self, world):
        super(SimplePass, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.PREPARE: self.prepare,
            self.GET_BALL: self.get_ball,
            self.AVOID: self.avoid,
            self.ALIGN_HORIZ: self.align_horiz,
            self.ALIGN_MID_FAR: self.align_mid_far,
            self.ALIGN_STRAIGHT: self.align_straight,
            self.SHOOT: self.shoot,
            self.WAIT: self.wait
        }

        self.catchTime = -1

        self.TIME_LIMIT = 8

        self.SPACE_THRESHOLD = 60

        self.our_attacker = self.world.our_attacker
        self.our_defender = self.world.our_defender
        self.their_attacker = self.world.their_attacker
        self.ball = self.world.ball

    def prepare(self):
        self.current_state = self.GET_BALL
        if self.our_defender.catcher == 'closed':
            self.our_defender.catcher = 'open'
            return open_catcher()
        else:
            return do_nothing()

    def get_ball(self):

        path = self.world.our_defender.get_pass_path(self.world.our_attacker)
        BLOCKED = path.overlaps(Polygon(self.world.their_attacker.get_polygon()))
        print BLOCKED;


        displacement, angle = self.our_defender.get_direction_to_point(self.ball.x, self.ball.y)
        if self.our_defender.can_catch_ball(self.ball):
            self.current_state = self.AVOID
            self.catchTime = time.clock()
            self.our_defender.catcher = 'closed'
            return grab_ball()
        else:
            if (displacement > 40):
                print "sfsdfh"
                return calculate_motor_speed(displacement, angle, careful=True)
            else:
                return calculate_motor_speed_for_catch(displacement, angle, careful=True)

####################################################################################################################
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
####################################################################################################################

    def align_horiz(self):
        # aim horizontally
       
        angle = self.our_defender.get_rotation_to_point(self.world.our_attacker.x, self.world.our_attacker.y)

        action = calculate_motor_speed(None, angle, careful=True)

        if action['left_motor'] == 0 and action['right_motor'] == 0:
            self.shootReadyTime = time.clock()
            self.current_state = self.SHOOT
            return action
        else:
            return action

    def align_mid_far(self):            #aligns to middle of far away wall to do a bounce pass
        x_aim = self.world.pitch.width/2
        y_aim = 0

        if (self.our_defender.y < self.world.pitch.height/2):
            y_aim = self.world.pitch.height

        angle = self.our_defender.get_rotation_to_point(x_aim, y_aim)
        
        action = calculate_motor_speed(None, angle, careful=True)

        if action['left_motor'] == 0 and action['right_motor'] == 0:
            self.shootReadyTime = time.clock()
            self.current_state = self.SHOOT
            return action
        else:
            return action

    def align_straight(self):       # aligns perpendicular because path is blocked

        angle = self.our_defender.get_rotation_to_point(self.world.our_attacker.x, self.world.our_defender.y)

        action = calculate_motor_speed(None, angle, careful=True)

        if action['left_motor'] == 0 and action['right_motor'] == 0:
            self.shootReadyTime = time.clock()
            self.current_state = self.SHOOT
            return do_nothing()
        else:
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
