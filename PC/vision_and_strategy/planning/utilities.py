from math import tan, pi, hypot, log
from planning.models import Robot

DISTANCE_MATCH_THRESHOLD = 15
ANGLE_MATCH_THRESHOLD = pi/10
BALL_ANGLE_THRESHOLD = pi/20
CURVE_THRESHOLD = pi/5
CURVE_SPEED_DIFF = 10

FORWARD_SPEED = 80
FORWARD_SPEED_CAREFUL = 40
TURNING_SPEED = 40
TURNING_SPEED_CAREFUL = 40
TURNING_SPEED_SUPER_CAREFUL = 35

BALL_VELOCITY = 3

def in_line(robot1, robot2, careful=False):
    '''
    Checks if robot1 and robot2 are horizontally in line
    '''
    if careful:
        threshold = DISTANCE_MATCH_THRESHOLD
    else:
        threshold = DISTANCE_MATCH_THRESHOLD + 10
    return abs(robot1.y - robot2.y) < threshold

def is_facing(robot1, robot2, careful=False):
    '''
    Checks if robot1 is facing robot2
    '''
    if careful:
        threshold = BALL_ANGLE_THRESHOLD
    else:
        threshold = ANGLE_MATCH_THRESHOLD
    angle = robot1.get_rotation_to_point(robot2.x, robot2.y)
    return abs(angle) < threshold

def is_shot_blocked(world, our_robot, their_robot):
    '''
    Checks if our robot could shoot past their robot
    '''
    predicted_y = predict_y_intersection(
        world, their_robot.x, our_robot, full_width=True, bounce=True)
    if predicted_y is None:
        return True
    print '##########', predicted_y, their_robot.y, their_robot.length
    print abs(predicted_y - their_robot.y) < their_robot.length
    return abs(predicted_y - their_robot.y) < their_robot.length


def is_attacker_shot_blocked(world, our_attacker, their_defender):
    '''
    Checks if our attacker would score if it would immediately turn and shoot.
    '''

    # Acceptable distance that the opponent defender can be relative to our
    # shooting position in order for us to have a clear shot.
    distance_threshold = 40

    # Return True if attacker and defender ar close to each other on
    # the y dimension
    return abs(our_attacker.y - their_defender.y) < distance_threshold


def can_score(world, our_robot, their_goal, turn=0):
    # Offset the robot angle if need be
    robot_angle = our_robot.angle + turn
    goal_zone_poly = world.pitch.zones[their_goal.zone][0]

    reverse = True if their_goal.zone == 3 else False
    goal_posts = sorted(goal_zone_poly, key=lambda x: x[0], reverse=reverse)[:2]
    # Makes goal be sorted from smaller to bigger
    goal_posts = sorted(goal_posts, key=lambda x: x[1])

    goal_x = goal_posts[0][0]

    robot = Robot(
        our_robot.zone, our_robot.x, our_robot.y, robot_angle % (pi * 2), our_robot.velocity)

    predicted_y = predict_y_intersection(world, goal_x, robot, full_width=True)

    return goal_posts[0][1] < predicted_y < goal_posts[1][1]

def predict_y_intersection(world, predict_for_x, robot, full_width=False, bounce=False):
        '''
        Predicts the (x, y) coordinates of the ball shot by the robot
        Corrects them if it's out of the bottom_y - top_y range.
        If bounce is set to True, predicts for a bounced shot
        Returns None if the robot is facing the wrong direction.
        '''
        x = robot.x
        y = robot.y
        top_y = world._pitch.height - 60 if full_width else world.our_goal.y + (world.our_goal.width/2) - 0
        bottom_y = 60 if full_width else world.our_goal.y - (world.our_goal.width/2) + 0
        angle = robot.angle
        if (robot.x < predict_for_x and not (pi/2 < angle < 3*pi/2)) or (robot.x > predict_for_x and (3*pi/2 > angle > pi/2)):
            if bounce:
                if not (0 <= (y + tan(angle) * (predict_for_x - x)) <= world._pitch.height):
                    bounce_pos = 'top' if (y + tan(angle) * (predict_for_x - x)) > world._pitch.height else 'bottom'
                    x += (world._pitch.height - y) / tan(angle) if bounce_pos == 'top' else (0 - y) / tan(angle)
                    y = world._pitch.height if bounce_pos == 'top' else 0
                    angle = (-angle) % (2*pi)
            predicted_y = (y + tan(angle) * (predict_for_x - x))
            # Correcting the y coordinate to the closest y coordinate on the goal line:
            if predicted_y > top_y:
                return top_y
            elif predicted_y < bottom_y:
                return bottom_y
            return predicted_y
        else:
            return None


def grab_ball():
    return {'catcher': 1}


def kick_ball(power):
    return {'kicker': power}


def open_catcher():
    return {'drop': 1}

# Not currently supported by arduino
def turn_shoot(orientation):
    return {'turn_90': orientation, 'left_motor': 0, 'right_motor': 0, 'kicker': 1, 'catcher': 0, 'speed': 1000}


def has_matched(robot, x=None, y=None, angle=None,
                angle_threshold=ANGLE_MATCH_THRESHOLD, distance_threshold=DISTANCE_MATCH_THRESHOLD):
    dist_matched = True
    angle_matched = True
    if not(x is None and y is None):
        dist_matched = hypot(robot.x - x, robot.y - y) < distance_threshold
    if not(angle is None):
        angle_matched = abs(angle) < angle_threshold
    return dist_matched and angle_matched


def calculate_motor_speed_defence(displacement, angle, backwards_ok=False, careful=False):
    '''
    Complicated view of calculating the speed for defence
    '''

    # Could potentially calculate careful turning speed based on angle
    # and slow down as we get closer
    if careful:
        threshold = BALL_ANGLE_THRESHOLD
    else:
        threshold = ANGLE_MATCH_THRESHOLD

    if angle is not None:

        # Multiplier is negative if the motors are to run backwards
        multiplier = 1

        # Check if we can get there with less turning by going backwards
        moving_backwards = False
        if backwards_ok and abs(angle) > pi/2:
            moving_backwards = True
            if angle > 0:
                angle = (angle - pi)
            else:
                angle = (angle + pi)
            multiplier = -1
        
        if (abs(angle) < CURVE_THRESHOLD and abs(angle) > threshold and 
                displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD):
            # Move forward curving
            turnSpeedHigh = min(100, TURNING_SPEED_CAREFUL + CURVE_SPEED_DIFF)
            turnSpeedLow = max(0, TURNING_SPEED_CAREFUL - CURVE_SPEED_DIFF)           
            if angle <= 0:
                return {'left_motor': turnSpeedLow, 'right_motor': turnSpeedHigh}
            else:
                return {'left_motor': turnSpeedHigh, 'right_motor': turnSpeedLow}
        elif abs(angle) > threshold:
            if careful:
                turnSpeed = TURNING_SPEED_CAREFUL * multiplier
            else:
                turnSpeed = TURNING_SPEED * multiplier

            if angle <= 0:
                return {'left_motor': -turnSpeed, 'right_motor': turnSpeed}
            else:
                return {'left_motor': turnSpeed, 'right_motor': -turnSpeed}
        elif displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD:
            if careful:
                speed = FORWARD_SPEED_CAREFUL * multiplier
            else:
                speed = FORWARD_SPEED * multiplier
            return {'left_motor': speed, 'right_motor': speed}
            
        else:
            return {'left_motor': 0, 'right_motor': 0}
        
    elif displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD:
        if careful:
            speed = FORWARD_SPEED_CAREFUL
        else:
            speed = FORWARD_SPEED
        return {'left_motor': speed, 'right_motor': speed}
    else:
        return {'left_motor': 0, 'right_motor': 0}


def calculate_motor_speed(displacement, angle, backwards_ok=False, careful=False):
    '''
    Simplistic view of calculating the speed
    '''

    # Could potentially calculate careful turning speed based on angle
    # and slow down as we get closer
    if careful:
        threshold = BALL_ANGLE_THRESHOLD
    else:
        threshold = ANGLE_MATCH_THRESHOLD

    if angle is not None:

        # Multiplier is negative if the motors are to run backwards
        multiplier = 1

        # Check if we can get there with less turning by going backwards
        moving_backwards = False
        if backwards_ok and abs(angle) > pi/2:
            moving_backwards = True
            if angle > 0:
                angle = (angle - pi)
            else:
                angle = (angle + pi)
            multiplier = -1
        
        if (abs(angle) < CURVE_THRESHOLD and abs(angle) > threshold and 
                displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD):
            # Move forward curving
            turnSpeedHigh = min(100, TURNING_SPEED_CAREFUL + CURVE_SPEED_DIFF)
            turnSpeedLow = max(0, TURNING_SPEED_CAREFUL - CURVE_SPEED_DIFF)           
            if moving_backwards:
                # Backwards arc
                if angle <= 0:
                    return {'left_motor': -turnSpeedHigh, 'right_motor': -turnSpeedLow}
                else:
                    return {'left_motor': -turnSpeedLow, 'right_motor': -turnSpeedHigh}
            else:
                # Forwards arc
                if angle <= 0:
                    return {'left_motor': turnSpeedLow, 'right_motor': turnSpeedHigh}
                else:
                    return {'left_motor': turnSpeedHigh, 'right_motor': turnSpeedLow}
        elif abs(angle) > threshold:
            if careful:
                turnSpeed = TURNING_SPEED_CAREFUL * multiplier
            else:
                turnSpeed = TURNING_SPEED * multiplier

            if angle <= 0:
                return {'left_motor': -turnSpeed, 'right_motor': turnSpeed}
            else:
                return {'left_motor': turnSpeed, 'right_motor': -turnSpeed}
        elif displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD:
            if careful:
                speed = FORWARD_SPEED_CAREFUL * multiplier
            else:
                speed = FORWARD_SPEED * multiplier
            return {'left_motor': speed, 'right_motor': speed}
            
        else:
            return {'left_motor': 0, 'right_motor': 0}
        
    elif displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD:
        if careful:
            speed = FORWARD_SPEED_CAREFUL
        else:
            speed = FORWARD_SPEED
        return {'left_motor': speed, 'right_motor': speed}
    else:
        return {'left_motor': 0, 'right_motor': 0}


def calculate_motor_speed_for_catch(displacement, angle, backwards_ok=False, careful=False):
    '''
    calculate moving for catching the ball
    '''

    # Could potentially calculate careful turning speed based on angle
    # and slow down as we get closer
    if careful:
        threshold = BALL_ANGLE_THRESHOLD
    else:
        threshold = ANGLE_MATCH_THRESHOLD

    if angle is not None:

        # Multiplier is negative if the motors are to run backwards
        multiplier = 1
        # Check if we can get there with less turning by going backwards
        if backwards_ok and abs(angle) > pi/2:
            moving_backwards = True
            if angle > 0:
                angle = (angle - pi)
            else:
                angle = (angle + pi)
            multiplier = -1
        
        # if (abs(angle) < CURVE_THRESHOLD and abs(angle) > threshold and 
        #         displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD):
        #     # Move forward curving
        #     turnSpeedHigh = min(100, TURNING_SPEED_SUPER_CAREFUL + CURVE_SPEED_DIFF)
        #     turnSpeedLow = max(0, TURNING_SPEED_SUPER_CAREFUL - CURVE_SPEED_DIFF)           
        #     if angle <= 0:
        #         return {'left_motor': turnSpeedLow, 'right_motor': turnSpeedHigh}
        #     else:
        #         return {'left_motor': turnSpeedHigh, 'right_motor': turnSpeedLow}
        if abs(angle) > threshold:
            if careful:
                turnSpeed = TURNING_SPEED_SUPER_CAREFUL * multiplier
            else:
                turnSpeed = TURNING_SPEED * multiplier

            if angle <= 0:
                return {'left_motor': -turnSpeed, 'right_motor': turnSpeed}
            else:
                return {'left_motor': turnSpeed, 'right_motor': -turnSpeed}
        elif displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD:
            if careful:
                speed = TURNING_SPEED_SUPER_CAREFUL * multiplier
            else:
                speed = FORWARD_SPEED * multiplier
            return {'left_motor': speed, 'right_motor': speed}
            
        else:
            return {'left_motor': 0, 'right_motor': 0}
        
    elif displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD:
        if careful:
            speed = TURNING_SPEED_SUPER_CAREFUL
        else:
            speed = FORWARD_SPEED
        return {'left_motor': speed, 'right_motor': speed}
    else:
        return {'left_motor': 0, 'right_motor': 0}


def do_nothing():
    return {'left_motor': 0, 'right_motor': 0, 'kicker': 0, 'catcher': 0}
