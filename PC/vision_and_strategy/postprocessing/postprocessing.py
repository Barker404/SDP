from planning.models import Vector
from copy import deepcopy
from math import atan2, pi, hypot
import math
import time
import numpy as np
class Postprocessing(object):
    '''
    Postprocessing performs analysis of the PitchObjects after the vision module
    has performed it's locating. This generally calculates angle and velocity
    measurements.
    '''

    #: Number of frame vectors to store for lookup
    CACHE_SIZE = 5
    CACHE_MEASURE = 100
    #: Stores the average time between post-processing analyze calls
    average_time_diff = 0
    count = 0

    #: Stores the last CACHE_SIZE vectors in dictionary form
    _frames = []
    _measure = []
    def __init__(self):
        '''
        Initialise the postprocessing at time 0 with default positions for all.
        '''
        self._time = time.time()
        _vectors = {}
        _vectors['ball'] = {'vec': Vector(0, 0, 0, 0), 'time': self._time}
        _vectors['our_attacker'] = {'vec': Vector(0, 0, 0, 0), 'time': self._time}
        _vectors['their_attacker'] = {'vec': Vector(0, 0, 0, 0), 'time': self._time}
        _vectors['our_defender'] = {'vec': Vector(0, 0, 0, 0), 'time': self._time}
        _vectors['their_defender'] = {'vec': Vector(0, 0, 0, 0), 'time': self._time}

        self._frames.append(_vectors)
        self._measure.append(_vectors)
    def analyze(self, vector_dict):
        '''
        This method analyzes current positions and collects them into a dictionary
        before returning them.

        :param vector_dict: An old dictionary containing vectors for each PitchObject
        :returns: A new vector dictionary containing analysed data for this frame.
        '''

        # Increase our time for each analyze call
        self.count += 1

        self.average_time_diff = ((self.average_time_diff * (self.count-1)) + time.time() - self._time) / self.count
        self._time = time.time()

        new_vector_dict = {}
        new_vector = {}
        for name, info in vector_dict.iteritems():
            if name == 'ball':
                new_vector_dict[name] = self.analyze_ball(info)
            else:
                new_vector_dict[name] = self.analyze_robot(name, info)

            new_vector[name] = {}
            new_vector[name]['vec'] = new_vector_dict[name]
            new_vector[name]['time'] = self._time

        # Check if we have more frames than desired, and pop the first if so
        if len(self._frames) > self.CACHE_SIZE:
            self._frames.pop(0)
        if len(self._measure) > self.CACHE_MEASURE:
            self._measure.pop(0)
        # Either way, append our new vector frame to the cache
        self._frames.append(new_vector)
        self._measure.append(new_vector)
        return new_vector_dict

    def analyze_ball(self, info):
        '''
        Calculates the new angle and velocity of the ball using a simple time calculation and
        change in direction from the previous info dictionary.

        :param info: The previous postprocessing dictionary for the ball
        :returns: A new vector dictionary containing x,y,angle,velocity for the ball.
        '''
        if not(info['x'] is None) and not (info['y'] is None):
            
            # Calculate change in the balls movement

            
            # Use difference between current time and previous time as the duration of movement
            delta_x = info['x'] - self._frames[-1]['ball']['vec'].x
            delta_y = info['y'] - self._frames[-1]['ball']['vec'].y
            velocity = hypot(delta_y, delta_x)
            if velocity == 0:
                angle = 0
            else:
                angle = atan2(delta_y, delta_x) % (2*pi)
            return Vector(int(info['x']), int(info['y']), angle, velocity)
        else:
            return deepcopy(self._frames[-1]['ball']['vec'])

    def analyze_robot(self, key, info):
        '''
        Calculates the angle and velocity of a robot given the new frame and the change in time.

        :param key: A robot key: ['our_attacker', 'their_attacker', 'our_defender', 'their_defender']
        :param info: The previous vector dictionary for this robot.
        '''
        if not(info['x'] is None) and not(info['y'] is None) and not(info['angle'] is None):

            # Retrieve last vector frame for reference
            old_vector = self._frames[-1][key]

            # Reject angle if it has altered further than it should have
            old_angle = old_vector['vec'].angle
            robot_angle = info['angle']

            diff = min((2 * pi) - abs(old_angle - robot_angle), abs(old_angle - robot_angle))
            
            #: pi/2 is the maximum angle turn allowed in a single frame before we reject this as a vision
            #: error
            if len(self._frames) > self.CACHE_SIZE and diff > pi/2:
                info['angle'] = old_angle
                robot_angle = old_angle
                # print 'Rejected new angle'

            delta_x = info['x'] - old_vector['vec'].x
            delta_y = info['y'] - old_vector['vec'].y

            # Calculate the angle of the delta vector relative to (1, 0)
            #delta_angle = atan2(delta_y, delta_x)

            # Offset the angle if negative, we only want positive values
           # delta_angle = delta_angle % (2*math.pi)

            velocity = hypot(delta_y, delta_x)
         
            return Vector(info['x'], info['y'], info['angle'], velocity)
        else:
            return deepcopy(self._frames[-1][key]['vec'])

