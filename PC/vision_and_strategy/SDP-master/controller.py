from vision.vision import Vision, Camera, GUI
from planning.planner import Planner
from postprocessing.postprocessing import Postprocessing
from preprocessing.preprocessing import Preprocessing
import vision.tools as tools
from cv2 import waitKey
import cv2
import serial
import warnings
import time


warnings.filterwarnings("ignore", category=DeprecationWarning)


class Controller:
    """
    Primary source of robot control. Ties vision and planning together.
    """

    def __init__(self, pitch, color, our_side, video_port=0, comm_port='/dev/ttyACM0', comms=1):
        """
        Entry point for the SDP system.

        Params:
            [int] video_port                port number for the camera
            [string] comm_port              port number for the arduino
            [int] pitch                     0 - main pitch, 1 - secondary pitch
            [string] our_side               the side we're on - 'left' or 'right'
            *[int] port                     The camera port to take the feed from
            *[Robot_Controller] attacker    Robot controller object - Attacker Robot has a RED
                                            power wire
            *[Robot_Controller] defender    Robot controller object - Defender Robot has a YELLOW
                                            power wire
        """
        assert pitch in [0, 1]
        assert color in ['yellow', 'blue']
        assert our_side in ['left', 'right']

        self.pitch = pitch

        # Set up the Arduino communications
        self.arduino = Arduino(comm_port, 115200, 1, comms)

        # Set up camera for frames
        self.camera = Camera(port=video_port, pitch=self.pitch)
        frame = self.camera.get_frame()
        center_point = self.camera.get_adjusted_center(frame)

        # Set up vision
        self.calibration = tools.get_colors(pitch)
        self.vision = Vision(
            pitch=pitch, color=color, our_side=our_side,
            frame_shape=frame.shape, frame_center=center_point,
            calibration=self.calibration)

        # Set up postprocessing for vision
        self.postprocessing = Postprocessing()

        # Set up main planner
        self.planner = Planner(our_side=our_side, pitch_num=self.pitch)

        # Set up GUI
        self.GUI = GUI(calibration=self.calibration, arduino=self.arduino, pitch=self.pitch)

        self.color = color
        self.side = our_side

        self.preprocessing = Preprocessing()

        self.robot = Robot_Controller()

    # LB: This looks important, name should probably change
    # Seems to be the main loop
    def wow(self):
        """
        Ready your sword, here be dragons.
        """
        counter = 1L
        timer = time.clock()
        try:
            c = True
            while c != 27:  # the ESC key

                ### Vision ###

                frame = self.camera.get_frame()
                pre_options = self.preprocessing.options
                # Apply preprocessing methods toggled in the UI
                preprocessed = self.preprocessing.run(frame, pre_options)
                frame = preprocessed['frame']
                if 'background_sub' in preprocessed:
                    cv2.imshow('bg sub', preprocessed['background_sub'])
                # Find object positions
                # model_positions have their y coordinate inverted

                model_positions, regular_positions = self.vision.locate(frame)
                model_positions = self.postprocessing.analyze(model_positions)

                ### Planning ###

                self.planner.update_world(model_positions)
                attacker_actions = None
                defender_actions = None
                attackerState = None
                defenderState = None


                # test
                attacking = False





                # Milestone 2: we are either attacking or defending
                if attacking:
                    # LB: again with the two robots
                    attacker_actions = self.planner.plan('attacker')

                    if self.robot is not None:
                        self.robot.execute(self.arduino, attacker_actions)

                    # Information about states
                    # LB: These should also include the current strategy name
                    attackerState = (self.planner.attacker_state, self.planner.attacker_strat_state)
                else:   
                    defender_actions = self.planner.plan('defender')

                    print defender_actions

                    if self.robot is not None:
                        self.robot.execute(self.arduino, defender_actions)

                    defenderState = (self.planner.defender_state, self.planner.defender_strat_state)

                ### Interface ###

                # Use 'y', 'b', 'r' to change color.
                c = waitKey(2) & 0xFF
                # LB: why are we making this empty here? It passes to the GUI, should we not keep track of the actions?
                actions = []
                fps = float(counter) / (time.clock() - timer)

                # Information about the grabbers from the world
                # LB: Does this need to be defined here? Can we not send them (to the gui) directly?
                grabbers = {
                    'our_defender': self.planner._world.our_defender.catcher_area,
                    'our_attacker': self.planner._world.our_attacker.catcher_area
                }

                # Draw vision content and actions
                self.GUI.draw(
                    frame, model_positions, actions, regular_positions, fps, attackerState,
                    defenderState, attacker_actions, defender_actions, grabbers,
                    our_color=self.color, our_side=self.side, key=c, preprocess=pre_options)
                counter += 1

        except:
            if self.robot is not None:
                self.robot.shutdown(self.arduino)
            raise

        finally:
            # Write the new calibrations to a file.
            tools.save_colors(self.pitch, self.calibration)
            if self.robot is not None:
                self.robot.shutdown(self.arduino)



class Robot_Controller(object):
    """
    Robot_Controller class for robot control.
    """

    def __init__(self):
        """
        Initialise variables
        """
        self.current_speed = 0

    def execute(self, comm, action):
        """
        Execute robot action.
        """

        # LB: arduino control assumptions turned out to be wrong:
        # 'left_motor' and 'right_motor' are values between 
        # -(utilities.MAX_DISPLACEMENT_SPEED) and (utilities.MAX_DISPLACEMENT_SPEED)
        # or between -(utilities.MAX_ANGLE_SPEED) and (utilities.MAX_ANGLE_SPEED)
        # Team 7 used stepper motors, so these seem to actually represent a number of steps
        # Roughly, how long to fire the motors for
        # This might need translating to our system
        #
        # speed (sent to arduino twice?) is an extra value (equivalent to max speed/acceleration)
        # When moving straight forwards, it is some complicated value
        # otherwise, it is one of two values - (magic numbers) 95 or 300 (depending on "carefullness")
        #
        # Best bet might be to ignore the (general) speed 
        # and simply transform the left/right values to "speeds"

        # Do whatever actions are specified in the action dict
        # To kick without affecting wheels, don't send 'left_motor' or 'right_motor' at all
        if 'left_motor' in action or 'right_motor' in action:
            left_motor = 0
            right_motor = 0
            if 'left_motor' in action:
                left_motor = int(action['left_motor'])
            if 'right_motor' in action:
                right_motor = int(action['right_motor'])
            msg = 'RUN_ENG %d %d\r' % (max(min(left_motor, 99), -99), max(min(right_motor, 99), -99))
            comm.write(msg)

        if 'speed' in action:
            # LB: need to decide whether to use this or not
            speed = action['speed']

        if 'kicker' in action and action['kicker'] != 0:
            try:
                comm.write('RUN_KICK\r')
                # Let the kick finish before we tell it what else to do
                time.sleep(0.5)
            except StandardError:
                pass
        elif 'catcher' in action and action['catcher'] != 0:
            try:
                comm.write('RUN_CATCH\r')
            except StandardError:
                pass

    def shutdown(self, comm):
        comm.write('RUN_KICK\r')
        comm.write('RUN_ENG %d %d\r' % (0, 0))


class Arduino:

    def __init__(self, port, rate, timeOut, comms):
        self.serial = None
        self.comms = comms
        self.port = port
        self.rate = rate
        self.timeout = timeOut
        self.setComms(comms)
        self.thing = 0

    def setComms(self, comms):
        if comms > 0:
            self.comms = 1
            if self.serial is None:
                try:
                    self.serial = serial.Serial(self.port, self.rate, timeout=self.timeout)
                except:
                    print self.port
                    print self.rate
                    print self.timeout
                    print "No Arduino detected!"
                    print "Continuing without comms."
                    self.comms = 0
        else:
            self.write('RUN_ENG %d %d\r' % (0, 0))
            self.comms = 0

    def write(self, string):
        if self.comms == 1:
            if self.thing % 50 == 0:
                print string
                self.serial.write(string)
        self.thing += 1


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("pitch", help="[0] Main pitch, [1] Secondary pitch")
    parser.add_argument("side", help="The side of our defender ['left', 'right'] allowed.")
    parser.add_argument("color", help="The color of our team - ['yellow', 'blue'] allowed.")
    parser.add_argument(
        "-n", "--nocomms", help="Disables sending commands to the robot.", action="store_true")

    args = parser.parse_args()
    if args.nocomms:
        c = Controller(
            pitch=int(args.pitch), color=args.color, our_side=args.side, comms=0).wow()
    else:
        c = Controller(
            pitch=int(args.pitch), color=args.color, our_side=args.side).wow()
