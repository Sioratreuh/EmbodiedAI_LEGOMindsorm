import argparse
from time import sleep

from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, MoveTank, SpeedPercent, LineFollowErrorLostLine, LineFollowErrorTooFast, follow_for_ms, follow_for_forever



class RobotConfig:
    # I/O Values
    LEFT_WHEEL = OUTPUT_A
    RIGHT_WHEEL = OUTPUT_D
    
    # Color Sensor constants
    BLACK = 10  #TODO: check what values of black we get
    WHITE = 60 #TODO: check what values of white we get
    COLORS = ('unknown','black','blue','green','yellow','red','white','brown') #In case we use the color sensors
    COLOR_CHECK = {color:position for position, color in enumerate(COLORS)} # can be used to check if black is the color returned by the sensor COLORS_CHECK.get('black')==self.colorSensor.value()
    
    # Line Control Constants
    ##* Initial values tank example from https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html) ##
    ### PID Controls ###
    KP = 11.3 # Proportional
    KI = 0.05 # Integral
    KD = 3.2  # Derivative
    
    FOLLOW_LEFT_EDGE = True  #True if the sensor is on the left 
    FOLLOW_FOR=follow_for_ms #Controls for how long to follow the line, is either follow_forever or follow_for_ms and then specify a variable ms=something
    FOLLOW_LINE_TIME = 4500  #Controls the time for the loop to restart if
    
    # Speed Constants
    MAX_SPEED = 100
    WORKING_SPEED = 50
    
    
    
class LineFollowingRobot:
    def __init__(   self,
                    left_wheel=RobotConfig.LEFT_WHEEL,
                    right_wheel=RobotConfig.RIGHT_WHEEL,
                    working_speed=RobotConfig.WORKING_SPEED,
                    kp=RobotConfig.KP,
                    ki=RobotConfig.KI,
                    kd=RobotConfig.KD,
                    black=RobotConfig.BLACK,
                    white=RobotConfig.WHITE,
                    follow_left_edge=RobotConfig.FOLLOW_LEFT_EDGE,
                    follow_for=RobotConfig.FOLLOW_FOR,
                    follow_line_time=RobotConfig.FOLLOW_LINE_TIME):
        print("Creating Robot")
        #Variables
        self.left_wheel = left_wheel
        self.right_wheel = right_wheel
        self.working_speed = working_speed
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.black = black
        self.white = white
        self.follow_left_edge = follow_left_edge
        self.follow_for = follow_for
        self.follow_line_time = follow_line_time
        
        # Create Device CLasses
        self.sound = Sound()
        self.button = Button()
        self.colorSensor = ColorSensor()
        self.tank = MoveTank(self.left_wheel, self.right_wheel)
        self.sound.beep()

    def start(self):
        print("Starting the line following process")
        while not self.button.on_enter: # Stops if the central button is pressed
            # There is an option to add a follow time
            try:
                self.tank.follow_line(
                    kp=self.kp,
                    ki=self.ki,
                    kd=self.kd,
                    speed=SpeedPercent(self.working_speed),
                    target_light_intensity=self.black,
                    follow_left_edge=self.follow_left_edge,
                    white=self.white,
                    follow_for=RobotConfig.FOLLOW_FOR,
                    ms=self.follow_line_time
                )
            except LineFollowErrorLostLine:
                # If it happens we can consider making it slower
                # This happens when the robot is moving too fas for the PID
                print("EXCEPTION: Robot moving too fast for the PID control")
            except LineFollowErrorLostLine:
                # Here we can try to do something to ensure it returns to the line
                print("EXCEPTION: Robot lost the line")
            
            self.tank.off() #Just in case
            self.sound.beep()



def get_arguments():
    """Gets the arguments introduced in the command line when running the program
    """
    print("Obtainning arguments")
    
    parser = argparse.ArgumentParser(description="Lego Mindstorms Line Following Robot")
    parser.add_argument("--l_w", type=str, default=RobotConfig.LEFT_WHEEL, help="Left wheel motor port (outA, outB, outC, outD)")
    parser.add_argument("--r_w", type=str, default=RobotConfig.RIGHT_WHEEL, help="Right wheel motor port (outA, outB, outC, outD)")
    parser.add_argument("--sp", type=int, default=RobotConfig.WORKING_SPEED, help="Working speed percentage (0-100)")
    parser.add_argument("--kp", type=float, default=RobotConfig.KP, help="")
    parser.add_argument("--ki", type=float, default=RobotConfig.KI, help="")
    parser.add_argument("--kd", type=float, default=RobotConfig.KD, help="")
    parser.add_argument("--b", type=int, default=RobotConfig.BLACK, help="Intensity for the line(0-255)")
    parser.add_argument("--w", type=int, default=RobotConfig.WHITE, help="Intensity value for outside the line (0-255)")
    parser.add_argument("--follow_left_edge", type=bool, default=RobotConfig.FOLLOW_LEFT_EDGE, help="True if sensor is on the left side")
    parser.add_argument("--follow_time", type=int, default=RobotConfig.FOLLOW_LINE_TIME, help="Time in ms to follow line before doing other actions")
    
    
    args = parser.parse_args()
    return args


def main():
    args = get_arguments()
    robot = LineFollowingRobot(
        left_wheel=args.l_w,
        right_wheel=args.r_w,
        working_speed=args.sp,
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        black=args.b,
        white=args.w,
        follow_left_edge=args.follow_left_edge,
        follow_line_time=args.follow_time
    )

    robot.start()
if __name__ == "__main__":
    main()
