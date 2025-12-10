"""
A.T.L.A.BOT library

This library provides the RobotSerial class fo communicating
between RoboDK and an Arduino-controlled robot with serial port.
"""
import sys
from time import sleep
import serial
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
from robodk import transl
from serial.serialutil import SerialException

class RobotSerial:
    """Class to communicate with Arduino via serial port
     Objects of this class tries to connect to the port when
     created, the prot is different based on the operating system:
        - Windows: 'COM5' or equivalent"""
    def __init__(self, port, gripper_length=(0,0,0), robot_speed=10, baudrate=115200, read_delay=0.1, write_delay=0.1):
        try: # trying to connect to the robot with Serial
            self.__ser = serial.Serial(port, baudrate)
        except SerialException:
            print("The robot is not connected")
            sys.exit(0)
        self.__RDK = Robolink()
        self.__robot = self.__RDK.Item('ATLABOT', ITEM_TYPE_ROBOT)
        sleep(read_delay)  #Waiting for serial to initialize
        self.__write_delay = write_delay
        self.__gripper_length = gripper_length #UR gripper = (0,20,-130) | custom TCP = (0,0,-55)
        self.__robot_speed = robot_speed
        sleep(1.5) #Satrtup delay
        self.__homing_robot() #getting positions of all joints
        self.__robot_speed_control(self.__robot_speed)
        print("")

    def home(self):
        """Move robot to home posison"""
        home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.__send_signal(home)
        print(f"Robot going to home: {home}")
        self.__robot.MoveJ(home)
        self.__read_signal()

    def MoveJ(self, target):
        """Sending the angles to all joints and moves on the simulation with MoveJ"""
        joint_values = self.__joint_values_for_target(target)
        if joint_values is not None:
            self.__send_signal(joint_values)
            self.__read_signal()
            self.__robot.MoveJ(joint_values)
        else:
            print(f"Could not get joint values from target {joint_values}.")

    def Joints(self):
        """Return the current joint position as a robodk.robomath.Mat()"""
        return self.__robot.Joints()

    def Pose(self):
        """Returns the relative pose of an object"""
        return self.__robot.Pose()

    def setDO(self, io_bool):
        """Sending gripper IO signals"""
        self.__send_signal(io_bool, "G")
        self.__read_signal()

    def __joint_values_for_target(self, target):
        """Retrieves all the angles of all the joints of the target and stores them in a list"""
        if str(target).startswith("RoboDK"): #checks if target is a direct object of robodk
            outer_list = self.__robot.SolveIK(target.Pose() * transl(self.__gripper_length[0],self.__gripper_length[1],self.__gripper_length[2]))
        elif str(target).startswith("Pose"): #checks if target is a Pose objekt
            outer_list = self.__robot.SolveIK(target * transl(self.__gripper_length[0],self.__gripper_length[1],self.__gripper_length[2]))
        else:
            outer_list = target

        if type(outer_list) == list:
            inner_list = outer_list
        else:
            inner_list = outer_list.list()

        try:
            #Uses the enumerate function to create indexes on all elements in the list, in order to be able to change the current element.
            list_of_joints = []
            for i in inner_list:
                round_value = round(i, 2)
                list_of_joints.append(float(round_value))

            if list_of_joints and len(list_of_joints) > 0:
                return list_of_joints
        except AttributeError:
            print("Target is missing Pose() or the IK-result is invalid")

        except Exception as e:
            print(f"{e}")

    def __robot_speed_control(self, speed):
        """Setts a speed for all jonits, the speed has to be between 1 and 90"""
        under_speed_limit = 1
        upper_speed_limit = 31
        if speed <= under_speed_limit or speed >= upper_speed_limit:
            print(f"The robot Speed must be between {under_speed_limit} and {upper_speed_limit - 1}")
            sys.exit(0)
        else:
            self.__robot.setSpeed(speed)
            self.__send_signal(speed, "L")
            self.__read_signal()

    def __homing_robot(self):
        """Asks the robot the current encoder values"""
        self.__send_signal("","E")
        self.__read_signal()

    def __correcting_robodk(self, encoder_data):
        """Moves the robot in RoboDK based on the encoder values from the real robot"""
        self.__robot.setJoints(encoder_data)

    def __read_signal(self):
        """Reading signals from Arduino."""
        read_time = True
        while read_time is True:
            if self.__ser.in_waiting > 0:
                read_serial = self.__ser.readline().decode('utf-8').strip()
                if read_serial == "end":
                    read_time = False
                elif read_serial.startswith("Encoder:"):
                    read = read_serial[8:].split(",")
                    text_list = []
                    for i in read:
                        text_list.append(float(i))
                    print(f"Encoder values from robot: {text_list}")
                    self.__correcting_robodk(text_list)
                elif read_serial.startswith("Emergency"):
                    print("------------------------")
                    print("Robot is in emergency stop!")
                    print("------------------------")
                    sys.exit(0)
                else:
                    print(f"{read_serial}")

    def __send_signal(self, info, message_type="R"):
        """Sends signals to the Arduino using Serial."""
        if message_type == "R": #Degrees for all the joints
            info = 'R' + str(info) + '\n'

        elif message_type == "L": #Robot Speed
            info = 'L' + str(info) + '\n'

        elif message_type == "E": #Asking for encoder values
            info = 'E' + '\n'

        elif message_type == "G": #Activating or deactivating gripper
            info = 'G' + str(info) + '\n'

        self.__ser.write(info.encode('utf-8'))
        sleep(self.__write_delay)
