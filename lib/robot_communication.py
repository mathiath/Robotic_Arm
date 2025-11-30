"""
robot_communication library

This library provides the RobotSerial class fo communicating
between RoboDK and an Ardino-controlled robot with serial port.
"""
import sys
from time import sleep
import serial
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
from serial.serialutil import SerialException

class RobotSerial:
    """Class to communicate with Arduino via serial port
     Objects of this class tries to connect to the port when created,
     the prot is different based on the operating system:
        - Windows: 'COM5' or equivalent
        - Linux: '/dev/ttyACM0' or equivalent
        - MacOS: '/dev/tty.usbmodemXXXX' or equivalent"""
    def __init__(self, port, tcp_speed=30, baudrate=115200, read_delay=0.1, write_delay=0.1):
        try:
            self.ser = serial.Serial(port, baudrate)
        except SerialException:
            print("The robot is't connected")
            sys.exit(0)
        self.RDK = Robolink()
        self.robot = self.RDK.Item('', ITEM_TYPE_ROBOT)
        sleep(read_delay)  #Waiting for serial to initialize
        self.write_delay = write_delay
        self.tcp_speed = tcp_speed
        sleep(1.5) #Satrtup delay
        self.homing_robot() #getting positions of all joints
        self.robot_speed_control(tcp_speed)
        print("")

    def home(self):
        """Move robot to home posison"""
        home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_signal(home)
        print(f"Robot going to home: {home}")
        self.robot.MoveJ(home)

    def joint_values_for_target(self, target):
        """Retrieves all the angles of all the joints of the target and stores them in a list"""
        outer_list = self.robot.SolveIK(target.Pose())
        list_of_joints = outer_list.list()
        try:
            #Uses the enumerate function to create indexes on all elements in the list,
            # in order to be able to change the current element.
            for idx, i in enumerate(list_of_joints):
                round_value = round(i, 2)
                list_of_joints[idx] = round_value
            if list_of_joints and len(list_of_joints) > 0:
                return list_of_joints
        except AttributeError:
            print("Target is missing Pose() or the IK-result is invalid")

        except Exception as e:
            print(f"{e}")

    def MoveJ(self, target):
        """Sending the angles to all joints and moves on the simulation with MoveJ"""
        joint_values = self.joint_values_for_target(target)
        if joint_values is not None:
            self.send_signal(joint_values)
            self.read_signal()
            self.robot.MoveJ(joint_values)
        else:
            print(f"Kunne ikke hente leddverdier for target '{joint_values}'.")

    def robot_speed_control(self, speed):
        """Setts a speed for all jonits, the speed has to be between 1 and 50"""
        under_speed_limet = 1
        upper_speed_limet = 51
        if speed <= under_speed_limet or speed >= upper_speed_limet:
            print(f"The set robot speed is not between {under_speed_limet} and {upper_speed_limet}")
            self.robot.setSpeed(50)
            self.send_signal(50, "L")
            self.read_signal()
            self.tcp_speed = 50
        else:
            self.robot.setSpeed(speed)
            self.send_signal(speed, "L")
            self.read_signal()

    def homing_robot(self):
        """Asks the robot the current encoder values"""
        self.send_signal("","E")
        self.read_signal()

    def correcting_robodk(self, encoder_data):
        """"Moves the robot in RoboDK based on the encoder values from the real robot"""
        self.robot.setJoints(encoder_data)

    def read_signal(self):
        """Reading signals from Arduino."""
        read_time = True
        while read_time is True:
            if self.ser.in_waiting > 0:
                read_serial = self.ser.readline().decode('utf-8').strip()
                if read_serial == "end":
                    read_time = False
                elif read_serial.startswith("Encoder:"):
                    read = read_serial[8:].split(",")
                    text_list = []
                    for i in read:
                        text_list.append(float(i))
                    print(f"Encoder values from robot: {text_list}")
                    self.correcting_robodk(text_list)
                else:
                    print(f"{read_serial}")

    def send_signal(self, info, message_type="R"):
        """Sends signals to the Arduino using Serial."""
        if message_type == "R": #Degrees for all the joints
            info = 'R' + str(info) + '\n'

        elif message_type == "S": #Speed for all the joints
            info = 'S' + str(info) + '\n'

        elif message_type == "L": #Robot Speed
            info = 'L' + str(info) + '\n'

        elif message_type == "E": #Asking for encoder values
            info = 'E' + '\n'

        elif message_type == "G": #Activating or deactivating gripper
            info = 'G' + str(info) + '\n'

        self.ser.write(info.encode('utf-8'))
        sleep(self.write_delay)

    def close(self):
        """Closing the connection with Arduino"""
        if self.ser.is_open:
            self.ser.close()

    def setDO(self, io_bool):
        """Sending gripper IO signals"""
        self.send_signal(io_bool, "G")
        self.read_signal()
