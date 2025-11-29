import sys
from time import sleep
import serial
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
from serial.serialutil import SerialException

class RobotSerial:
    """Klasse for å kommunisere med Arduino via serieport
     Når et objekt av denne klassen opprettes, prot-enn er ulike basert på operativsystemet:
        - Windows: 'COM3' eller tilsvarende
        - Linux: '/dev/ttyACM0' eller tilsvarende
        - MacOS: '/dev/tty.usbmodemXXXX' eller tilsvarende"""
    def __init__(self, port, tcp_speed=30, baudrate=115200, read_delay=0.1, write_delay=0.1):
        try:
            self.ser = serial.Serial(port, baudrate)
        except SerialException:
            print("The robot is't connected")
            sys.exit(0)
        self.RDK = Robolink()
        self.robot = self.RDK.Item('', ITEM_TYPE_ROBOT)
        sleep(read_delay)  # Venter på at serial skal initialisere
        self.write_delay = write_delay
        self.tcp_speed = tcp_speed
        sleep(1.5) # Satrtup delay
        self.homing_robot()
        self.robot_speed_control(tcp_speed)
        print("")

    def home(self):
        """Move robot to home posison"""
        home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_signal(home)
        print(f"Robot going to home: {home}")
        self.robot.MoveJ(home)

    def joint_values_for_target(self, target):
        """Henter ut alle vinklene til alle leddene og leger dem i en liste"""
        #item_type = target.Type()
        #if item_type == ITEM_TYPE_FRAME:
        outer_list = self.robot.SolveIK(target.Pose())
        list_of_joints = outer_list.list()
        #else:
            #list_of_joints = target.Joints().list()
        try:
            for idx, i in enumerate(list_of_joints): #Bruker enumerate funksjoen for å lage indeser på alle elementent i listen, for å kunne endre vært elemment
                round_value = round(i, 2)
                list_of_joints[idx] = round_value
            if list_of_joints and len(list_of_joints) > 0:
                return list_of_joints
        except Exception as e:
            print(f"{e}")

    def MoveJ(self, target):
        """Sender vinklene til alle leddene og flyter på simulasjonen med MoveJ"""
        joint_values = self.joint_values_for_target(target)
        if joint_values is not None:
            self.send_signal(joint_values)
            print(f"degrees of joints to {target.Name()}: ")
            #print(joint_values)
            self.read_signal()
            self.robot.MoveJ(joint_values)
        else:
            print(f"Kunne ikke hente leddverdier for target '{joint_values}'.")

    def MoveL(self, target):
        """Sender vinklene til alle leddene og flyter på simulasjonen med MoveL.
        Er ikke ferdigstit
        """
        joint_values = self.joint_values_for_target(target)

        if joint_values is not None:
            #joint_speed = self.calculat_joint_speed(target)
            pose = target.Pose()
            #self.send_signal(joint_speed, "S")
            print(f"Speed of joints to {target.Name()}: ")
            #self.read_signal()
            sleep(0.2)
            self.send_signal(joint_values)
            print(f"degrees of joints to {target.Name()}: ")
            self.read_signal()
            self.robot.MoveL(pose)
        else:
            print(f"Kunne ikke hente leddverdier for target '{joint_values}'.")

    def robot_speed_control(self, speed):
        """setts a speed for all jonits, the speed has to be between 1 and 100"""
        under_speed_limet = 1
        upper_speed_limet = 101
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
        """Ved oppstart flyter robodk modell til Home
        ved resten av tifellene vil den flytte roboten i RoboDK basert på enkoder værdiene fra roboten"""
        self.send_signal("","H")
        self.read_signal()

    def get_encoder_value(self):
        """Leser av data-en til encoderne"""
        self.send_signal("","E")
        self.read_signal()

    def correcting_robodk(self, encoder_data):
        """"Flytter roboten i RoboDK basert på enkoder værdiene fra roboten"""
        self.robot.setJoints(encoder_data)

    def read_signal(self):
        """Leser signal fra Arduino."""
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
                elif read_serial.startswith("Home:"):
                    read = read_serial[5:].split(",")
                    text_list = []
                    for i in read:
                        text_list.append(float(i))
                    print(f"Robot going to home: {text_list}")
                    self.correcting_robodk(text_list)
                else:
                    print(f"{read_serial}")

    def send_signal(self, info, message_type="R"):
        """Sender signaler til Arduino-en ved hjelp av Serial."""
        if message_type == "R": #Degrees for all the joints
            info = 'R' + str(info) + '\n'

        elif message_type == "S": #Speed for all the joints
            info = 'S' + str(info) + '\n'

        elif message_type == "L": #Robot Speed
            info = 'L' + str(info) + '\n' 

        elif message_type == "H":
            info = 'H' + '\n'

        elif message_type == "G":
            info = 'G' + str(info) + '\n'

        self.ser.write(info.encode('utf-8'))
        sleep(self.write_delay)

    def close(self):
        """Lukker tilkoblingen til Arduino."""
        if self.ser.is_open:
            self.ser.close()
    
    def setDO(self, io_bool):
        """f"""
        self.send_signal(io_bool, "G")
        self.read_signal()
