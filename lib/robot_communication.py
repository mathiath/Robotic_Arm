import serial
from time import sleep
# from robodk import rotx, roty, rotz, transl, Mat, Pose_2_TxyzRxyz
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT  # , ITEM_TYPE_TARGET


# RDK = Robolink()
# robot = RDK.Item('HBot', ITEM_TYPE_ROBOT)

class RobotSerial:
    """Klasse for å kommunisere med Arduino via serieport
     Når et objekt av denne klassen opprettes, prot-enn er ulike basert på operativsystemet:
        - Windows: 'COM3' eller tilsvarende
        - Linux: '/dev/ttyACM0' eller tilsvarende
        - MacOS: '/dev/tty.usbmodemXXXX' eller tilsvarende"""

    def __init__(self, port, baudrate=9600, read_delay=0.1, write_delay=0.1):
        self.ser = serial.Serial(port, baudrate)
        self.RDK = Robolink()
        self.robot = self.RDK.Item('', ITEM_TYPE_ROBOT)
        sleep(read_delay)  # Venter på at serial skal initialisere
        self.write_delay = write_delay
        sleep(1.5)  # Satrtup delay
        self.get_encoder_value()

    def joint_values_for_target(self, target_name):
        """Henter ut alle vinklene til alle leddene og leger dem i en liste"""
        try:
            j = target_name.Joints().list()
            for idx, i in enumerate(j):
                round_value = round(i, 3)
                j[idx] = round_value

            if j and len(j) > 0:
                return j

        except Exception as e:
            print(f"{e}")

    def MoveJ(self, target_name):
        """Sender vinklene til alle leddene og flyter på simulasjonen med MoveJ"""
        joint_values = self.joint_values_for_target(target_name)
        if joint_values is not None:
            self.send_signal(joint_values)
            self.read_signal()
            self.robot.MoveJ(joint_values)
        else:
            print(f"Kunne ikke hente leddverdier for target '{joint_values}'.")

    def get_encoder_value(self):
        """Leser av data-en til encoderne"""
        self.send_signal("", "E")
        self.read_signal()

    def encoders_correcting_robodk(self, encoder_data):
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
                    self.encoders_correcting_robodk(text_list)
                else:
                    print(f"{read_serial}")

    def send_signal(self, info, type="R"):
        """Sender signaler til Arduino-en ved hjelp av Serial."""
        if type == "R":  # Degrees for all the joints
            info = 'R' + str(info) + '\n'
        elif type == "S":  # Speed for all the joints
            info = 'S' + str(info) + '\n'
        elif type == "E":
            info = 'E' + '\n'
        elif type == "T":
            info = 'T' + str(info) + '\n'

        self.ser.write(info.encode('utf-8'))
        sleep(self.write_delay)

    def close(self):
        """Lukker tilkoblingen til Arduino."""
        if self.ser.is_open:
            self.ser.close()
