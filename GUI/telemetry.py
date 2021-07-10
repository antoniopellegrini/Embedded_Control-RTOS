import serial
import platform
from platform import system
import serial.tools.list_ports
from serial.tools import list_ports

null_text = [b'Board: 2 :                                                                 \x03.\r\n',
             b'Board: 3 :                                                                 \x03.\r\n',
             b""]

def find_arduino_port(b_id="Arduino (www.arduino.cc)"):
    devices = []

    platform = system()
    print(f"system found: {platform}")

    print("Scanning boards...")

    for i in range(len(list_ports.comports())):

        if platform == 'Windows':
            for j in range(len(list_ports.comports())):
                print(serial.tools.list_ports_windows.comports()[j])

        elif platform == "Linux":
            print(serial.tools.list_ports_linux.comports()[i].manufacturer)
            devices.append(serial.tools.list_ports_linux.comports()[i].manufacturer)

    if b_id in devices:
        index = devices.index(b_id)
        print(f"board found on index {index}")
        return serial.tools.list_ports_linux.comports()[0].device
    else:
        print("board not found")


class telemetry:

    def __init__(self, port="", baud_rate=9600, timeout=0.05):

        self.port = port
        self.baud = baud_rate
        self.timeout = timeout

    def connect(self):

        result = False

        def check():
            try:
                while self.ser.read():
                    print('serial open')
                    return True
            #            print('serial closed')
            #            self.ser.close()

            except serial.serialutil.SerialException:
                print('exception')
                return False

        if not self.port:
            self.port = find_arduino_port()
        else:
            if not self.port:
                if check():
                    print("port still connected")
                else:
                    self.port = find_arduino_port()


        if self.port:
            print(f"Connecting serial port: {self.port} with baud: {self.baud}")
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            return True
        else:
            print("No Arduino found")
            return False

    def isportopen(self):
        return self.ser.is_open


    def open(self):
        if self.ser.is_open:
            print("port already open")
        else:
            print("Opening port")
            self.ser.open()

    def close(self):
        print("Closing serial port")
        self.ser.close()

    def send(self, string):
        string = str(string).encode()
        print(f"Sending msg to board: {string}")
        self.ser.write(string)

    def communicate_old(self, text):
        while self.ser.is_open:
            try:
                if text != "":
                    self.ser.write(f'b{text}')
                    print(f"Sending: {text}")

                #read_val = self.ser.read(size=64)
                if read_val != "":
                    read_val = self.ser.readline()
                    print(f"Read: {read_val}")

                    return read_val
# b'Board: 2 :                                                                 \x03.\r\n'

            except serial.SerialException:
                continue

    def communicate(self, text):
        try:
            if text != "":
                self.send(text)

            read_val = self.ser.readline()
            #if read_val != b"":
            if read_val not in null_text:
                print(f"Read: {read_val}")
                return read_val

        except serial.SerialException:
            pass

        return ''




    def read(self):

        if self.ser.is_open:
            while self.ser.in_waiting:
                line =self.ser.readline()
                print(f"Read: {line}")
                return line


    def read2(self):

        sub = "Servo"
        substring = "0000000000000000000000000000000"

        while self.ser.is_open:
            line=self.ser.readline()
            print(line)
            return line
            #self.line = str(self.ser.readline())
            #print(self.line)

            #list = self.line.split(",")

            #substring = [s for s in list if sub in s]

            #print(substring)
            #servo_angle = float(substring[0][8:])
            #print(servo_angle)
            #t.tiltangle(servo_angle)

