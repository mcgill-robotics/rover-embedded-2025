import serial


class Gripper():
    def __init__(self, port : str, baud_rate : int = 115200):
        self.port : str = port
        self.baud_rate : int = baud_rate
        self.ser : serial.Serial = None
        self.is_connected : bool = False
        self.buffer : bytes = b"" 
        
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.is_connected = True
        except serial.SerialException as e: 
            raise ConnectionError(f"Failed to connect to arm_brushed board (gripper). Error: {e}")
        
    def _read_serial(self):

        if self.is_connected is False:
            raise ConnectionError("Cannot read frm serial port, not connected to board")
        
        data = self.ser.read(self.ser.in_waiting or 1)

        if data:
            self.buffer += data
            while True:
                if b'\n' in self.buffer:
                    line, self.buffer = self.buffer.split(b'\n', 1)
                    line = line.decode('utf-8').strip()
                    self._parse_data(line)
                else:
                    break

    
    def close_gripper(self):
        if self.is_connected is False:
            raise ConnectionError("Cannot read frm serial port, not connected to board")
        message = ("c").encode()
        self.ser.write(message)


    def open_gripper(self):
        if self.is_connected is False:
            raise ConnectionError("Cannot read frm serial port, not connected to board")
        message = ("o").encode()
        self.ser.write(message)

    def stop_gripper(self):
        if self.is_connected is False:
            raise ConnectionError("Cannot read frm serial port, not connected to board")
        message = ("s").encode()
        self.ser.write(message)
        

    def cw_roll(self):
        if self.is_connected is False:
            raise ConnectionError("Cannot read frm serial port, not connected to board")
        message = ("d").encode()
        self.ser.write(message)

    def ccw_roll(self):
        if self.is_connected is False:
            raise ConnectionError("Cannot read frm serial port, not connected to board")
        message = ("w").encode()
        self.ser.write(message)

    def stop_roll(self):
        if self.is_connected is False:
            raise ConnectionError("Cannot read frm serial port, not connected to board")
        message = ("r").encode()
        self.ser.write(message)


if __name__ == "__main__":
    import time
    board = Gripper("COM8")

    try:
        board.connect()
    except ConnectionError as e:
        print(e)
        exit(1)


    while True:
        command = input("Input command here(o,c,s): ")
        if command == "o":
            board.open_gripper()
        else if command == "c":
            board.close_gripper()
        else if command == "s":
            board.stop_gripper()
        else:
            print("Invalid command")
