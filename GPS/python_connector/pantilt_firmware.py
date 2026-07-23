import serial

DELIMITER = 0x00


def cobs_encode(data: bytes) -> bytes:
    """COBS-encodes data (must not contain the delimiter byte after encoding)."""

    output = bytearray(b'\x00')
    code_pos = 0
    code = 1
    for byte in data:
        if byte == DELIMITER:
            output[code_pos] = code
            code_pos = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_pos] = code
                code_pos = len(output)
                output.append(0)
                code = 1
    output[code_pos] = code
    return bytes(output)


def cobs_decode(frame: bytes) -> bytes:
    """Decodes one COBS frame (without its trailing delimiter byte)."""

    output = bytearray()
    i, n = 0, len(frame)
    while i < n:
        code = frame[i]
        if code == 0 or i + code > n:
            raise ValueError("invalid COBS frame")
        i += 1
        output += frame[i:i + code - 1]
        i += code - 1
        if code < 0xFF and i < n:
            output.append(DELIMITER)
    return bytes(output)


class PanTiltGPS():
    """
    Represents the UART board with GPS and pantilt relay over a single USB CDC port.

    The board exposes one serial port that:
      - Outputs GPS data as: satellites,latitude,longitude,heading
      - Accepts pantilt commands forwarded to the servo board over UART
      - Allows communication with another terminal from the UART board

    Attributes
    ----------
    port: str
        The USB CDC port for both GPS output and pantilt input.
    baud_rate: int
        The baud rate of the connection.
    ser: serial.Serial
        The serial connection to the board.
    is_connected: bool
        Whether the board is connected to the computer.
    buffer: bytes
        Receive buffer for an incomplete (undelimited) COBS frame.
    gps_sats: float
        Number of satellites connected to the GPS.
    coords: list[float]
        The latitude and longitude [lat, lon].
    heading: float
        The heading of motion in degrees.
    pan_angle: float
        The reported pan angle.
    tilt_angle: float
        The reported tilt angle.
    terminal_rx: bytearray
        Raw bytes received from the secondary UART while it's in terminal mode.
    """

    def __init__(self, port: str, baud_rate: int = 115200):
        """
        Parameters
        ----------
        port: str
            The USB CDC port (COM? for Windows, /dev/ttyACM? for Linux).
        baud_rate: int, optional
            The baud rate of the connection. Default is 115200 bps.
        """

        self.port: str = port
        self.baud_rate: int = baud_rate
        self.ser: serial.Serial = None
        self.is_connected: bool = False
        self.buffer: bytes = b""

        # GPS data
        self.gps_sats: float = 0
        self.coords: list[float] = [-1.0, -1.0]
        self.heading: float = 0.0

        # Pantilt data
        self.pan_angle: float = 0.0
        self.tilt_angle: float = 0.0

        # Diagnostic data
        self.gps1_valid_frames: int = 0
        self.gps1_error_frames: int = 0
        self.gps2_valid_frames: int = 0
        self.gps2_error_frames: int = 0

        # Terminal functionality
        self.terminal_rx: bytearray = bytearray()
        self.mode: str = "gps"

    def connect(self):
        """
        Connects to the board. Run this before using this object.

        Raises
        ------
        ConnectionError
            If it fails to connect to the board.
        """

        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.is_connected = True
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to connect to board. Error: {e}")

    def _read_serial_gps(self):
        """Reads GPS data from serial if available."""

        if not self.is_connected:
            raise ConnectionError("Cannot read from serial port, not connected to board.")

        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except serial.SerialException:
            self.is_connected = False
            return
        if not data:
            return
        self.buffer += data
        while bytes([DELIMITER]) in self.buffer:
            frame, self.buffer = self.buffer.split(bytes([DELIMITER]), 1)
            if not frame:
                continue
            try:
                payload = cobs_decode(frame)
            except ValueError:
                continue
            if payload:
                self.parse_frame(payload[0:1], payload[1:])

    def parse_frame(self, msg_type: bytes, payload: bytes):
        """Parses a decoded [type][payload] frame from the board."""

        text = payload.decode('utf-8', errors='replace')

        if msg_type == b"g":
            fields = text.split(',')
            if len(fields) < 4:
                return
            try:
                self.gps_sats = float(fields[0])
                self.coords[0] = float(fields[1])
                self.coords[1] = float(fields[2])
                self.heading = float(fields[3])
            except ValueError:
                pass

        elif msg_type == b"p":
            fields = text.split(',')
            if len(fields) < 2:
                return
            try:
                self.pan_angle = float(fields[0])
                self.tilt_angle = float(fields[1])
            except ValueError:
                pass

        elif msg_type == b"t":
            self.terminal_rx += payload

        elif msg_type == b"d":
            fields = text.split(',')
            if len(fields) < 5:
                return
            try:
                self.gps1_valid_frames = int(fields[0])
                self.gps1_error_frames = int(fields[1])
                self.gps2_valid_frames = int(fields[2])
                self.gps2_error_frames = int(fields[3])
            except ValueError:
                pass
            self.mode = "term" if fields[4] == "t" else "gps"

    def send_frame(self, msg_type: bytes, payload: bytes):
        """Encodes and writes a [type][payload] COBS frame to the board."""

        try:
            self.ser.write(cobs_encode(msg_type + payload) + bytes([DELIMITER]))
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to write to board. Error: {e}")

    def run(self):
        """Runs the object's main loop. Call this function in your main loop."""

        self._read_serial_gps()

    def is_gps_connected(self) -> bool:
        """Returns whether the GPS has at least 3 satellite connections."""

        return self.gps_sats >= 3

    def get_gps_satellites(self) -> float:
        """Gets the number of satellites connected to the GPS."""

        return self.gps_sats

    def get_gps(self) -> list[float]:
        """Gets the last available GPS coordinates as [satellites, latitude, longitude, heading]."""

        return [float(self.gps_sats), self.coords[0], self.coords[1], self.heading]
    
    def get_pantilt(self) -> list[float]:
        """Gets the last available pantilt angles as [pan angle, tilt angle]."""
        
        return [self.pan_angle, self.tilt_angle]

    def add_pan_angle(self, angle: float):
        """
        Adds an increment of angle to the pan servo.

        Parameters
        ----------
        angle : float
            The increment to add to the pan servo angle.

        Raises
        ------
        ConnectionError
            If there is no connection.
        """

        if not self.is_connected:
            raise ConnectionError("Cannot write to serial port, not connected to board.")
        self.send_frame(b"p", f"{angle},0.0".encode())

    def add_tilt_angle(self, angle: float):
        """
        Adds an increment of angle to the tilt servo.

        Parameters
        ----------
        angle : float
            The increment to add to the tilt servo angle.

        Raises
        ------
        ConnectionError
            If there is no connection.
        """

        if not self.is_connected:
            raise ConnectionError("Cannot write to serial port, not connected to board.")
        self.send_frame(b"p", f"0.0,{angle}".encode())

    def set_mode(self, mode: str):
        """
        Switches what the second UART port (independent of pantilt) is used for.

        Parameters
        ----------
        mode : str
            One of "gps", "term". Boot default is "gps".

        Raises
        ------
        ConnectionError
            If there is no connection.
        ValueError
            If mode isn't one of "gps", "term".
        """

        if mode not in ("gps", "term"):
            raise ValueError(f"Invalid mode: {mode!r}, must be 'gps' or 'term'.")
        if not self.is_connected:
            raise ConnectionError("Cannot write to serial port, not connected to board.")
        self.send_frame(b"m", mode.encode())

    def write_terminal(self, data: bytes):
        """
        Sends raw bytes to the secondary UART. Only takes effect while
        that port is in terminal mode (see `set_mode`).

        Parameters
        ----------
        data : bytes
            Raw bytes to forward to the terminal device. Max 255 bytes per
            call (the board's frame buffer is 256 bytes, including the type
            byte). Split larger payloads into multiple calls.

        Raises
        ------
        ConnectionError
            If there is no connection.
        ValueError
            If `data` is longer than 255 bytes.
        """

        if len(data) > 255:
            raise ValueError(f"Terminal frame too large ({len(data)} bytes, max 255); split into multiple calls.")
        if not self.is_connected:
            raise ConnectionError("Cannot write to serial port, not connected to board.")
        self.send_frame(b"t", data)

    def read_terminal(self) -> bytes:
        """Returns and clears any raw bytes received from the terminal device since the last call."""

        data = bytes(self.terminal_rx)
        self.terminal_rx.clear()
        return data
    
    def get_gps_diag(self):
        """Returns the number of valid and error frames for each of the two GPS."""

        return [[self.gps1_valid_frames, self.gps1_error_frames], [self.gps2_valid_frames, self.gps2_error_frames]]

    def get_mode(self) -> str:
        "Returns the current mode of the GPS 2 UART. Can either be `gps` or `term`."

        return self.mode

if __name__ == "__main__":
    import time
    board = PanTiltGPS("/dev/ttyACM0")
    try:
        board.connect()
    except ConnectionError as e:
        print(e)
        exit(1)

    while True:
        board.run()
        print(f"GPS lock: {board.is_gps_connected()}, GPS: {board.get_gps()}")
        print(f"Pantilt: {board.get_pantilt()}")
        print(f"Diagnostic: {board.get_diagnostic_data()}")
        time.sleep(0.05)
