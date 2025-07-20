





import enum
from typing import Optional, Union
import serial


class CommandID(enum.IntEnum):
    START = 1
    STOP = 2
    STATUS = 3

class BaseStation:
    def __init__(self, port, baudrate=9600):
        self.serial_port = serial.Serial(port, baudrate)

    def send_message(self, message: bytes):
        self.serial_port.write(message)

    def receive_message(self) -> Optional[bytes]:
        if self.serial_port.in_waiting > 0:
            return self.serial_port.read(self.serial_port.in_waiting)
        return None
    
    def create_data(self, command_id: CommandID, data: Optional[list[Union[float, int, str]]] = None) -> bytes:
        """Create a data packet with a start byte, command ID, and optional data."""
        
        packet = bytearray()
        packet.append(0xAA)
        packet.append(command_id.value)
        packet.append(len(data) if data else 0)
        if data is not None:
            for item in data:
                if isinstance(item, float):
                    packet.extend(item.to_bytes(4, 'little', signed=True))
                elif isinstance(item, int):
                    packet.extend(item.to_bytes(4, 'little', signed=True))
                elif isinstance(item, str):
                    packet.extend(item.encode('utf-8'))
                else:
                    raise ValueError("Unsupported data type")
