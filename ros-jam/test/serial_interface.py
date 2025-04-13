import serial
import json
import time

class MCUInterface:

    #Adjust the port accordingly
    def __init__(self, port='/dev/ttyACM0', baudrate=9600, timeout=2):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Wait for MCU to boot
    

    # Sends and reads the response
    def send_command(self, command: dict) -> dict:
        msg = json.dumps(command) + '\n'

        # Encode and send message, then decode and read it
        self.ser.write(msg.encode('utf-8'))
        response = self.ser.readline().decode('utf-8').strip()

        try:
            return json.loads(response)
        
        except json.JSONDecodeError:
            return {'status': 'error', 'msg': 'invalid response'}

    def close(self):
        self.ser.close()



# Example of sending a specific command (cmd here) to the mcu and using send_command, which sends and reads
if __name__ == "__main__":
    mcu = MCUInterface()

    # Example command
    cmd = {"cmd": "led_on", "value": 1}
    print("Sending:", cmd)
    response = mcu.send_command(cmd)
    print("Response:", response)

    mcu.close()
