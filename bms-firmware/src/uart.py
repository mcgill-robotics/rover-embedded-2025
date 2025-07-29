import pyftdi.serialext
import time

from enum import IntEnum

port = pyftdi.serialext.serial_for_url("ftdi://ftdi:ft-x:0:3/1", baudrate=9600)
time.sleep(0.5)

port.write(b'START\n')
print("Sent start command to ATTiny's");
print(port.readline().decode(errors='ignore')) 

try:
    while True:
        line1 = port.readline()
        print(line1.decode(errors='ignore').strip())
except KeyboardInterrupt:
    print("\nExiting...")

port.close()






# class BmsCommunicationsAPI():

#     '''
#     Attributes
#     ----------
#     a: int
#         is a number
#     '''



#     def __init__(self, port_a: str, port_b: str, volgtage_level: VoltageLevels, baudrate: int = 9600):
#         ''' The initialization function for the BMS communciation api
        
#         Parameters
#         ----------
#         port_a: str
#             The port that the first BMS chip connects to

#         Returns
#         -------
#         str
#             The string that does blahj blajh
        
#         Raises
#         -------

#         '''
#         self.port_a = port_a
#         self.port_b = port_b


#     def connect(self):
#         '''
#         Raises
#         -------
#         ValueError
#             if you done fucked up
#         '''
#         raise ValueError('NAH')
#         pass


# new_obj = BmsCommunicationsAPI("1", "2", VoltageLevels.VOLTAGE_3_3)
# new_obj.connect()
