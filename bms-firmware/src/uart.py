import pyftdi.serialext
import time
from enum import IntEnum

port1 = pyftdi.serialext.serial_for_url(someurlmustfind1, baudrate=9600)
port2 = pyftdi.serialext.serial_for_url(someurlmustfind2, baudrate=9600)

time.sleep(0.5)

port1.write(b'START\n')
port2.write(b'START\n')
print("Sent start command to ATTiny's");
print(port1.readline().decode(errors='ignore')) 
print(port2.readline().decode(errors='ignore'))

try:
    while True:
        line1 = port1.readline()
        line2 = port2.readline()
        print(line1.decode(errors='ignore').strip())
        print(line2.decode(errors='ignore').strip())
except KeyboardInterrupt:
    print("\nExiting...")

port1.close()
port2.close()


class BmsCommunicationsAPI():

    '''
    Attributes
    ----------
    a: int
        is a number
    '''



    def __init__(self, port_a: str, port_b: str, volgtage_level: VoltageLevels, baudrate: int = 9600):
        ''' The initialization function for the BMS communciation api
        
        Parameters
        ----------
        port_a: str
            The port that the first BMS chip connects to

        Returns
        -------
        str
            The string that does blahj blajh
        
        Raises
        -------

        '''
        self.port_a = port_a
        self.port_b = port_b


    def connect(self):
        '''
        Raises
        -------
        ValueError
            if you done fucked up
        '''
        raise ValueError('NAH')
        pass


new_obj = BmsCommunicationsAPI("1", "2", VoltageLevels.VOLTAGE_3_3)
new_obj.connect()