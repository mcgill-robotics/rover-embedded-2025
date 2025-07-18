from pyftdi.ftdi import Ftdi

def get_devices():
    ftdi = Ftdi()
    return ftdi.list_devices()
    
if __name__ == "__main__":
    devices = get_devices()
    print("Connected FTDI Devices:")
    for device in devices:
        print(f" - {device}")