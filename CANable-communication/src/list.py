import usb
dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)
print(dev)