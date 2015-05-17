''' Py2CStick Constants
'''

# Vendor vid/pid supplied by Objective Development.
I2C_STICK_VID = 0x16c0
I2C_STICK_PID = 0x05dc

# I2C read flag.
I2C_M_WR = 0x00
I2C_M_RD = 0x01

# USB interface commands -- must match command ids in firmware.
CMD_ECHO = 8
CMD_GET_FUNC = 9
CMD_SET_DELAY = 10
CMD_GET_STATUS = 11

# CMD_I2C_BEGIN - flag to I2C_IO -- indicates the transaction contains 
#                 an I2C start.
# CMD_I2C_END   - flag to I2C_IO -- indicates the transaction contains
#                 an I2C end.
# CMD_I2C_IO    - flag indicating IO transaction.
# CMD_I2C_PROBE - flag indicating only being/end of transaction --
#                 used to probe for an address.
CMD_I2C_BEGIN = 1
CMD_I2C_END = 2
CMD_I2C_IO = 4
CMD_I2C_PROBE = CMD_I2C_BEGIN | CMD_I2C_END

# USB status flags.
STATUS_IDLE = 0
STATUS_ADDRESS_ACK = 1
STATUS_ADDRESS_NAK = 2

# USB contants used to build the request type for a control transfer.
USB_ENDPOINT_OUT = (0 << 7)
USB_ENDPOINT_IN = (1 << 7)

USB_TYPE_STANDARD = (0 << 5)
USB_TYPE_CLASS = (1 << 5)
USB_TYPE_VENDOR = (2 << 5)
USB_TYPE_RESERVED = (3 << 5)

USB_RECIPIENT_DEVICE = 0
USB_RECIPIENT_INTERFACE = 1
USB_RECIPIENT_ENDPOINT = 2
USB_RECIPIENT_OTHER = 3

# Constants for a class transfer.
USB_CTRL_OUT = USB_ENDPOINT_OUT | USB_TYPE_CLASS | USB_RECIPIENT_DEVICE
USB_CTRL_IN  = USB_ENDPOINT_IN  | USB_TYPE_CLASS | USB_RECIPIENT_DEVICE

# Property enumeration
class EProperty(object):
    def __init__(self):
        self.NONE = 0
        self.DELAY = 1

# String index enumeration
class EStringIndex(object):
    def __init__(self):
        self.VENDOR = 1
        self.DEVICE_NAME = 2

