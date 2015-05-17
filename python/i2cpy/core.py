''' Core routines for the Py2CStick.
'''
import struct
import usb.core
import constant

class Py2CStick(object):
    def __init__(self, deviceAddress = None, delay = 10):
        ''' Initializer
        '''
        assert deviceAddress != None, 'Must set device i2c address.'
        
        self.deviceAddress = deviceAddress
        self.delay = delay
        
        self.device = self.OpenUsbDevice()
        self.SetProperty(constant.EProperty().DELAY, delay)
        
    def OpenUsbDevice(self):
        ''' Find and open the USB device.
        '''
        # Find the first device with the proper PID/VID and product name.
        usbDevice = None
        deviceList = list(usb.core.find(find_all=True, \
            idProduct = constant.I2C_STICK_PID, idVendor = constant.I2C_STICK_VID))
            
        for device in deviceList:
            if device._product is None:
                device._product = usb.util.get_string(device, device.iProduct)
            if device.product == "i2c stick":
                usbDevice = device
                break;

        if usbDevice == None:
            raise ValueError('Cannot find device.')
        
        # Select the desired configuration and interface.
        usbDevice.set_configuration(1)
        usb.util.claim_interface(usbDevice, 0)
        return usbDevice
        
    def Echo(self, value):
        ''' Echo a value back from the USB device.
        '''
        assert value >= 0 and value <= 65535, 'Value must be between 0 and 65535'
        
        echoedValue = self.device.ctrl_transfer(constant.USB_CTRL_IN, constant.CMD_ECHO, value, 0, 2)
        return 256 * echoedValue[1] + echoedValue[0]


    # I2C commands.

    def ScanAddress(self, address):
        ''' Ping an address on the I2C bus.
            Return True if address is valid, False otherwise.
        '''
        responseData = self.device.ctrl_transfer( \
            constant.USB_CTRL_IN, constant.CMD_I2C_PROBE, constant.I2C_M_RD, address, 0)
        return self.GetStatus() == constant.STATUS_ADDRESS_ACK
        
    def ScanBus(self, startAddress, endAddress):
        ''' Scan a series of addresses on the I2C bus.
        '''
        assert endAddress <= 127, 'End address must be <= 127 (0x7F).'
        assert startAddress >= 0, 'Start address must be >= 0 (0x00).'
        assert startAddress <= endAddress, 'Start address must be <= end address.'
        
        validAddresses = list()
        for address in range(startAddress, endAddress + 1):
            if self.ScanAddress(address):
                validAddresses.append(address)
        return validAddresses
        
    def WriteByte(self, data):
        ''' Write a single byte to the I2C device.
        '''
        assert self.deviceAddress != None, 'Device address not set.'
        
        writeBuffer = struct.pack('B', data)
        lengthTransferred = self.device.ctrl_transfer( \
            constant.USB_CTRL_OUT, \
            constant.CMD_I2C_BEGIN + constant.CMD_I2C_IO + constant.CMD_I2C_END, \
            constant.I2C_M_WR, self.deviceAddress, writeBuffer)
        assert self.GetStatus() == constant.STATUS_ADDRESS_ACK
        return lengthTransferred

    def WriteRegister(self, registerNumber, registerValue, registerSize = 1):
        ''' Write a 1 or 2 byte value to the i2c device
            Default is to write 1 byte.
        '''
        assert self.deviceAddress != None, 'Device address not set.'
        assert registerSize == 1 or registerSize == 2, 'Register size must be 1 or 2.'
        
        if registerSize == 1:
            writeBuffer = struct.pack('!BB', registerNumber, registerValue)
        else:
            writeBuffer = struct.pack('!BH', registerNumber, registerValue)
              
        lengthTransferred = self.device.ctrl_transfer( \
            constant.USB_CTRL_OUT, \
            constant.CMD_I2C_BEGIN + constant.CMD_I2C_IO + constant.CMD_I2C_END, \
            constant.I2C_M_WR, self.deviceAddress, writeBuffer)
        assert self.GetStatus() == constant.STATUS_ADDRESS_ACK
        return lengthTransferred
            
    def ReadRegister(self, registerNumber, registerSize = 0):
        ''' Send command read a 0, 1, or 2 byte value from the interface.
            Default is to read 0 byte.
        '''            
        assert self.deviceAddress != None, 'Device address not set.'
        assert registerSize >= 0 and registerSize <= 2, 'Register size must be 1 or 2.'
        
        # Write the register address to the device.
        if  registerSize == 0:
            request = constant.CMD_I2C_BEGIN + constant.CMD_I2C_IO + constant.CMD_I2C_END
        else:
            request = constant.CMD_I2C_BEGIN + constant.CMD_I2C_IO
        
        writeBuffer = struct.pack('B', registerNumber)
        lengthTransferred = self.device.ctrl_transfer( \
            constant.USB_CTRL_OUT, request, \
            constant.I2C_M_WR, self.deviceAddress, writeBuffer)       
        assert self.GetStatus() == constant.STATUS_ADDRESS_ACK, \
            'Write command status failed.'
        
        # Read the response.
        registerValue = 0
        if  registerSize > 0:
            returnBuffer = self.device.ctrl_transfer( \
                constant.USB_CTRL_IN, \
                constant.CMD_I2C_IO + constant.CMD_I2C_END, \
                constant.I2C_M_RD, self.deviceAddress, registerSize)
            assert self.GetStatus() == constant.STATUS_ADDRESS_ACK, \
                'Write command status failed.'

            if len(returnBuffer) == 2:
                registerValue = 256 * returnBuffer[0] + returnBuffer[1]
            else:
                registerValue = returnBuffer[0]
                
        # Return register value.
        return registerValue
        
                                
    # Interface command routines.
    
    def SetProperty(self, propertyEnum, value):
        ''' Set a property in the I2C USB interface.
            propertyEnum determines the property being set.
            value is the property value.
            This does not do anything on the I2C bus.  It is strictly a command
            issued to the interface.
        '''
        # Set the command number.
        command = 0xFF
        if propertyEnum == constant.EProperty().DELAY:
            command = constant.CMD_SET_DELAY
        else:
            raise ValueError('Unrecognized property.')
            
        # Perform the transfer
        lengthTransferred = self.device.ctrl_transfer( \
            constant.USB_TYPE_VENDOR, command, value)
        return lengthTransferred

    def GetStatus(self):
        ''' Get the current transaction status from the interface.
            This does not do anything on the I2C bus.  It is strictly a command
            issued to the interface.
        '''
        # The Read routine returns an array so use indexing to return the 
        # value at the [0] index.
        return self.Read(constant.CMD_GET_STATUS, 1)[0]
                
    def Read(self, cmd, responseLength):
        ''' Issue a command to the i2c_tiny_usb interface and read the response.
            This does not do anything on the I2C bus.  It is strictly a command
            issued to the interface.
            
            cmd is the command being issued to the interface
            responseLength is the length of the returned data.
            The returned data is an array of bytes ('B')
        '''
        responseData = self.device.ctrl_transfer( \
            constant.USB_CTRL_IN, cmd, 0, 0, responseLength)
        return responseData
           
    def Write(request, value, index):
        ''' Write a set of bytes to the i2c_tiny_usb device via a control transfer.
            This does not do anything on the I2C bus.  It is strictly a command
            issued to the interface.
            
            request is the command being issued to the interface.
            value is the value to be written to the interface.
            index is the interface register to which the value is written.
            Returns the number of bytes transferred.
        '''
        transferLength = self.device.ctrl_transfer( \
            constant.USB_CTRL_OUT, request, value, index, 0)
        return transferLength
             
        
