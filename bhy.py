from time import sleep_ms
from machine import I2C, Pin
import binascii
import struct
import gc

class BHY:
    board_version = 0
    com_buffer = bytes(9482)
    # Data Type and Dimesion definitions
    BHY_DT_PADDING = {'id': 0, 'name': 'Padding', 'fifo_size': 1}
    BHY_DT_QUATERNION = {'id': 1, 'name': 'Quaternion+', 'fifo_size': 11}
    BHY_DT_VECTOR_PLUS = {'id': 2, 'name': 'Vector+', 'fifo_size': 8}
    BHY_DT_VECTOR_ORIENTATION = {'id': 2, 'name': 'Vector+ Orientation', 'fifo_size': 8}
    BHY_DT_VECTOR_UNCALIB = {'id': 3, 'name': 'Vector Uncalibrated', 'fifo_size': 14}
    BHY_DT_SCALAR = {'id': 4, 'name': 'Scalar Data', 'fifo_size': 3}
    BHY_DT_SCALAR_BARO = {'id': 5, 'name': 'Scalar Data (Barometer)', 'fifo_size': 4}
    BHY_DT_SCALAR_HEART = {'id': 6, 'name': 'Scalar Data (Heart Rate)', 'fifo_size': 2}
    BHY_DT_EVENT_DATA = {'id': 7, 'name': 'Event Data', 'fifo_size': 2}
    BHY_DT_ACTIVITY_DATA = {'id': 8, 'name': 'Activity Data', 'fifo_size': 3}
    BHY_DT_DEBUG = {'id': 9, 'name': 'Debug', 'fifo_size': 14}
    BHY_DT_BSX = {'id': 10, 'name': 'BSX Raw Data', 'fifo_size': 17}
    BHY_DT_TIME = {'id': 11, 'name': 'Timestamp', 'fifo_size': 3}
    BHY_DT_META = {'id': 12, 'name': 'Meta Event', 'fifo_size': 4}

    # VIRTUAL SENSOR DEFINITION
    VS_TYPE_ACCELEROMETER = 1
    VS_TYPE_GEOMAGNETIC_FIELD = 2
    VS_TYPE_ORIENTATION  = 3
    VS_TYPE_GYROSCOPE = 4
    VS_TYPE_LIGHT = 5
    VS_TYPE_PRESSURE = 6
    VS_TYPE_TEMPERATURE = 7
    VS_TYPE_PROXIMITY = 8
    VS_TYPE_GRAVITY = 9
    VS_TYPE_LINEAR_ACCELERATION = 10
    VS_TYPE_ROTATION_VECTOR = 11
    VS_TYPE_RELATIVE_HUMIDITY = 12
    VS_TYPE_AMBIENT_TEMPERATURE = 13
    VS_TYPE_MAGNETIC_FIELD_UNCALIBRATED = 14
    VS_TYPE_GAME_ROTATION_VECTOR = 15
    VS_TYPE_GYROSCOPE_UNCALIBRATED = 16
    VS_TYPE_SIGNIFICANT_MOTION = 17
    VS_TYPE_STEP_DETECTOR = 18
    VS_TYPE_STEP_COUNTER  = 19
    VS_TYPE_GEOMAGNETIC_ROTATION_VECTOR = 20
    VS_TYPE_HEART_RATE = 21
    VS_TYPE_TILT = 22
    VS_TYPE_WAKEUP = 23
    VS_TYPE_GLANCE  = 24
    VS_TYPE_PICKUP = 25
    # VS_TYPE_ACTIVITY_RECOGNITION = 26 # ???
    VS_TYPE_ACTIVITY_RECOGNITION = 31

    # Event Definitions
    EV_PADDING = 0
    EV_DEBUG = 245
    EV_BSX_C = 249
    EV_BSX_B = 250
    EV_BSX_A = 251

    EV_WAKEUP_TIMESTAMP_LSW = 246
    EV_WAKEUP_TIMESTAMP_MSW = 247
    EV_WAKEUP_META_EVENTS = 248

    EV_TIMESTAMP_LSW = 252
    EV_TIMESTAMP_MSW = 253
    EV_META_EVENTS = 254

    EV_NOT_IMPLEMENTED = 255

    # Registers definitions
    BHY_REG_Buffer_Out = 0x00 # until 0x31
    BHY_REG_FIFO_Flush = 0x32
    BHY_REG_Chip_Control = 0x34
    BHY_REG_Int_Status = 0x36
    BHY_REG_Chip_Status = 0x37
    BHY_REG_Bytes_Remaining_LSB = 0x38
    BHY_REG_Bytes_Remaining_MSB = 0x39
    BHY_REG_PARAMETER_READ_BUFFER_ZERO = 0x3B
    BHY_REG_PARAMETER_PAGE_SELECT_ADDR = 0x54
    BHY_REG_Host_Interface_Control = 0x55
    BHY_REG_PARAMETER_WRITE_BUFFER_ZERO = 0x5C
    BHY_REG_PARAMETER_REQUEST_ADDR = 0x64
    BHY_REG_ROM_Version = 0x70
    BHY_REG_RAM_Version = 0x72
    BHY_REG_Product_ID = 0x90
    BHY_REG_Revision_ID = 0x91
    BHY_REG_Upload_Adress = 0x94
    BHY_REG_Upload_Data = 0x96
    BHY_REG_Upload_CRC = 0x97
    BHY_REG_Reset_Request = 0x9B

    # GLOBAL DEFINITIONS
    BHY_SYSTEM_PAGE = 1
    BHY_ALGORITHM_PAGE = 2
    BHY_SENSORS_PAGE = 3
    BHY_MAX_SENSOR_ID = 32
    BHY_SID_WAKEUP_OFFSET = 32
    BHY_REG_PARAMETER_ACKNOWLEDGE_ADDR = 0x3A
    BHY_SENSOR_PARAMETER_WRITE = 0xC0
    BHY_SENSOR_CONFIGURATION_SIZE = 8
    BHY_PARAMETER_FIFO_CONTROL = 2

    BHY_PHYSICAL_SENSOR_PRESENT_PARAMETER = 31
    BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_DETAIL_0 = 32

    BHY_ALGORITHM_STANDBY_REQUEST = (1<<0).to_bytes(1,'big')
    BHY_ABORT_TRANSFER = (1<<1).to_bytes(1,'big')
    BHY_UPDATE_TRANSFER_COUNT = (1<<2).to_bytes(1,'big')
    BHY_WAKEUP_FIFO_HOST_INTERRUPT_DISABLE = (1<<3).to_bytes(1,'big')
    BHY_NED_COORDINATES = (1<<4).to_bytes(1,'big')
    BHY_AP_SUSPENDED = (1<<5).to_bytes(1,'big')
    BHY_REQUEST_SENSOR_SELF_TEST = (1<<6).to_bytes(1,'big')
    BHY_NON_WAKEUP_FIFO_HOST_INTERRUPT_DISABLE = (1<<7).to_bytes(1,'big')

    BHY_CPU_RUN_REQUEST = (1<<0).to_bytes(1,'big')
    BHY_HOST_UPLOAD_ENABLE = (1<<1).to_bytes(1,'big')
    BHY_FIFO_FLUSH_ALL = b'\xFF'


    def __init__(self, MAIN_I2C, address=0x28, int_pin = 0, stand_alone = False, board_version = 2, debug = False):
        self.i2c = MAIN_I2C

        if int_pin:
            self.int_pin = Pin(int_pin)
            self.int_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.bhy_int_handler)
        else:
            self.int_pin = 0

        self.BHY_ADDR = address
        self.fw_file = "BHY/fw_bin/"
        if stand_alone:
            # Stand-alone BHI160B
            self.fw_file += "bosch_pcb_7183_di03_bmi160-7183_di03-2-1-11824.fw"
        else:
            # Integrated with BMM150
            self.fw_file += "bosch_pcb_7183_di03_bmi160_bmm150-7183_di03-2-1-11824.fw"
        
        self.debug = debug
        self.commBuffer = bytearray()
        self.board_version = board_version
        self.int_status = 0

    def printDebug(self, msg):
        if self.debug:
            print(msg)

    def bhy_interrupt(self):
        if self.int_pin:
            return self.int_pin.value()
        out = int.from_bytes(self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_Int_Status, 1),'big', False) & 1
        return out
    
    def bhy_int_handler(self, pin):
        print("Interrupt! self.int_status is " + str(self.int_status) )
        self.int_status = not self.int_status

    # TODO: maybe implement a systeam to parse the raw data into a more readable matrix
    def getRemappingMatrix(self, sensor_id):
        return self.readParameterPage(self.BHY_SYSTEM_PAGE, self.BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_DETAIL_0 + sensor_id, 16)

    def setRemappingMatrix(self, sensor_id, matrix):
        # Convert the input matrix in nibbles
        buf = bytearray()
        for i in range(5):
            if matrix[i * 2] == 0:
                buf += b'\x00'
            elif matrix[i * 2] == 1:
                buf += b'\x01'
            elif matrix[i * 2] == -1:
                buf += b'\x0F'
            else:
                return False

            if (i == 4):
                break
            if matrix[i * 2 + 1] == 0:
                pass
            elif matrix[i * 2 + 1] == 1:
                buf[i]  = buf[i] | int.from_bytes(b'\x10', 'big')
            elif matrix[i * 2 + 1] == -1:
                buf[i]  = buf[i] | int.from_bytes(b'\xF0', 'big')
            else:
                return False

        # Actually write the configuration
        return self.writeParameterPage(self.BHY_SYSTEM_PAGE, self.BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_DETAIL_0 + sensor_id, buf)

    def dump_Chip_status(self):
        out = ""
        out += "\n\n---Reading chip status---\n"
        rom_version = self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_ROM_Version, 2)
        out += "\nROM version is: " + str(binascii.hexlify(rom_version))
        product_id = self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_Product_ID, 1)
        out += "\nProduct ID is: " + str(binascii.hexlify(product_id))
        revision_id = self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_Revision_ID, 1)
        out += "\nRevision ID is: " + str(binascii.hexlify(revision_id))

        chip_status = int.from_bytes(self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_Chip_Status, 1),'big', False)

        if(chip_status & 1):
            out += "\n\t-EEPROM Detected!"
        if(chip_status & 2):
            out += "\n\t-EEUploadDone!"
        if(chip_status & 4):
            out += "\n\t-EEUploadError!"
        if(chip_status & 8):
            out += "\n\t-Firmware Idle (halted)!"
        if(chip_status & 16):
            out += "\n\t-No EEPROM!"

        ram_version = self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_RAM_Version, 4)
        out += "\nRam version is: " + str(binascii.hexlify(ram_version))

        self.BHY_crc = self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_Upload_CRC, 4)
        out += "\nBHI CRC:" + str(binascii.hexlify(self.BHY_crc))
        out += "\n------------------------\n\n"

        return out
   
    # Perdoname madre por mi vida loca
    def swap(self, x):
        r = bytearray()
        r.append(x[3])
        r.append(x[2])
        r.append(x[1])
        r.append(x[0])
        return r

    def upload_BHI160B_RAM(self):
        count = 0
        my_crc = 0
        
        self.printDebug("Chip control BEFORE RESET is " + str(int.from_bytes(self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_Chip_Control, 1),'big', False))) # OH GOD BYTES

        # Request BHI to reset
        self.printDebug("Resetting the BHI")
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_Reset_Request, b'\x01')
   
        sleep_ms(100) # TODO: we should wait for the interrupt signal, and not just wait a random amount of ms
        
        # Set the BHI to UploadMode, writing 1 to the HOST_UPLOAD_ENABLE of the Chip Control Register
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_Chip_Control, self.BHY_HOST_UPLOAD_ENABLE)
        self.printDebug("Chip control AFTER SETTING/RESETING is " + str(int.from_bytes(self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_Chip_Control, 1),'big', False))) # OH GOD BYTES

        # Setting Upload_address point at 0x0
        self.printDebug("Resetting data writing position")
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_Upload_Adress, b'\x00')
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_Upload_Adress+1, b'\x00')

        f = open(self.fw_file, 'rb')

        header_1 = f.read(4) # Something
        fw_crc = f.read(4)   # Stored CRC within the source fw
        header_2 = f.read(4) # Something
        header_3 = f.read(4) # Something

        self.printDebug("Uploading RAM patch")
        
        # Loop for read and upload 4-byte at the time, swapping them
        while True:
            data = f.read(4)
            if not data:
                break

            data = self.swap(data) # TODO: should i use bytes() to convert data?
            my_crc = binascii.crc32(data, my_crc)

            self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_Upload_Data, data) # OH GOD BYTES
            #sleep_ms(1)
            count += 4

        self.printDebug("Written " + str(count) + str(" bytes to BHI160B!"))

        # Wait a few millisecond - TODO: WE SHOULD USE THE INTERRUPT ONCE WE HAVE IT
        sleep_ms(100)
        self.BHY_crc = self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_Upload_CRC, 4)
        
        self.printDebug("Stored CRC:" + str(binascii.hexlify(fw_crc)))
        self.printDebug("BHI CRC:" + str(binascii.hexlify(self.BHY_crc)))
        self.printDebug("Calculated CRC:" + str(hex(my_crc))) # TODO: find a way to calculate the corret CRC
        if self.BHY_crc == fw_crc:
            self.printDebug("\tUpload CRC match stored CRC!")
            return True

        return False
        # TODO: find a way to calculate the corret CRC
        # if(self.BHY_crc != my_crc) :
        #     dump_Chip_status()
        #     self.printDebug("UPLOAD FAILED! CRC MISMATCH!!! ABORTING!")
        
    def parse_Int_Status(self):
        out = ""
        int_stat = int.from_bytes(self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_Int_Status, 1),'big', False)

        if(int_stat & 1):
            out += "\t-Host Interrupt!"
        if(int_stat & 2):
            out += "\t-Wakeup Watermark!"
        if(int_stat & 4):
            out += "\t-Wakeup Latency!"
        if(int_stat & 8):
            out += "\t-Wakeup Immediate!"
        if(int_stat & 16):
            out += "\t-Non-Wakeup Watermark!"
        if(int_stat & 32):
            out += "\t-Non-Wakeup Latency!"
        if(int_stat & 64):
            out += "\t-Non-Wakeup Immediate!"

        return out

    def flushFifo(self):
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_FIFO_Flush, self.BHY_FIFO_FLUSH_ALL)

    def pageSelect(self, page, parameter, rw = False):

        # First we select the page
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_PARAMETER_PAGE_SELECT_ADDR, page.to_bytes(1, 'big'))

        mask = 1 << 7
        if not rw: # We want to read
            parameter &= ~mask
        else: # We want to write
            parameter |= mask

        # Then we request the particular address
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_PARAMETER_REQUEST_ADDR, parameter.to_bytes(1, 'big'))

        # Finally we check for ACKNOWLEDGE
        ack = 0
        for i in range(1000): # Polling the register a bunch of times.. maybe less?
            ack = self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_PARAMETER_ACKNOWLEDGE_ADDR, 1)

            if ack == parameter.to_bytes(1, 'big'): # Fuck python bytes
                return True
            elif ack == 0x80: # Why this is important? Bosh uses it in the BHI example ...
                sleep_ms(50)
            else:
                sleep_ms(1)

        return False # Error selecting page

    def readParameterPage(self, page, parameter, length = 8):
        self.printDebug("About to READ" + str(length) + "byte from page:" + str(page) + " - parameter:" + str(parameter))
        # We ask the BHY for a particular parameter in a particular page with READ condition
        if self.pageSelect(page, parameter, rw = False):
            # Ok we have the correct data within the BHY buffer, we must read it!
            ret = self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_PARAMETER_READ_BUFFER_ZERO, length)
        else: # We didn't get the ack
            self.printDebug("Parameter page select FAILED! Page:" + str(page) + "- Parameter:" + str(parameter))
            ret = 0

        # Finally we end the caratteristics transfer procedure by writing 0 to the Parameter_Request register
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_PARAMETER_REQUEST_ADDR, b'\x00')

        return ret
        
    def writeParameterPage(self, page, parameter, buffer):
        self.printDebug("About to write buf:" + str(buffer) + "to page:" + str(page) + " - parameter:" + str(parameter))
        # Write data from the local buffer to the bhy buffer
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_PARAMETER_WRITE_BUFFER_ZERO, buffer)

        # Give the actual write command by selecting the page destination of the buffer
        ret = self.pageSelect(page, parameter, rw=True) # We want to write

        # Finally we end the caratteristics transfer procedure by writing 0 to the Parameter_Request register
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_PARAMETER_REQUEST_ADDR, b'\x00')

        return ret

    def getPhysicalSensorPresent(self):
        return self.readParameterPage(self.BHY_SYSTEM_PAGE, self.BHY_PHYSICAL_SENSOR_STATUS_PARAMETER, 8)

    def configVirtualSensorWithConfig(self, config):
        return self.configVirtualSensor(config["sensor_id"], 
                                        config["wakeup_status"], 
                                        config["sample_rate"], 
                                        config["max_report_latency_ms"], 
                                        config["flush_sensor"], 
                                        config["change_sensitivity"], 
                                        config["dynamic_range"])

    def configVirtualSensor(self,sensorId,wakeup,samplingRate,maxReportLatency,flushSensor,changeSensitivity,dynamicRange):
        
        # Check if Sensor ID is in range
        if sensorId >= self.BHY_MAX_SENSOR_ID:
            return 0

        effectiveId = sensorId
        if wakeup:
            effectiveId += self.BHY_SID_WAKEUP_OFFSET

        # TODO: Implement comlete FLUSH procedure
        # if flushSensor == self.BHY_FIFO_FLUSH_ALL:
        #     self.flushFifo()

        # Computes the param page as sensor_id + 0xC0 (sensor parameter write)
        effectiveId += self.BHY_SENSOR_PARAMETER_WRITE

        # TODO: check if these operations are correct
        # self.commBuffer.append(samplingRate.to_bytes(2, 'little'))
        # self.commBuffer.append(maxReportLatency.to_bytes(2, 'little'))
        # self.commBuffer.append(changeSensitivity.to_bytes(2, 'little'))
        # self.commBuffer.append(dynamicRange.to_bytes(2, 'little'))
        commBuffer = bytearray() # Prepare the 
        commBuffer.append((samplingRate) & 0xFF)
        commBuffer.append((samplingRate >> 8) & 0xFF)
        commBuffer.append((maxReportLatency) & 0xFF)
        commBuffer.append((maxReportLatency >> 8) & 0xFF)
        commBuffer.append((changeSensitivity) & 0xFF)
        commBuffer.append((changeSensitivity >> 8) & 0xFF)
        commBuffer.append((dynamicRange) & 0xFF)
        commBuffer.append((dynamicRange >> 8) & 0xFF)

        return self.writeParameterPage(self.BHY_SENSORS_PAGE, effectiveId, commBuffer)

    def requestSelfTest(self):
        self.printDebug("Requesting a Self-test")
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_Host_Interface_Control, self.BHY_REQUEST_SENSOR_SELF_TEST)

    def startMainTask(self):
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_Chip_Control, self.BHY_CPU_RUN_REQUEST)

    def stopMainTask(self):
        self.i2c.writeto_mem(self.BHY_ADDR, self.BHY_REG_Chip_Control, b'\x00')

    def setFIFOControl(self, wakeup_size = 100, non_wakeup_size = 100):
        data = struct.pack('<4H', wakeup_size, 0, non_wakeup_size, 0) 
        ret = self.writeParameterPage(page=self.BHY_SYSTEM_PAGE, parameter=self.BHY_PARAMETER_FIFO_CONTROL, buffer=data)
        return ret
    
    def getFIFOControl(self):
        data = self.readParameterPage(page=self.BHY_SYSTEM_PAGE, parameter=self.BHY_PARAMETER_FIFO_CONTROL)
        return struct.unpack('<4H', data)

    def readFIFO(self):
        # Read how many bytes we can read from FIFO
        temp = self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_Bytes_Remaining_LSB, 2)
        to_read = int.from_bytes(temp,'little', False)

        # 9482 Obtained by reading fifo controlo parameter (SYSTEM PAGE)
        if to_read > 9482:
            to_read = 9482

        # TODO: find a way to limit how many byte will be read from the FIFO
        # Read the bytes available from FIFO
        self.com_buffer = self.i2c.readfrom_mem(self.BHY_ADDR, self.BHY_REG_Buffer_Out, to_read)
        return self.com_buffer
        
    # TODO: refactor this function to NOT use local buffer but instead read the correct offset of the BHY memory
    def parse_fifo(self, buffer, raw = False):
        out = []
        max_len = 100
        while True:
            if not len(buffer):
                break  

            sensor_id = buffer[0] # The first byte of fifo is the sensor id of the next data

            if sensor_id == self.BHY_DT_PADDING:
                break
           
            data_type = self.get_event_dim(sensor_id) # Get the data dimension for that sensor output 
            if(data_type == -1): # Unkown event
                break

            if raw:
                out.append([sensor_id, data_type, buffer[:data_type['fifo_size']]])
            else: # The user want data cooked for human readability
                out.append([sensor_id, data_type, self.cook_data(data_type, buffer[:data_type['fifo_size']])])

            # Slice buffer to the next event
            buffer = buffer[data_type['fifo_size']:]

        return out

    def cook_data(self, type, buf):
        if type == self.BHY_DT_PADDING:
            pass
        elif type == self.BHY_DT_QUATERNION:
            return self.parseQuaternionPlus(buf)
        elif type == self.BHY_DT_VECTOR_PLUS:
            return self.parseVectorPlus(buf)
        elif type == self.BHY_DT_VECTOR_ORIENTATION:
            return self.parseOrientation(buf)
        elif type == self.BHY_DT_VECTOR_UNCALIB:
            return self.parseVectorUncalib(buf)
        elif type == self.BHY_DT_SCALAR:
            return self.parseScalar(buf)
        elif type == self.BHY_DT_SCALAR_BARO:
            return self.parseScalarBaro(buf)
        elif type == self.BHY_DT_SCALAR_HEART:
            return self.parseScalarHeart(buf)
        elif type == self.BHY_DT_EVENT_DATA:
            return self.parseEventData(buf)
        elif type == self.BHY_DT_ACTIVITY_DATA:
            return self.parseActivityRecognitionData(buf)
        elif type == self.BHY_DT_DEBUG:
            return self.parseDebug(buf)
        elif type == self.BHY_DT_META:
            return self.parseMetaEvent(buf)
        elif type == self.BHY_DT_TIME:
            return self.parseTime(buf)
        else:
            return {"sensor_id": -1, "data": "Unkown data format"}

    # Parse fifo event based on Table 29 from the datasheet
    def get_event_dim(self, id):

        if id == self.EV_PADDING:
            return self.BHY_DT_PADDING

        elif (id == self.VS_TYPE_ROTATION_VECTOR or 
              id == self.VS_TYPE_ROTATION_VECTOR + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_GAME_ROTATION_VECTOR or 
              id == self.VS_TYPE_GAME_ROTATION_VECTOR + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_GEOMAGNETIC_ROTATION_VECTOR or 
              id == self.VS_TYPE_GEOMAGNETIC_ROTATION_VECTOR + self.BHY_SID_WAKEUP_OFFSET
             ):
            return self.BHY_DT_QUATERNION

        elif (id == self.VS_TYPE_ACCELEROMETER or 
              id == self.VS_TYPE_ACCELEROMETER + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_GEOMAGNETIC_FIELD or 
              id == self.VS_TYPE_GEOMAGNETIC_FIELD + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_GYROSCOPE or 
              id == self.VS_TYPE_GYROSCOPE + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_GRAVITY or 
              id == self.VS_TYPE_GRAVITY + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_LINEAR_ACCELERATION or 
              id == self.VS_TYPE_LINEAR_ACCELERATION + self.BHY_SID_WAKEUP_OFFSET
             ):
            return self.BHY_DT_VECTOR_PLUS
        elif (id == self.VS_TYPE_ORIENTATION or 
              id == self.VS_TYPE_ORIENTATION + self.BHY_SID_WAKEUP_OFFSET
             ):
             return self.BHY_DT_VECTOR_ORIENTATION

        elif (id == self.VS_TYPE_LIGHT or 
              id == self.VS_TYPE_LIGHT + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_PROXIMITY or 
              id == self.VS_TYPE_PROXIMITY + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_RELATIVE_HUMIDITY or 
              id == self.VS_TYPE_RELATIVE_HUMIDITY + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_STEP_COUNTER or 
              id == self.VS_TYPE_STEP_COUNTER + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_TEMPERATURE or 
              id == self.VS_TYPE_TEMPERATURE + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_AMBIENT_TEMPERATURE or 
              id == self.VS_TYPE_AMBIENT_TEMPERATURE + self.BHY_SID_WAKEUP_OFFSET
             ):
             return self.BHY_DT_SCALAR

        elif (id == self.VS_TYPE_PRESSURE or 
              id == self.VS_TYPE_PRESSURE + self.BHY_SID_WAKEUP_OFFSET
             ):
            return self.BHY_DT_SCALAR_BARO

        elif (id == self.VS_TYPE_SIGNIFICANT_MOTION or 
              id == self.VS_TYPE_SIGNIFICANT_MOTION + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_SIGNIFICANT_MOTION or 
              id == self.VS_TYPE_SIGNIFICANT_MOTION + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_STEP_DETECTOR or
              id == self.VS_TYPE_STEP_DETECTOR + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_TILT or 
              id == self.VS_TYPE_TILT + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_WAKEUP or 
              id == self.VS_TYPE_WAKEUP + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_GLANCE or 
              id == self.VS_TYPE_GLANCE + self.BHY_SID_WAKEUP_OFFSET or

              id == self.VS_TYPE_PICKUP or 
              id == self.VS_TYPE_PICKUP + self.BHY_SID_WAKEUP_OFFSET
             ):
            return self.BHY_DT_EVENT_DATA

        elif (id == self.VS_TYPE_MAGNETIC_FIELD_UNCALIBRATED or 
              id == self.VS_TYPE_MAGNETIC_FIELD_UNCALIBRATED + self.BHY_SID_WAKEUP_OFFSET or
              
              id == self.VS_TYPE_GYROSCOPE_UNCALIBRATED or 
              id == self.VS_TYPE_GYROSCOPE_UNCALIBRATED + self.BHY_SID_WAKEUP_OFFSET
             ):
             return self.BHY_DT_VECTOR_UNCALIB

        elif (id == self.VS_TYPE_HEART_RATE or 
              id == self.VS_TYPE_HEART_RATE + self.BHY_SID_WAKEUP_OFFSET
             ):
             return self.BHY_DT_SCALAR_HEART

        elif (id == self.VS_TYPE_ACTIVITY_RECOGNITION or 
              id == self.VS_TYPE_ACTIVITY_RECOGNITION + self.BHY_SID_WAKEUP_OFFSET
             ):
             return self.BHY_DT_ACTIVITY_DATA

        elif id == self.EV_DEBUG:
            return self.BHY_DT_DEBUG

        elif (id == self.EV_BSX_A or
              id == self.EV_BSX_B or
              id == self.EV_BSX_C
             ):
             return self.BHY_DT_BSX

        elif (id == self.EV_TIMESTAMP_LSW or 
              id == self.EV_WAKEUP_TIMESTAMP_LSW or
              
              id == self.EV_TIMESTAMP_MSW or 
              id == self.EV_WAKEUP_TIMESTAMP_MSW
             ):
             return self.BHY_DT_TIME

        elif (id == self.EV_META_EVENTS or 
              id == self.EV_WAKEUP_META_EVENTS
             ):
             return self.BHY_DT_META

        else:
            return -1 # Unkown event

    def sensorIdToName(self, id):
        if id == self.VS_TYPE_ACCELEROMETER:
            return "VS_TYPE_ACCELEROMETER"
        elif id == self.VS_TYPE_GEOMAGNETIC_FIELD:
            return "VS_TYPE_GEOMAGNETIC_FIELD"
        elif id == self.VS_TYPE_ORIENTATION:
            return "VS_TYPE_ORIENTATION"
        elif id == self.VS_TYPE_GYROSCOPE:
            return "VS_TYPE_GYROSCOPE"
        elif id == self.VS_TYPE_LIGHT:
            return "VS_TYPE_LIGHT"
        elif id == self.VS_TYPE_PRESSURE:
            return "VS_TYPE_PRESSURE"
        elif id == self.VS_TYPE_TEMPERATURE:
            return "VS_TYPE_TEMPERATURE"
        elif id == self.VS_TYPE_PROXIMITY:
            return "VS_TYPE_PROXIMITY"
        elif id == self.VS_TYPE_GRAVITY:
            return "VS_TYPE_GRAVITY"
        elif id == self.VS_TYPE_LINEAR_ACCELERATION:
            return "VS_TYPE_LINEAR_ACCELERATION"
        elif id == self.VS_TYPE_ROTATION_VECTOR:
            return "VS_TYPE_ROTATION_VECTOR"
        elif id == self.VS_TYPE_RELATIVE_HUMIDITY:
            return "VS_TYPE_RELATIVE_HUMIDITY"
        elif id == self.VS_TYPE_AMBIENT_TEMPERATURE:
            return "VS_TYPE_AMBIENT_TEMPERATURE"
        elif id == self.VS_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
            return "VS_TYPE_MAGNETIC_FIELD_UNCALIBRATED"
        elif id == self.VS_TYPE_GAME_ROTATION_VECTOR:
            return "VS_TYPE_GAME_ROTATION_VECTOR"
        elif id == self.VS_TYPE_GYROSCOPE_UNCALIBRATED:
            return "VS_TYPE_GYROSCOPE_UNCALIBRATED"
        elif id == self.VS_TYPE_SIGNIFICANT_MOTION:
            return "VS_TYPE_SIGNIFICANT_MOTION"
        elif id == self.VS_TYPE_STEP_DETECTOR:
            return "VS_TYPE_STEP_DETECTOR"
        elif id == self.VS_TYPE_STEP_COUNTER:
            return "VS_TYPE_STEP_COUNTER"
        elif id == self.VS_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
            return "VS_TYPE_GEOMAGNETIC_ROTATION_VECTOR"
        elif id == self.VS_TYPE_HEART_RATE:
            return "VS_TYPE_HEART_RATE"
        elif id == self.VS_TYPE_TILT:
            return "VS_TYPE_TILT"
        elif id == self.VS_TYPE_WAKEUP:
            return "VS_TYPE_WAKEUP"
        elif id == self.VS_TYPE_GLANCE:
            return "VS_TYPE_GLANCE"
        elif id == self.VS_TYPE_PICKUP:
            return "VS_TYPE_PICKUP"
        elif id == self.VS_TYPE_ACTIVITY_RECOGNITION:
            return "VS_TYPE_ACTIVITY_RECOGNITION"
        else:
            return "Unkow sensors"
  
    def parseQuaternionPlus(self, data):
        out = {}
        out["sensor_id"],out["x"],out["y"],out["z"],out["w"],out["accuracy"] = struct.unpack('<B5h', data)
        out["x"] /= 16384
        out["y"] /= 16384
        out["z"] /= 16384
        out["w"] /= 16384
        return out

    def parseOrientation(self, data):
        out = {}
        out["sensor_id"],out["x"],out["y"],out["z"],out["accuracy"] = struct.unpack('<BH2hB', data)
        out["x"] *= 360
        out["x"] /= 32768
        out["y"] *= 360
        out["y"] /= 32768
        out["z"] *= 360
        out["z"] /= 32768
        return out

    def parseVectorPlus(self, data):
        out = {}
        out["sensor_id"],out["x"],out["y"],out["z"],out["accuracy"] = struct.unpack('<B3hB', data)
        return out
    
    def parseVectorUncalib(self, data):
        out = {}
        out["sensor_id"],out["x"],out["y"],out["z"],out["x_bias"],out["y_bias"],out["z_bias"],out["accuracy"] = struct.unpack('<B6hB', data)
        return out

    def parseScalar(self, data):
        out = {}
        out["sensor_id"],out["data"] = struct.unpack('<Bh', data)
        return out

    # TODO: Implement 24bit integer parsing
    def parseScalarBaro(self, data):
        raise Exception("NotImplemented", "Parsing Barometer data is not implemented yet")
    
    # TODO: Find out what format Heart rate is
    def parseScalarHeart(self, data):
        raise Exception("NotImplemented", "Parsing Heart Rate data is not implemented yet")

    def parseEventData(self, data):
        out = {}
        out["sensor_id"] = struct.unpack('<B', data)[0]
        return out

    def parseActivityRecognitionData(self, data):
        out = {}
        list = []
        out["sensor_id"], activity = struct.unpack('>BH', data)
        # TODO: I have swapped the "started" and "ended" adjective.. dont know why
        if(activity & (1<<0)):
            list.append("Still activity started")
        if(activity & (1<<1)):
            list.append("Walking activity started")
        if(activity & (1<<2)):
            list.append("Running activity started")
        if(activity & (1<<3)):
            list.append("On Bicycle activity started")
        if(activity & (1<<4)):
            list.append("In Vehicle activity started")
        if(activity & (1<<5)):
            list.append("Tilting activity started")
        if(activity & (1<<6)):
            list.append("Reserved (6)")
        if(activity & (1<<7)):
            list.append("Reserved (7)")
        if(activity & (1<<8)):
            list.append("Still activity ended")
        if(activity & (1<<9)):
            list.append("Walking activity ended")
        if(activity & (1<<10)):
            list.append("Running activity ended")
        if(activity & (1<<11)):
            list.append("On Bicycle activity ended")
        if(activity & (1<<12)):
            list.append("In Vehicle activity ended")
        if(activity & (1<<13)):
            list.append("Tilting activity ended")
        if(activity & (1<<14)):
            list.append("Reserved (14)")
        if(activity & (1<<15)):
            list.append("Reserved (15)")
        
        out["activity_list"] = list 
        return out

    def parseDebug(self, data):
        raise Exception("NotImplemented", "Parsing Debug data is not implemented yet")

    def parseTime(self, data):
        out = {}
        out["sensor_id"], out['timestamp'] = struct.unpack('<Bh', data)
        return out

    def parseMetaEvent(self, data):
        if len(data) != 4:
            return "Malformed Event",""

        if data[0] != self.EV_META_EVENTS and data[0] != self.EV_WAKEUP_META_EVENTS:
            return "Not a Meta Event!", ""

        if data[1] == 0:
            return "Not used", ""
        elif data[1] == 1:
            return "Flush Completed", "Sensor Type:"+str(data[2:])
        elif data[1] == 4:
            return "Error", "Error Register:"+str(data[2:])
        elif data[1] == 11:
            return "Sensor error", ""
        elif data[1] == 15:
            info = "Self-Test Results"
            desc = "Sensor Type: "
            if data[2] == 1:
                desc += "Accelerometer ID 1"
            elif data[2] == 16:
                desc += "Uncalibrated Gyroscope ID 16"
            elif data[2] == 14:
                desc += "Uncalibrated Magnetometer ID 14"

            desc += " - STATUS: "

            if data[3] == 0:
                desc += "Test PASSED"
            else:
                desc += "Test FAILED - " + str(data[3])

            return info, desc
        elif data[1] == 16:
            return "Inizialized", "RAM Ver LSB:"+str(binascii.hexlify(data[2:]))