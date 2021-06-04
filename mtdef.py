"""Constant and messages definition for MT communication."""
import struct

class DeviceState:
    """State of the device"""
    # measurement state
    Measurement = 0
    # config state
    Config = 1


class MID:
    """Values for the message id (MID)"""
    # Error message, 1 data byte
    Error = 0x42
    # Warning message, 1 data byte
    Warning = 0x43

    # State MID
    # Wake up procedure
    WakeUp = 0x3E
    # Wake up ack to put device in config mode
    WakeUpAck = 0x3F
    # Switch to config state
    GoToConfig = 0x30
    # Switch to measurement state
    GoToMeasurement = 0x10
    # Reset device
    Reset = 0x40

    # Informational messages
    # Request device id
    ReqDID = 0x00
    # DeviceID, 4 bytes: HH HL LH LL
    DeviceID = 0x01
    # Request product code in plain text
    ReqProductCode = 0x1C
    # Product code (max 20 bytes data)
    ProductCode = 0x1D
    # ReqHardwareVersion
    ReqHardwareVersion = 0x1E
    # HardwareVersion
    HardwareVersion = 0x1F

    # Request firmware revision
    ReqFWRev = 0x12
    # Firmware revision, 3 bytes: major minor rev
    FirmwareRev = 0x13

    # Device specific messages
    # Restore factory defaults
    RestoreFactoryDef = 0x0E
    # Baudrate, 1 byte
    SetBaudrate = 0x18
    # Run the built-in self test (MTi-1/10/100 series)
    RunSelftest = 0x24
    # Self test results, 2 bytes
    SelftestAck = 0x25
    # Error mode, 2 bytes, 0000, 0001, 0002, 0003 (default 0001)
    SetErrorMode = 0xDA
    # Transmit delay (RS485), 2 bytes, number of clock ticks (1/29.4912 MHz)
    SetTransmitDelay = 0xDC
    # Set state of OptionFlags (MTi-1/2/3), 4 + 4 bytes
    SetOptionFlags = 0x48
    # Location ID, 2 bytes, arbitrary, default is 0
    SetLocationID = 0x84

    # Synchronization messages
    # Synchronization settings (MTi-1/10/100 series only), N*12 bytes
    SetSyncSettings = 0x2C

    # Configuration messages
    # Request configuration
    ReqConfiguration = 0x0C
    # Configuration, 118 bytes
    Configuration = 0x0D

    # Extended output mode (MTi-10/100), 2 bytes, bit 4 for extended UART
    SetExtOutputMode = 0x86
    # Output configuration (MTi-1/10/100 series only), N*4 bytes
    SetOutputConfiguration = 0xC0
    # Configure NMEA data output (MTi-10/100), 2 bytes
    SetStringOutputType = 0x8E
    # Set sensor of local alignment quaternion
    SetAlignmentRotation = 0xEC

    # Lever arm of the GPSin sensor coordinates (MTi-G and MTi-700 only),
    # 3*4 bytes
    SetGnssLeverArm = 0x68
    SetGnssLeverArmAck = 0x69

    # Data messages
    # Request MTData message (for 65535 skip factor)
    ReqData = 0x34
    # Legacy data packet
    MTData = 0x32
    # Newer data packet (MTi-10/100 series only)
    MTData2 = 0x36

    # Filter messages
    # Reset orientation, 2 bytes
    ResetOrientation = 0xA4
    # Request or set UTC time from sensor (MTI-G and MTi-10/100 series)
    SetUTCTime = 0x60
    # Set correction ticks to UTC time
    AdjustUTCTime = 0xA8
    # UTC Time (MTI-G and MTi-10/100 series), 12 bytes
    UTCTime = 0x61
    # Request the available XKF scenarios on the device
    ReqAvailableScenarios = 0x62
    # Available Scenarios
    AvailableScenarios = 0x63
    # SetFilterProfile, data
    SetFilterProfile = 0x64
    # Magnitude of the gravity used for the sensor fusion mechanism, 4 bytes
    SetGravityMagnitude = 0x66
    # Latitude, Longitude and Altitude for local declination and gravity
    # (MTi-10/100 series only), 24 bytes
    SetLatLonAlt = 0x6E
    SetLatLonAltAck = 0x6F
    # Initiate No Rotation procedure (not on MTi-G), 2 bytes
    SetNoRotation = 0x22
    # SetGnssPlatform
    SetGnssPlatform = 0x76
    # SetGnssReceiverSettings
    SetGnssReceiverSettings = 0xAC
    # SetInitialHeading
    SetInitialHeading = 0xD6
    SetInitialHeadingAck = 0xD7
    # ForwardGnssData
    ForwardGnssData = 0xE2
    ForwardGnssDataAck = 0xE3


class DeprecatedMID:
    """Deprecated message Ids."""
    # Informational messages
    # Compatibility for XBus Master users
    InitMT = 0x02
    InitMTResults = 0x03
    # Request data length according to current configuration
    ReqDataLength = 0x0A
    # Data Length, 2 bytes
    DataLength = 0x0B
    # Request GPS status (MTi-G only)
    ReqGPSStatus = 0xA6
    # GPS status (MTi-G only)
    GPSStatus = 0xA7

    # Synchronization messages
    # SyncIn setting (MTi only), (1+) 2 or 4 bytes depending on request
    SetSyncInSettings = 0xD6
    # SyncOut setting (MTi/MTi-G only), (1+) 2 or 4 bytes depending on request
    SetSyncOutSettings = 0xD8

    # Configuration messages

    # Object alignment matrix, 9*4 bytes
    SetObjectAlignment = 0xE0
    # Sampling period (MTi/MTi-G only), 2 bytes
    SetPeriod = 0x04
    # Skip factor (MTi/MTi-G only), 2 bytes
    SetOutputSkipFactor = 0xD4
    # Output mode (MTi/MTi-G only), 2 bytes
    SetOutputMode = 0xD0
    # Output settings (MTi/MTi-G only), 4 bytes
    SetOutputSettings = 0xD2
    # XKF Filter messages
    # Heading (MTi only), 4 bytes
    SetHeading = 0x82

    # Magnetic declination (MTi-G only), 4 bytes
    SetMagneticDeclination = 0x6A
    # Latitude, Longitude and Altitude for local declination and gravity
    # Processing flags (not on firmware 2.2 or lower for MTi/MTi-g), 1 byte
    SetProcessingFlags = 0x20


def getName(cls, value):
    '''Return the name of the first found member of class cls with given
    value.'''
    for k, v in cls.__dict__.items():
        if v == value:
            return k
    return ''


def getMIDName(mid):
    '''Return the name of a message given the message id.'''
    name = getName(MID, mid)
    if name:
        return name
    if mid & 1:
        name = getName(MID, mid-1)
        if name:
            return name+'Ack'
    return 'unknown MID'


class Baudrates(object):
    """Baudrate information and conversion."""
    # Baudrate mapping between ID and value
    Baudrates = [
        (0x0D, 4000000),
        (0x0E, 3686400),
        (0x0C, 2000000),
        (0x80,  921600),
        (0x0A,  921600),
        (0x00,  460800),
        (0x01,  230400),
        (0x02,  115200),
        (0x03,   76800),
        (0x04,   57600),
        (0x05,   38400),
        (0x06,   28800),
        (0x07,   19200),
        (0x08,   14400),
        (0x09,    9600),
        (0x0B,    4800),
        (0x80,  921600)]

    @classmethod
    def get_BRID(cls, baudrate):
        """Get baudrate id for a given baudrate."""
        for brid, br in cls.Baudrates:
            if baudrate == br:
                return brid
        raise MTException("unsupported baudrate.")

    @classmethod
    def get_BR(cls, baudrate_id):
        """Get baudrate for a given baudrate id."""
        for brid, br in cls.Baudrates:
            if baudrate_id == brid:
                return br
        raise MTException("unknown baudrate id.")


class OutputMode:
    """Values for the output mode."""
    Temp = 0x0001
    Calib = 0x0002
    Orient = 0x0004
    Auxiliary = 0x0008
    Position = 0x0010
    Velocity = 0x0020
    Status = 0x0800
    RAWGPS = 0x1000  # supposed to be incompatible with previous
    RAW = 0x4000  # incompatible with all except RAWGPS


class OutputSettings:
    """Values for the output settings."""
    Timestamp_None = 0x00000000
    Timestamp_SampleCnt = 0x00000001
    Timestamp_UTCTime = 0x00000002
    OrientMode_Quaternion = 0x00000000
    OrientMode_Euler = 0x00000004
    OrientMode_Matrix = 0x00000008
    CalibMode_AccGyrMag = 0x00000000
    CalibMode_GyrMag = 0x00000010
    CalibMode_AccMag = 0x00000020
    CalibMode_Mag = 0x00000030
    CalibMode_AccGyr = 0x00000040
    CalibMode_Gyr = 0x00000050
    CalibMode_Acc = 0x00000060
    CalibMode_Mask = 0x00000070
    DataFormat_Float = 0x00000000
    DataFormat_12_20 = 0x00000100  # not supported yet
    DataFormat_16_32 = 0x00000200  # not supported yet
    DataFormat_Double = 0x00000300  # not supported yet
    AuxiliaryMode_NoAIN1 = 0x00000400
    AuxiliaryMode_NoAIN2 = 0x00000800
    PositionMode_LLA_WGS84 = 0x00000000
    VelocityMode_MS_XYZ = 0x00000000
    Coordinates_NED = 0x80000000


class XDIType:
    """Values for the XDI groups."""
    #
    TemperatureGroup = 0x0800
    Temperature = 0x0810
    #
    TimestampGroup = 0x1000
    UtcTime = 0x1010
    PacketCounter = 0x1020
    SampleTimeFine = 0x1060
    SampleTimeCoarse = 0x1070
    #
    OrientationGroup = 0x2000
    Quaternion = 0x2010
    RotationMatrix = 0x2020
    EulerAngles = 0x2030
    #
    PressureGroup = 0x3000
    BaroPressure = 0x3010
    #
    AccelerationGroup = 0x4000
    DeltaV = 0x4010
    Acceleration = 0x4020
    FreeAcceleration = 0x4030
    AccelerationHR = 0x4040
    #
    PositionGroup = 0x5000
    AltitudeEllipsoid = 0x5020
    PositionEcef = 0x5030
    LatLon = 0x5040
    #
    GnssGroup = 0x7000
    GnssPvtData = 0x7010
    GnssSatInfo = 0x7020
    GnssPvtPulse = 0x7030
    #
    AngularVelocityGroup = 0x8000
    RateOfTurn = 0x8020
    DeltaQ = 0x8030
    RateOfTurnHR = 0x8040
    #
    RawSensorGroup = 0xA000
    RawAccGyrMagTemp = 0xA010
    RawGyroTemp = 0xA020
    #
    MagneticGroup = 0xC000
    MagneticField = 0xC020
    #
    VelocityGroup = 0xD000
    VelocityXYZ = 0xD010
    #
    StatusGroup = 0xE000
    StatusByte = 0xE010
    StatusWord = 0xE020
    DeviceId = 0xE030
    LocationId = 0xE040


class XDIDataId:
    """Settings for MTData2 Data Identifier"""
    Float32 = 0x00
    Fp1220 = 0x01
    Fp1632 = 0x02
    Float64 = 0x03
    ENU = 0x00
    NED = 0x04
    NWU = 0x08

class SyncFunction:
    OnePpsTimePulse = 14 # 0x0E
    ClockBiasEstimation = 9 # 0x09
    IntervalTransmitionMeasurement = 4 # 0x04
    ResetTimer = 2 # 0x02
    SampleAndSend = 7 # 0x07
    SendLast = 8 # 0x08
    StartSampling = 11 # 0x0B
    TriggerIndication = 3 # 0x03

class SyncLine:
    Bi1In = 3 # 0x03
    Bi1Out = 4 # 0x04
    ClockIn  = 0 # 0x00
    Gnss1Pps = 8 # 0x08
    GnssClockIn = 1 # 0x01
    In1 = 2 # 0x02
    In2 = 9 # 0x09
    Out = 7 # 0x07
    ReqData = 6 # 0x06

class SyncPolarity:
    Disabled = 0
    Positive = 1
    Negative = 2
    Both  = 3

class SyncSetting:
    def __init__(self, function = SyncFunction.OnePpsTimePulse,
            line = SyncLine.ClockIn, polarity = SyncPolarity.Disabled,
            triggerOnce = 0, skipFirst = 0, skipFactor = 0,
            pulseWidth = 0, offset = 0):
        self.function = function
        self.line = line
        self.polarity = polarity
        self.triggerOnce = triggerOnce # Trigger only once (1) or multiple times (0).
        self.skipFirst = skipFirst # The number of initial events to skip before taking action
        self.skipFactor = skipFactor # The number of events to skip after taking the action before taking action again.
        self.pulseWidth = pulseWidth # The width of the generated pulse in 100μs
        self.offset = offset # Offset from event to pulse generation (100μs units, range [-30000..+30000]).
    def toBytes(self):
        lst = (self.function, self.line, self.polarity, self.triggerOnce,
            self.skipFirst, self.skipFactor, self.pulseWidth, self.offset)
        return struct.pack('!BBBBHHHH', *lst)
    def fromBytes(data):
        lst = struct.unpack('!BBBBHHHH', data)
        return SyncSetting(*lst)
    def __repr__(self):
        return f"SyncSetting({self.function},{self.line},{self.polarity},{self.triggerOnce},{self.skipFirst},{self.skipFactor},{self.pulseWidth},{self.offset})"
    def __str__(self):
        return f"SyncSetting {self.function=},{self.line=},{self.polarity=},{self.triggerOnce=},{self.skipFirst=},{self.skipFactor=},{self.pulseWidth=},{self.offset=}"

class MTException(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return self.message


class MTTimeoutException(MTException):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return 'Timeout: %s' % self.message


class MTErrorMessage(MTException):
    ErrorCodes = {
        0x03: 'Invalid period',
        0x04: 'Invalid message',
        0x1E: 'Timer overflow',
        0x20: 'Invalid baudrate',
        0x21: 'Invalid parameter',
        0x28: 'Device error, try updating the firmware'
    }

    def __init__(self, code):
        self.code = code
        self.message = self.ErrorCodes.get(code, 'Unknown error: 0x%02X' % code)

    def __str__(self):
        return 'Error message 0x%02X: %s' % (self.code, self.message)
