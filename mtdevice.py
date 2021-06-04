#!/usr/bin/env python
import serial
import struct
import sys
import getopt
import time
import glob
import re

from .mtdef import MID, OutputMode, OutputSettings, MTException, Baudrates, \
    XDIType, XDIDataId, getMIDName, DeviceState, DeprecatedMID, MTErrorMessage, \
    MTTimeoutException, SyncSetting


################################################################
# MTDevice class
################################################################
class MTDevice(object):
    """XSens MT device communication object."""

    def __init__(self, port, baudrate=115200, timeout=0.2,
                 config_mode=False, verbose=False):
        """Open device."""
        self.verbose = verbose
        # serial interface to the device
        try:
            print(f"Connect to {port} @ {baudrate}")
            self.device = serial.Serial(port, baudrate, timeout=timeout,
                                        writeTimeout=timeout)
        except IOError:
            # FIXME with pyserial3 we might need some specific flags
            self.device = serial.Serial(port, baudrate, timeout=timeout,
                                        writeTimeout=timeout, rtscts=True,
                                        dsrdtr=True)
        self.device.flushInput()    # flush to make sure the port is ready TODO
        self.device.flushOutput()    # flush to make sure the port is ready TODO
        # timeout for communication
        self.timeout = 1000*timeout
        # state of the device
        self.state = None
        if config_mode:
            self.GoToConfig()

    ############################################################
    # Low-level communication
    ############################################################
    def write_msg(self, mid, data=b''):
        """Low-level message sending function."""
        length = len(data)
        if length > 254:
            lendat = b'\xFF' + struct.pack('!H', length)
        else:
            lendat = struct.pack('!B', length)
        packet = b'\xFA\xFF' + struct.pack('!B', mid) + lendat + data
        packet += struct.pack('!B', 0xFF & (-(sum(packet[1:]))))
        msg = packet
        start = time.time()
        while ((time.time()-start) < self.timeout) and self.device.read():
            pass
        self.device.write(msg)
        if self.verbose:
            print("MT: Write message id 0x%02X (%s) with %d data bytes: [%s]" %\
                (mid, getMIDName(mid), length,
                 ' '.join("%02X" % v for v in data)))

    def waitfor(self, size=1):
        """Get a given amount of data."""
        buf = bytearray()
        for _ in range(100):
            buf.extend(self.device.read(size-len(buf)))
            if len(buf) == size:
                return buf
            if self.verbose:
                print("waiting for %d bytes, got %d so far: [%s]" % \
                    (size, len(buf), ' '.join('%02X' % v for v in buf)))
        raise MTTimeoutException("waiting for message")

    def read_msg(self):
        """Low-level message receiving function."""
        start = time.time()
        while (time.time()-start) < self.timeout:
            # first part of preamble
            if self.waitfor() != bytes.fromhex("FA"):
                continue
            # second part of preamble
            if self.waitfor() != bytes.fromhex("FF"):  # we assume no timeout anymore
                continue
            # read message id and length of message
            mid, length = struct.unpack('!BB', self.waitfor(2))
            if length == 255:    # extended length
                length, = struct.unpack('!H', self.waitfor(2))
            # read contents and checksum
            buf = self.waitfor(length+1)
            checksum = buf[-1]
            data = struct.unpack('!%dB' % length, buf[:-1])
            # check message integrity
            if 0xFF & sum(data, 0xFF+mid+length+checksum):
                if self.verbose:
                    sys.stderr.write("invalid checksum; discarding data and "
                                     "waiting for next message.\n")
                continue
            if self.verbose:
                print("MT: Got message id 0x%02X (%s) with %d data bytes: [%s]"\
                    % (mid, getMIDName(mid), length,
                       ' '.join("%02X" % v for v in data)))
            if mid == MID.Error:
                raise MTErrorMessage(data[0])
            return (mid, buf[:-1])
        else:
            raise MTException("could not find message.")

    def write_ack(self, mid, data=b'', n_retries=500):
        """Send a message and read confirmation."""
        self.write_msg(mid, data)
        for _ in range(n_retries):
            mid_ack, data_ack = self.read_msg()
            if mid_ack == (mid+1):
                break
            elif self.verbose:
                print("ack (0x%02X) expected, got 0x%02X instead" % \
                    (mid+1, mid_ack))
        else:
            raise MTException("Ack (0x%02X) expected, MID 0x%02X received "
                              "instead (after %d retries)." % (mid+1, mid_ack,
                                                               n_retries))
        return data_ack

    def _ensure_config_state(self):
        """Switch device to config state if necessary."""
        if self.state != DeviceState.Config:
            self.GoToConfig()

    def _ensure_measurement_state(self):
        """Switch device to measurement state if necessary."""
        if self.state != DeviceState.Measurement:
            self.GoToMeasurement()

    ############################################################
    # High-level functions
    ############################################################
    def Reset(self, go_to_config=False):
        """Reset MT device.

        If go_to_config then send WakeUpAck in order to leave the device in
        config mode.
        """
        self.write_ack(MID.Reset)
        if go_to_config:
            time.sleep(0.01)
            mid, _ = self.read_msg()
            if mid == MID.WakeUp:
                self.write_msg(MID.WakeUpAck)
                self.state = DeviceState.Config
        else:
            self.state = DeviceState.Measurement

    def GoToConfig(self):
        """Place MT device in configuration mode."""
        print("GoToConfig")
        self.write_ack(MID.GoToConfig)
        self.state = DeviceState.Config

    def GoToMeasurement(self):
        """Place MT device in measurement mode."""
        print("GoToMeasurement")
        self._ensure_config_state()
        self.write_ack(MID.GoToMeasurement)
        self.state = DeviceState.Measurement

    def GetDeviceID(self):
        """Get the device identifier."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqDID)
        deviceID, = struct.unpack('!I', data)
        return deviceID

    def GetProductCode(self):
        """Get the product code."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqProductCode)
        return str(data).strip()

    def GetFirmwareRev(self):
        """Get the firmware version."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqFWRev)
        # XXX unpacking only 3 characters in accordance with the documentation
        # but some devices send 11 bytes instead.
        major, minor, revision = struct.unpack('!BBB', data[:3])
        return (major, minor, revision)

    def RunSelfTest(self):
        """Run the built-in self test."""
        self._ensure_config_state()
        data = self.write_ack(MID.RunSelfTest)
        bit_names = ['accX', 'accY', 'accZ', 'gyrX', 'gyrY', 'gyrZ',
                     'magX', 'magY', 'magZ']
        self_test_results = []
        for i, name in enumerate(bit_names):
            self_test_results.append((name, (data >> i) & 1))
        return self_test_results

    def GetBaudrate(self):
        """Get the current baudrate id of the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetBaudrate)
        return data[0]

    def SetBaudrate(self, brid):
        """Set the baudrate of the device using the baudrate id."""
        self._ensure_config_state()
        self.write_ack(MID.SetBaudrate, (brid,))

    def GetErrorMode(self):
        """Get the current error mode of the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetErrorMode)
        error_mode, = struct.unpack('!H', data)
        return error_mode

    def SetErrorMode(self, error_mode):
        """Set the error mode of the device.

        The error mode defines the way the device deals with errors (expect
        message errors):
            0x0000: ignore any errors except message handling errors,
            0x0001: in case of missing sampling instance: increase sample
                counter and do not send error message,
            0x0002: in case of missing sampling instance: increase sample
                counter and send error message,
            0x0003: in case of non-message handling error, an error message is
                sent and the device will go into Config State.
        """
        self._ensure_config_state()
        data = struct.pack('!H', error_mode)
        self.write_ack(MID.SetErrorMode, data)

    def GetOptionFlags(self):
        """Get the option flags (MTi-1 series)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetOptionFlags)
        set_flags, clear_flags = struct.unpack('!II', data)
        return set_flags, clear_flags

    def SetOptionFlags(self, set_flags, clear_flags):
        """Set the option flags (MTi-1 series)."""
        self._ensure_config_state()
        data = struct.pack('!II', set_flags, clear_flags)
        self.write_ack(MID.SetOptionFlags, data)

    def GetLocationID(self):
        """Get the location ID of the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetLocationID)
        location_id, = struct.unpack('!H', data)
        return location_id

    def SetLocationID(self, location_id):
        """Set the location ID of the device (arbitrary)."""
        self._ensure_config_state()
        data = struct.pack('!H', location_id)
        self.write_ack(MID.SetLocationID, data)

    def RestoreFactoryDefaults(self):
        """Restore MT device configuration to factory defaults (soft version).
        """
        self._ensure_config_state()
        self.write_ack(MID.RestoreFactoryDef)

    def GetTransmitDelay(self):
        """Get the transmission delay (only RS485)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetTransmitDelay)
        transmit_delay, = struct.unpack('!H', data)
        return transmit_delay

    def SetTransmitDelay(self, transmit_delay):
        """Set the transmission delay (only RS485)."""
        self._ensure_config_state()
        data = struct.pack('!H', transmit_delay)
        self.write_ack(MID.SetTransmitDelay, data)

    def GetSyncSettings(self):
        """Get the synchronisation settings."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetSyncSettings)
        sync_settings = [SyncSetting.fromBytes(data[o:o+12])
                         for o in range(0, len(data), 12)]
        return sync_settings

    def SetSyncSettings(self, sync_settings):
        """Set the synchronisation settings (mark IV)"""
        # Function / Line / Polarity / TriggerOnce / skipfactor / Pulse Width / Delay
        self._ensure_config_state()
        data = b''.join(ss.toBytes() for ss in sync_settings)
        self.write_ack(MID.SetSyncSettings, data)

    def GetOutputConfiguration(self):
        """Get the output configuration of the device (mark IV)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetOutputConfiguration)
        output_configuration = [struct.unpack('!HH', data[o:o+4])
                                for o in range(0, len(data), 4)]
        return output_configuration

    def SetOutputConfiguration(self, output_configuration):
        """Set the output configuration of the device (mark IV)."""
        self._ensure_config_state()
        data = b''.join(struct.pack('!HH', *output)
                        for output in output_configuration)
        self.write_ack(MID.SetOutputConfiguration, data)

    def GetStringOutputType(self):
        """Get the NMEA data output configuration."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetStringOutputType)
        string_output_type, = struct.unpack('!H', data)
        return string_output_type

    def SetStringOutputType(self, string_output_type):
        """Set the configuration of the NMEA data output."""
        self._ensure_config_state()
        data = struct.pack('!H', string_output_type)
        self.write_ack(MID.SetStringOutputType, data)

    def GetAlignmentRotation(self, parameter):
        """Get the object alignment.

        parameter indicates the desired alignment quaternion:
            0 for sensor alignment (RotSensor),
            1 for local alignment (RotLocal).
        """
        self._ensure_config_state()
        data = struct.pack('!B', parameter)
        data = self.write_ack(MID.SetAlignmentRotation, data)
        q0, q1, q2, q3 = struct.unpack('!ffff', data)
        return q0, q1, q2, q3

    def SetAlignmentRotation(self, parameter, quaternion):
        """Set the object alignment.

        parameter indicates the desired alignment quaternion:
            0 for sensor alignment (RotSensor),
            1 for local alignment (RotLocal).
        """
        self._ensure_config_state()
        data = struct.pack('!Bffff', parameter, *quaternion)
        self.write_ack(MID.SetAlignmentRotation, data)

    def GetExtOutputMode(self):
        """Get current extended output mode (for alternative UART)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetExtOutputMode)
        ext_mode, = struct.unpack('!H', data)
        return ext_mode

    def SetExtOutputMode(self, ext_mode):
        """Set extended output mode (for alternative UART)."""
        self._ensure_config_state()
        data = struct.pack('!H', ext_mode)
        self.write_ack(MID.SetExtOutputMode, data)

    def GetLatLonAlt(self):
        """Get the stored position of the device.
        It is used internally for local magnetic declination and local gravity.
        """
        self._ensure_config_state()
        data = self.write_ack(MID.SetLatLonAlt)
        if len(data) == 24:
            lat, lon, alt = struct.unpack('!ddd', data)
        elif len(data) == 12:
            lat, lon, alt = struct.unpack('!fff', data)
        else:
            raise MTException('Could not parse ReqLatLonAltAck message: wrong'
                              'size of message.')
        return (lat, lon, alt)

    def SetLatLonAlt(self, lat, lon, alt):
        """Set the position of the device.
        It is used internally for local magnetic declination and local gravity.
        """
        self._ensure_config_state()
        data = struct.pack('!ddd', lat, lon, alt)
        self.write_ack(MID.SetLatLonAlt, data)

    def GetAvailableScenarios(self):
        """Get the available XKF scenarios on the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqAvailableScenarios)
        scenarios = []
        try:
            for i in range(len(data)/22):
                scenario_type, version, label =\
                    struct.unpack('!BB20s', data[22*i:22*(i+1)])
                scenarios.append((scenario_type, version, label.strip()))
            # available XKF scenarios
            self.scenarios = scenarios
        except struct.error:
            raise MTException("could not parse the available XKF scenarios.")
        return scenarios

    def GetCurrentScenario(self):
        """Get the ID of the currently used XKF scenario."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetCurrentScenario)
        _, self.scenario_id = struct.unpack('!BB', data)  # version, id
        return self.scenario_id

    def SetCurrentScenario(self, scenario_id):
        """Sets the XKF scenario to use."""
        self._ensure_config_state()
        data = struct.pack('!BB', 0, scenario_id)  # version, id
        self.write_ack(MID.SetCurrentScenario, data)

    def ResetOrientation(self, code):
        """Reset the orientation.

        Code can take several values:
            0x0000: store current settings (only in config mode),
            0x0001: heading reset (NOT supported by MTi-G),
            0x0003: object reset.
        """
        data = struct.pack('!H', code)
        self.write_ack(MID.ResetOrientation, data)

    def SetNoRotation(self, duration):
        """Initiate the "no rotation" procedure to estimate gyro biases."""
        self._ensure_measurement_state()
        data = struct.pack('!H', duration)
        self.write_ack(MID.SetNoRotation, data)

    def GetUTCTime(self):
        """Get UTC time from device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetUTCTime)
        ns, year, month, day, hour, minute, second, flag = \
            struct.unpack('!IHBBBBBB', data)
        return (ns, year, month, day, hour, minute, second, flag)

    def SetUTCTime(self, ns, year, month, day, hour, minute, second, flag):
        """Set UTC time on the device."""
        self._ensure_config_state()
        data = struct.pack('!IHBBBBBB', ns, year, month, day, hour, minute,
                           second, flag)  # no clue what setting flag can mean
        self.write_ack(MID.SetUTCTime, data)

    def AdjustUTCTime(self, ticks):
        """Adjust UTC Time of device using correction ticks (0.1 ms)."""
        self._ensure_config_state()
        data = struct.pack('!i', ticks)
        self.write_msg(MID.AdjustUTCTime, data)  # no ack mentioned in the doc

    def read_measurement(self, mode=None, settings=None):
        self._ensure_measurement_state()
        # getting data
        mid, data = self.read_msg()
        if mid == MID.MTData:
            raise MTException("obsolete MTData.")
        elif mid == MID.MTData2:
            return self.parse_MTData2(data)
        else:
            #return None
            raise MTException("unknown data message: mid=0x%02X (%s)." %
                              (mid, getMIDName(mid)))

    def parse_MTData2(self, data):
        # Functions to parse each type of packet
        def parse_temperature(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Temperature
                o['Temp'], = parse_floats(content, ffmt, 1)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_timestamp(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # UTC Time
                o['ns'], o['Year'], o['Month'], o['Day'], o['Hour'],\
                    o['Minute'], o['Second'], o['Flags'] =\
                    struct.unpack('!LHBBBBBB', content)
            elif (data_id & 0x00F0) == 0x20:  # Packet Counter
                o['PacketCounter'], = struct.unpack('!H', content)
            elif (data_id & 0x00F0) == 0x30:  # Integer Time of Week
                o['TimeOfWeek'], = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x40:  # GPS Age  # deprecated
                o['gpsAge'], = struct.unpack('!B', content)
            elif (data_id & 0x00F0) == 0x50:  # Pressure Age  # deprecated
                o['pressureAge'], = struct.unpack('!B', content)
            elif (data_id & 0x00F0) == 0x60:  # Sample Time Fine
                o['SampleTimeFine'], = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x70:  # Sample Time Coarse
                o['SampleTimeCoarse'], = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x80:  # Frame Range
                o['startFrame'], o['endFrame'] = struct.unpack('!HH', content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_orientation_data(data_id, content, ffmt):
            o = {}
            o['frame'] = parse_frame(data_id)
            if (data_id & 0x00F0) == 0x10:  # Quaternion
                o['Q0'], o['Q1'], o['Q2'], o['Q3'] = \
                    parse_floats(content, ffmt, 4)
                #struct.unpack('!'+4*ffmt,content)
            elif (data_id & 0x00F0) == 0x20:  # Rotation Matrix
                o['a'], o['b'], o['c'], o['d'], o['e'], o['f'], o['g'], o['h'],\
                    o['i'] = parse_floats(content, ffmt, 9)
            elif (data_id & 0x00F0) == 0x30:  # Euler Angles
                o['Roll'], o['Pitch'], o['Yaw'] = parse_floats(content, ffmt, 3)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_pressure(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Baro pressure
                o['Pressure'], = struct.unpack('!L', content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_acceleration(data_id, content, ffmt):
            o = {}
            o['frame'] = parse_frame(data_id)
            if (data_id & 0x00F0) == 0x10:  # Delta V
                o['Delta v.x'], o['Delta v.y'], o['Delta v.z'] = \
                    parse_floats(content, ffmt, 3)
                    # struct.unpack('!'+3*ffmt, content)
            elif (data_id & 0x00F0) == 0x20:  # Acceleration
                o['accX'], o['accY'], o['accZ'] = \
                    parse_floats(content, ffmt, 3)
                    # struct.unpack('!'+3*ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Free Acceleration
                o['freeAccX'], o['freeAccY'], o['freeAccZ'] = \
                    parse_floats(content, ffmt, 3)
                    # struct.unpack('!'+3*ffmt, content)
            elif (data_id & 0x00F0) == 0x40:  # AccelerationHR
                o['accX'], o['accY'], o['accZ'] = \
                    parse_floats(content, ffmt, 3)
                    #struct.unpack('!'+3*ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_position(data_id, content, ffmt):
            o = {}
            o['frame'] = parse_frame(data_id)
            if (data_id & 0x00F0) == 0x10:  # Altitude MSL  # deprecated
                o['altMsl'], = parse_floats(content, ffmt, 1)
                # struct.unpack('!'+ffmt, content)
            elif (data_id & 0x00F0) == 0x20:  # Altitude Ellipsoid
                o['altEllipsoid'], = parse_floats(content, ffmt, 1)
                # struct.unpack('!'+ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Position ECEF
                o['ecefX'], o['ecefY'], o['ecefZ'] = \
                    struct.unpack('!'+3*ffmt, content)
            elif (data_id & 0x00F0) == 0x40:  # LatLon
                o['lat'], o['lon'] = parse_floats(content, ffmt, 2)
                # struct.unpack('!'+2*ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_GNSS(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # GnssPvtData
                o['itow'], o['year'], o['month'], o['day'], o['hour'],\
                    o['min'], o['sec'], o['valid'], o['tAcc'], o['nano'],\
                    o['fixtype'], o['flags'], o['numSV'], _ , o['lon'], o['lat'],\
                    o['height'], o['hMSL'], o['hAcc'], o['vAcc'], o['velN'],\
                    o['velE'], o['velD'], o['gSpeed'], o['headMot'], o['sAcc'],\
                    o['headAcc'], o['headVeh'], o['gdop'], o['pdop'],\
                    o['tdop'], o['vdop'], o['hdop'], o['ndop'], o['edop'] = \
                    struct.unpack('!IHBBBBBBIiBBBBiiiiIIiiiiiIIiHHHHHHH',
                                  content)
                # scaling correction
                o['lon'] *= 1e-7
                o['lat'] *= 1e-7
                o['headMot'] *= 1e-5
                o['headVeh'] *= 1e-5
                o['gdop'] *= 0.01
                o['pdop'] *= 0.01
                o['tdop'] *= 0.01
                o['vdop'] *= 0.01
                o['hdop'] *= 0.01
                o['ndop'] *= 0.01
                o['edop'] *= 0.01
            elif (data_id & 0x00F0) == 0x20:  # GNSS satellites info
                o['iTOW'], o['numSvs'] = struct.unpack('!LBxxx', content[:8])
                svs = []
                ch = {}
                for i in range(o['numSvs']):
                    ch['gnssId'], ch['svId'], ch['cno'], ch['flags'] = \
                        struct.unpack('!BBBB', content[8+4*i:12+4*i])
                    svs.append(ch)
                o['svs'] = svs
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_angular_velocity(data_id, content, ffmt):
            o = {}
            o['frame'] = parse_frame(data_id)
            if (data_id & 0x00F0) == 0x20:  # Rate of Turn
                o['gyrX'], o['gyrY'], o['gyrZ'] = \
                    parse_floats(content, ffmt, 3)
            elif (data_id & 0x00F0) == 0x30:  # Delta Q
                o['Delta q0'], o['Delta q1'], o['Delta q2'], o['Delta q3'] = \
                    parse_floats(content, ffmt, 4)
            elif (data_id & 0x00F0) == 0x40:  # RateOfTurnHR
                o['gyrX'], o['gyrY'], o['gyrZ'] = \
                    parse_floats(content, ffmt, 3)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_GPS(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x30:  # DOP
                o['iTOW'], g, p, t, v, h, n, e = \
                    struct.unpack('!LHHHHHHH', content)
                o['gDOP'], o['pDOP'], o['tDOP'], o['vDOP'], o['hDOP'], \
                    o['nDOP'], o['eDOP'] = 0.01*g, 0.01*p, 0.01*t, \
                    0.01*v, 0.01*h, 0.01*n, 0.01*e
            elif (data_id & 0x00F0) == 0x40:  # SOL
                o['iTOW'], o['fTOW'], o['Week'], o['gpsFix'], o['Flags'], \
                    o['ecefX'], o['ecefY'], o['ecefZ'], o['pAcc'], \
                    o['ecefVX'], o['ecefVY'], o['ecefVZ'], o['sAcc'], \
                    o['pDOP'], o['numSV'] = \
                    struct.unpack('!LlhBBlllLlllLHxBx', content)
                # scaling correction
                o['pDOP'] *= 0.01
            elif (data_id & 0x00F0) == 0x80:  # Time UTC
                o['iTOW'], o['tAcc'], o['nano'], o['year'], o['month'], \
                    o['day'], o['hour'], o['min'], o['sec'], o['valid'] = \
                    struct.unpack('!LLlHBBBBBB', content)
            elif (data_id & 0x00F0) == 0xA0:  # SV Info
                o['iTOW'], o['numCh'] = struct.unpack('!LBxxx', content[:8])
                channels = []
                ch = {}
                for i in range(o['numCh']):
                    ch['chn'], ch['svid'], ch['flags'], ch['quality'], \
                        ch['cno'], ch['elev'], ch['azim'], ch['prRes'] = \
                        struct.unpack('!BBBBBbhl', content[8+12*i:20+12*i])
                    channels.append(ch)
                o['channels'] = channels
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_SCR(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # ACC+GYR+MAG+Temperature
                o['accX'], o['accY'], o['accZ'], o['gyrX'], o['gyrY'], \
                    o['gyrZ'], o['magX'], o['magY'], o['magZ'], o['Temp'] = \
                    struct.unpack("!9Hh", content)
            elif (data_id & 0x00F0) == 0x20:  # Gyro Temperature
                o['tempGyrX'], o['tempGyrY'], o['tempGyrZ'] = \
                    struct.unpack("!hhh", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_analog_in(data_id, content, ffmt):  # deprecated
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Analog In 1
                o['analogIn1'], = struct.unpack("!H", content)
            elif (data_id & 0x00F0) == 0x20:  # Analog In 2
                o['analogIn2'], = struct.unpack("!H", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_magnetic(data_id, content, ffmt):
            o = {}
            o['frame'] = parse_frame(data_id)
            if (data_id & 0x00F0) == 0x20:  # Magnetic Field
                o['magX'], o['magY'], o['magZ'] = \
                    parse_floats(content, ffmt, 3)
                    # struct.unpack("!3"+ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_velocity(data_id, content, ffmt):
            o = {}
            o['frame'] = parse_frame(data_id)
            if (data_id & 0x00F0) == 0x10:  # Velocity XYZ
                o['velX'], o['velY'], o['velZ'] = \
                    parse_floats(content, ffmt, 3)
                    # struct.unpack("!3"+ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_status(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Status Byte
                o['StatusByte'], = struct.unpack("!B", content)
            elif (data_id & 0x00F0) == 0x20:  # Status Word
                o['StatusWord'], = struct.unpack("!L", content)
            elif (data_id & 0x00F0) == 0x40:  # RSSI  # deprecated
                o['RSSI'], = struct.unpack("!b", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_frame(data_id):
            frame = ''
            if (data_id & 0x000C) == XDIDataId.ENU:
                frame = 'ENU'
            elif (data_id & 0x000C) == XDIDataId.NED:
                frame = 'NED'
            elif (data_id & 0x000C) == XDIDataId.NWU:
                frame = 'NWU'
            return frame

        def parse_floats(content, ffmt, nb):
            if ffmt == "Float32":
                return struct.unpack("!"+"f"*nb, content)
            elif ffmt == "Float64":
                return struct.unpack("!"+"d"*nb, content)
            elif ffmt == "Fp1220":
                return [f / (2**20) for f in struct.unpack("!"+"i"*nb,content)]
            elif ffmt == "Fp1632":
                data = struct.unpack('!'+"Ih"*nb,content)
                out = []
                for i in range(len(data)//2):
                    out.append(data[2*i+1] + data[2*i] / (2**32))
                return out

        # data object
        output = {}
        while data:
            try:
                data_id, size = struct.unpack('!HB', data[:3])
                if (data_id & 0x0003) == XDIDataId.Float32:
                    float_format = 'Float32'
                elif (data_id & 0x0003) == XDIDataId.Fp1220:
                    float_format = 'Fp1220'
                elif (data_id & 0x0003) == XDIDataId.Fp1632:
                    float_format = 'Fp1632'
                elif (data_id & 0x0003) == XDIDataId.Float64:
                    float_format = 'Float64'

                content = data[3:3+size]
                data = data[3+size:]
                group = data_id & 0xF800
                ffmt = float_format
                if group == XDIType.TemperatureGroup:
                    output.setdefault('Temperature', {}).update(
                        parse_temperature(data_id, content, ffmt))
                elif group == XDIType.TimestampGroup:
                    output.setdefault('Timestamp', {}).update(
                        parse_timestamp(data_id, content, ffmt))
                elif group == XDIType.OrientationGroup:
                    output.setdefault('Orientation Data', {}).update(
                        parse_orientation_data(data_id, content, ffmt))
                elif group == XDIType.PressureGroup:
                    output.setdefault('Pressure', {}).update(
                        parse_pressure(data_id, content, ffmt))
                elif group == XDIType.AccelerationGroup:
                    output.setdefault('Acceleration', {}).update(
                        parse_acceleration(data_id, content, ffmt))
                elif group == XDIType.PositionGroup:
                    output.setdefault('Position', {}).update(
                        parse_position(data_id, content, ffmt))
                elif group == XDIType.GnssGroup:
                    output.setdefault('GNSS', {}).update(
                        parse_GNSS(data_id, content, ffmt))
                elif group == XDIType.AngularVelocityGroup:
                    output.setdefault('Angular Velocity', {}).update(
                        parse_angular_velocity(data_id, content, ffmt))
                elif group == XDIType.RawSensorGroup:
                    output.setdefault('SCR', {}).update(
                        parse_SCR(data_id, content, ffmt))
                elif group == XDIType.MagneticGroup:
                    output.setdefault('Magnetic', {}).update(
                        parse_magnetic(data_id, content, ffmt))
                elif group == XDIType.VelocityGroup:
                    output.setdefault('Velocity', {}).update(
                        parse_velocity(data_id, content, ffmt))
                elif group == XDIType.StatusGroup:
                    output.setdefault('Status', {}).update(
                        parse_status(data_id, content, ffmt))
                else:
                    raise MTException("unknown XDI group: 0x%04X." % group)
            except struct.error:
                raise MTException("couldn't parse MTData2 message.")
        return output

    def ForwardGnssData(self, data):
        self.write_ack(MID.ForwardGnssData, data)

    def ChangeBaudrate(self, baudrate):
        """Change the baudrate, reset the device and reopen communication."""
        brid = Baudrates.get_BRID(baudrate)
        self.SetBaudrate(brid)
        self.Reset()
        # self.device.flush()
        self.device.baudrate = baudrate
        # self.device.flush()
        time.sleep(0.01)
        self.read_msg()
        self.write_msg(MID.WakeUpAck)

    def Close(self):
        if self.device:
            self.device.close()


################################################################
# Auto detect port
################################################################
def find_devices(verbose=False):
    mtdev_list = []
    for port in glob.glob("/dev/tty*S*"):
        if verbose:
            print(f"Trying {port}")
        try:
            br = find_baudrate(port, verbose)
            if br:
                mtdev_list.append((port, br))
        except MTException:
            pass
    return mtdev_list


################################################################
# Auto detect baudrate
################################################################
def find_baudrate(port, verbose=False):
    baudrates = (115200, 460800, 921600, 2000000, 3686400, 4000000, 230400, 57600, 38400, 19200, 9600)
    for br in baudrates:
        if verbose:
            print(f"Trying {br} bd:")
            sys.stdout.flush()
        try:
            mt = MTDevice(port, br, verbose=verbose)
        except serial.SerialException:
            if verbose:
                print("fail: unable to open device.")
            raise MTException("unable to open %s" % port)
        try:
            mt.GoToConfig()
            mt.GoToMeasurement()
            if verbose:
                print("ok.")
            return br
        except MTException:
            if verbose:
                print("fail.")


################################################################
# Documentation for stand alone usage
################################################################
def usage():
        print("""MT device driver.
Usage:
    ./mtdevice.py [commands] [opts]

Commands:
    -h, --help
        Print this help and quit.
    -r, --reset
        Reset device to factory defaults.
    -a, --change-baudrate=NEW_BAUD
        Change baudrate from BAUD (see below) to NEW_BAUD.
    -c, --configure=OUTPUT
        Configure the device (see OUTPUT description below).
    -e, --echo
        Print MTData. It is the default if no other command is supplied.
    -i, --inspect
        Print current MT device configuration.
    -x, --xkf-scenario=ID
        Change the current XKF scenario.
    -v, --verbose
        Verbose output.

Generic options:
    -d, --device=DEV
        Serial interface of the device (default: /dev/ttyUSB0). If 'auto', then
        all serial ports are tested at all baudrates and the first
        suitable device is used.
    -b, --baudrate=BAUD
        Baudrate of serial interface (default: 115200). If 0, then all
        rates are tried until a suitable one is found.

Configuration option:
    OUTPUT
        The format is a sequence of "<group><type><frequency>?<format>?"
        separated by commas.
        The frequency and format are optional.
        The groups and types can be:
            t  temperature (max frequency: 1 Hz):
                tt  temperature
            i  timestamp (max frequency: 2000 Hz):
                iu  UTC time
                ip  packet counter
                if  sample time fine
                ic  sample time coarse
            o  orientation data (max frequency: 400 Hz):
                oq  quaternion
                om  rotation matrix
                oe  Euler angles
            b  pressure (max frequency: 50 Hz):
                bp  baro pressure
            a  acceleration (max frequency: 2000 Hz (see documentation)):
                ad  delta v
                aa  acceleration
                af  free acceleration
                ah  acceleration HR (max frequency 1000 Hz)
            p  position (max frequency: 400 Hz):
                pa  altitude ellipsoid
                pp  position ECEF
                pl  latitude longitude
            n  GNSS (max frequency: 4 Hz):
                np  GNSS PVT data
                ns  GNSS satellites info
                nu  Gnss Pvt Pulse
            w  angular velocity (max frequency: 2000 Hz (see documentation)):
                wr  rate of turn
                wd  delta q
                wh  rate of turn HR (max frequency 1000 Hz)
            r  Sensor Component Readout (max frequency: 2000 Hz):
                rr  ACC, GYR, MAG, temperature
                rt  Gyro temperatures
            m  Magnetic (max frequency: 100 Hz):
                mf  magnetic Field
            v  Velocity (max frequency: 400 Hz):
                vv  velocity XYZ
            s  Status (max frequency: 2000 Hz):
                sb  status byte
                sw  status word
                sd  DeviceId
                sl  LocationId
        Frequency is specified in decimal and is assumed to be the maximum
        frequency if it is omitted.
        Format is a combination of the precision for real valued numbers and
        coordinate system:
            precision:
                f  single precision floating point number (32-bit) (default)
                d  double precision floating point number (64-bit)
                a  fixed point 12.20 32-bit number (32-bit)
                b  fixed point 16.32 48-bit number (48-bit)
            coordinate system:
                e  East-North-Up (default)
                n  North-East-Down
                w  North-West-Up
        Examples:
            The default configuration for the MTi-1/10/100 IMUs can be
            specified either as:
                "wd,ad,mf,ip,if,sw"
            or
                "wd2000fe,ad2000fe,mf100fe,ip2000,if2000,sw2000"
            For getting quaternion orientation in float with sample time:
                "oq400fw,if2000"
            For longitude, latitude, altitude and orientation (on MTi-G-700):
                "pl400fe,pa400fe,oq400fe"
""")


################################################################
# Main function
################################################################
def main():
    # parse command line
    shopts = 'hra:c:eild:b:m:s:p:f:x:v'
    lopts = ['help', 'reset', 'change-baudrate=', 'configure=', 'echo',
             'inspect', 'device=', 'baudrate=',
             'output-mode=', 'output-settings=', 'period=',
             'deprecated-skip-factor=', 'xkf-scenario=', 'verbose']
    try:
        opts, args = getopt.gnu_getopt(sys.argv[1:], shopts, lopts)
    except getopt.GetoptError as e:
        print(e)
        usage()
        return 1
    # default values
    device = '/dev/ttyUSB0'
    baudrate = 115200
    mode = None
    settings = None
    period = None
    skipfactor = None
    new_baudrate = None
    new_xkf = None
    actions = []
    verbose = False
    # filling in arguments
    for o, a in opts:
        if o in ('-h', '--help'):
            usage()
            return
        elif o in ('-r', '--reset'):
            actions.append('reset')
        elif o in ('-a', '--change-baudrate'):
            try:
                new_baudrate = int(a)
            except ValueError:
                print("change-baudrate argument must be integer.")
                return 1
            actions.append('change-baudrate')
        elif o in ('-c', '--configure'):
            output_config = get_output_config(a)
            if output_config is None:
                return 1
            actions.append('configure')
        elif o in ('-e', '--echo'):
            actions.append('echo')
        elif o in ('-i', '--inspect'):
            actions.append('inspect')
        elif o in ('-x', '--xkf-scenario'):
            try:
                new_xkf = int(a)
            except ValueError:
                print("xkf-scenario argument must be integer.")
                return 1
            actions.append('xkf-scenario')
        elif o in ('-d', '--device'):
            device = a
        elif o in ('-b', '--baudrate'):
            try:
                baudrate = int(a)
            except ValueError:
                print("baudrate argument must be integer.")
                return 1
        elif o in ('-m', '--output-mode'):
            mode = get_mode(a)
            if mode is None:
                return 1
        elif o in ('-s', '--output-settings'):
            settings = get_settings(a)
            if settings is None:
                return 1
        elif o in ('-p', '--period'):
            try:
                period = int(a)
            except ValueError:
                print("period argument must be integer.")
                return 1
        elif o in ('-f', '--skip-factor'):
            try:
                skipfactor = int(a)
            except ValueError:
                print("skip-factor argument must be integer.")
                return 1
        elif o in ('-v', '--verbose'):
            verbose = True
    # if nothing else: echo
    if len(actions) == 0:
        actions.append('echo')
    try:
        if device == 'auto':
            devs = find_devices(verbose)
            if devs:
                print("Detected devices:", "".join(f'\n\{d} @ {p}' for d, p in devs))
                device, baudrate = devs[0]
                print("Using {device} @ {baudrate}")
            else:
                print("No suitable device found.")
                return 1
        # find baudrate
        if not baudrate:
            baudrate = find_baudrate(device, verbose)
        if not baudrate:
            print("No suitable baudrate found.")
            return 1
        # open device
        try:
            mt = MTDevice(device, baudrate, verbose=verbose)
        except serial.SerialException:
            raise MTException("unable to open %s" % device)
        # execute actions
        if 'inspect' in actions:
            inspect(mt, device, baudrate)
        if 'change-baudrate' in actions:
            print("Changing baudrate from {baudrate} to {new_baudrate}:")
            sys.stdout.flush()
            mt.ChangeBaudrate(new_baudrate)
            print(" Ok")  # should we test that it was actually ok?
        if 'reset' in actions:
            print("Restoring factory defaults")
            sys.stdout.flush()
            mt.RestoreFactoryDefaults()
            print(" Ok")  # should we test that it was actually ok?
        if 'configure' in actions:
            print("Changing output configuration")
            sys.stdout.flush()
            mt.SetOutputConfiguration(output_config)
            print(" Ok")  # should we test that it was actually ok?
        if 'xkf-scenario' in actions:
            print("Changing XKF scenario")
            sys.stdout.flush()
            mt.SetCurrentScenario(new_xkf)
            print(" Ok")
        if 'echo' in actions:
            # if (mode is None) or (settings is None):
            #     mode, settings, length = mt.auto_config()
            #     print mode, settings, length
            try:
                while True:
                    print(mt.read_measurement(mode, settings))
            except KeyboardInterrupt:
                pass
    except MTErrorMessage as e:
        print("MTErrorMessage:", e)
    except MTException as e:
        print("MTException:", e)


def inspect(mt, device, baudrate):
    """Inspection."""
    def config_fmt(config):
        """Hexadecimal configuration."""
        return '[%s]' % ', '.join('(0x%04X, %d)' % (mode, freq)
                                  for (mode, freq) in config)

    def hex_fmt(size=4):
        """Factory for hexadecimal representation formatter."""
        fmt = '0x%%0%dX' % (2*size)

        def f(value):
            """Hexadecimal representation."""
            # length of string is twice the size of the value (in bytes)
            return fmt % value
        return f

    def sync_fmt(settings):
        """Synchronization settings: N*12 bytes"""
        return '[%s]' % ', '.join(str(s) for s in settings)

    def try_message(m, f, formater=None, *args, **kwargs):
        print(f'  {m} ')
        try:
            if formater is not None:
                print(formater(f(*args, **kwargs)))
            else:
                print(f(*args, **kwargs))
        except MTErrorMessage as e:
            if e.code == 0x04:
                print('message unsupported by your device.')
            else:
                raise e
    print(f"Device: {device} at {baudrate} Bd:")
    try_message("device ID:", mt.GetDeviceID, hex_fmt(4))
    try_message("product code:", mt.GetProductCode)
    try_message("firmware revision:", mt.GetFirmwareRev)
    try_message("baudrate:", mt.GetBaudrate)
    try_message("error mode:", mt.GetErrorMode, hex_fmt(2))
    try_message("option flags:", mt.GetOptionFlags, hex_fmt(8))
    try_message("location ID:", mt.GetLocationID, hex_fmt(2))
    try_message("transmit delay:", mt.GetTransmitDelay)
    try_message("synchronization settings:", mt.GetSyncSettings, sync_fmt)
    try_message("output configuration (mark IV devices):",
                mt.GetOutputConfiguration, config_fmt)
    try_message("string output type:", mt.GetStringOutputType)
    try_message("alignment rotation sensor:", mt.GetAlignmentRotation,
                parameter=0)
    try_message("alignment rotation local:", mt.GetAlignmentRotation,
                parameter=1)
    try_message("extended output mode:", mt.GetExtOutputMode, hex_fmt(2))
    try_message("GPS coordinates (lat, lon, alt):", mt.GetLatLonAlt)
    try_message("available scenarios:", mt.GetAvailableScenarios)
    try_message("current scenario ID:", mt.GetCurrentScenario)
    try_message("UTC time:", mt.GetUTCTime)


def get_output_config(config_arg):
    """Parse the mark IV output configuration argument."""
    # code and max frequency
    code_dict = {
        # TemperatureGroup
        'tt': (XDIType.Temperature, 1),
        # TimestampGroup
        'iu': (XDIType.UtcTime, 2000),
        'ip': (XDIType.PacketCounter, 2000),
        'if': (XDIType.SampleTimeFine, 2000),
        'ic': (XDIType.SampleTimeCoarse, 2000),
        # OrientationGroup
        'oq': (XDIType.Quaternion, 400),
        'om': (XDIType.RotationMatrix, 400),
        'oe': (XDIType.EulerAngles, 400),
        # PressureGroup
        'bp': (XDIType.BaroPressure, 50),
        # AccelerationGroup
        'ad': (XDIType.DeltaV, 2000),
        'aa': (XDIType.Acceleration, 2000),
        'af': (XDIType.FreeAcceleration, 2000),
        'ah': (XDIType.AccelerationHR, 1000),
        # PositionGroup
        'pa': (XDIType.AltitudeEllipsoid, 400),
        'pp': (XDIType.PositionEcef, 400),
        'pl': (XDIType.LatLon, 400),
        # GnssGroup
        'np': (XDIType.GnssPvtData, 4),
        'ns': (XDIType.GnssSatInfo, 4),
        'ns': (XDIType.GnssPvtPulse, 4),
        # AngularVelocityGroup
        'wr': (XDIType.RateOfTurn, 2000),
        'wd': (XDIType.DeltaQ, 2000),
        'wh': (XDIType.RateOfTurnHR, 1000),
        # RawSensorGroup
        'rr': (XDIType.RawAccGyrMagTemp, 2000),
        'rt': (XDIType.RawGyroTemp, 2000),
        # MagneticGroup
        'mf': (XDIType.MagneticField, 100),
        # VelocityGroup
        'vv': (XDIType.VelocityXYZ, 400),
        # StatusGroup
        'sb': (XDIType.StatusByte, 2000),
        'sw': (XDIType.StatusWord, 2000),
        'sd': (XDIType.DeviceId, 2000),
        'sl': (XDIType.LocationId, 2000)
    }
    # format flags
    format_dict = {
        'f': XDIDataId.Float32,
        'd': XDIDataId.Float64,
        'a': XDIDataId.Fp1220,
        'b': XDIDataId.Fp1632,
        'e': XDIDataId.ENU,
        'n': XDIDataId.NED,
        'w': XDIDataId.NWU}
    config_re = re.compile('([a-z]{2})(\d+)?([fdabenw])?([fdabnew])?')
    output_configuration = []
    try:
        for item in config_arg.split(','):
            group, frequency, fmt1, fmt2 = config_re.findall(item.lower())[0]
            code, max_freq = code_dict[group]
            if fmt1 in format_dict:
                code |= format_dict[fmt1]
            if fmt2 in format_dict:
                code |= format_dict[fmt2]
            if frequency:
                frequency = min(max_freq, int(frequency))
            else:
                frequency = max_freq
            output_configuration.append((code, frequency))
        return output_configuration
    except (IndexError, KeyError):
        print(f'could not parse output specification {item}')
        return


def get_mode(arg):
    """Parse command line output-mode argument."""
    try:  # decimal
        mode = int(arg)
        return mode
    except ValueError:
        pass
    if arg[0] == '0':
        try:  # binary
            mode = int(arg, 2)
            return mode
        except ValueError:
            pass
        try:  # hexadecimal
            mode = int(arg, 16)
            return mode
        except ValueError:
            pass
    # string mode specification
    mode = 0
    for c in arg:
        if c == 't':
            mode |= OutputMode.Temp
        elif c == 'c':
            mode |= OutputMode.Calib
        elif c == 'o':
            mode |= OutputMode.Orient
        elif c == 'a':
            mode |= OutputMode.Auxiliary
        elif c == 'p':
            mode |= OutputMode.Position
        elif c == 'v':
            mode |= OutputMode.Velocity
        elif c == 's':
            mode |= OutputMode.Status
        elif c == 'g':
            mode |= OutputMode.RAWGPS
        elif c == 'r':
            mode |= OutputMode.RAW
        else:
            print(f"Unknown output-mode specifier: {c}")
            return
    return mode


def get_settings(arg):
    """Parse command line output-settings argument."""
    try:  # decimal
        settings = int(arg)
        return settings
    except ValueError:
        pass
    if arg[0] == '0':
        try:  # binary
            settings = int(arg, 2)
            return settings
        except ValueError:
            pass
        try:  # hexadecimal
            settings = int(arg, 16)
            return settings
        except ValueError:
            pass
    # strings settings specification
    timestamp = 0
    orient_mode = 0
    calib_mode = OutputSettings.CalibMode_Mask
    NED = 0
    for c in arg:
        if c == 't':
            timestamp = OutputSettings.Timestamp_SampleCnt
        elif c == 'n':
            timestamp = OutputSettings.Timestamp_None
        elif c == 'u':
            timestamp |= OutputSettings.Timestamp_UTCTime
        elif c == 'q':
            orient_mode = OutputSettings.OrientMode_Quaternion
        elif c == 'e':
            orient_mode = OutputSettings.OrientMode_Euler
        elif c == 'm':
            orient_mode = OutputSettings.OrientMode_Matrix
        elif c == 'A':
            calib_mode &= OutputSettings.CalibMode_Acc
        elif c == 'G':
            calib_mode &= OutputSettings.CalibMode_Gyr
        elif c == 'M':
            calib_mode &= OutputSettings.CalibMode_Mag
        elif c == 'i':
            calib_mode &= OutputSettings.AuxiliaryMode_NoAIN2
        elif c == 'j':
            calib_mode &= OutputSettings.AuxiliaryMode_NoAIN1
        elif c == 'N':
            NED = OutputSettings.Coordinates_NED
        else:
            print(f"Unknown output-settings specifier: {c}")
            return
    settings = timestamp | orient_mode | calib_mode | NED
    return settings


if __name__ == '__main__':
    main()
