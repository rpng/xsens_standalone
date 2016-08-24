"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class imu_raw_t(object):
    __slots__ = ["timestamp", "linear_accel", "angular_vel"]

    def __init__(self):
        self.timestamp = 0
        self.linear_accel = [ 0.0 for dim0 in range(3) ]
        self.angular_vel = [ 0.0 for dim0 in range(3) ]

    def encode(self):
        buf = BytesIO()
        buf.write(imu_raw_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.timestamp))
        buf.write(struct.pack('>3f', *self.linear_accel[:3]))
        buf.write(struct.pack('>3f', *self.angular_vel[:3]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != imu_raw_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return imu_raw_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = imu_raw_t()
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        self.linear_accel = struct.unpack('>3f', buf.read(12))
        self.angular_vel = struct.unpack('>3f', buf.read(12))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if imu_raw_t in parents: return 0
        tmphash = (0x48414edf0d61592) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if imu_raw_t._packed_fingerprint is None:
            imu_raw_t._packed_fingerprint = struct.pack(">Q", imu_raw_t._get_hash_recursive([]))
        return imu_raw_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

