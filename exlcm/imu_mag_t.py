"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class imu_mag_t(object):
    __slots__ = ["timestamp", "frame", "data"]

    def __init__(self):
        self.timestamp = 0
        self.frame = ""
        self.data = [ 0.0 for dim0 in range(3) ]

    def encode(self):
        buf = BytesIO()
        buf.write(imu_mag_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.timestamp))
        __frame_encoded = self.frame.encode('utf-8')
        buf.write(struct.pack('>I', len(__frame_encoded)+1))
        buf.write(__frame_encoded)
        buf.write(b"\0")
        buf.write(struct.pack('>3f', *self.data[:3]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != imu_mag_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return imu_mag_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = imu_mag_t()
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        __frame_len = struct.unpack('>I', buf.read(4))[0]
        self.frame = buf.read(__frame_len)[:-1].decode('utf-8', 'replace')
        self.data = struct.unpack('>3f', buf.read(12))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if imu_mag_t in parents: return 0
        tmphash = (0x2286256e3ebaf9da) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if imu_mag_t._packed_fingerprint is None:
            imu_mag_t._packed_fingerprint = struct.pack(">Q", imu_mag_t._get_hash_recursive([]))
        return imu_mag_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

