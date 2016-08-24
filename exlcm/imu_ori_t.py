"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class imu_ori_t(object):
    __slots__ = ["timestamp", "data"]

    def __init__(self):
        self.timestamp = 0
        self.data = [ 0.0 for dim0 in range(4) ]

    def encode(self):
        buf = BytesIO()
        buf.write(imu_ori_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.timestamp))
        buf.write(struct.pack('>4f', *self.data[:4]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != imu_ori_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return imu_ori_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = imu_ori_t()
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        self.data = struct.unpack('>4f', buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if imu_ori_t in parents: return 0
        tmphash = (0xd66821b847637971) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if imu_ori_t._packed_fingerprint is None:
            imu_ori_t._packed_fingerprint = struct.pack(">Q", imu_ori_t._get_hash_recursive([]))
        return imu_ori_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

