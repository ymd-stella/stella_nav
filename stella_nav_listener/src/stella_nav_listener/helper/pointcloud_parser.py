from sensor_msgs.msg import PointField
import numpy as np
import struct


class PointCloudParser(object):
    _DATATYPES = {}
    _DATATYPES[PointField.INT8]    = ('b', 1)
    _DATATYPES[PointField.UINT8]   = ('B', 1)
    _DATATYPES[PointField.INT16]   = ('h', 2)
    _DATATYPES[PointField.UINT16]  = ('H', 2)
    _DATATYPES[PointField.INT32]   = ('i', 4)
    _DATATYPES[PointField.UINT32]  = ('I', 4)
    _DATATYPES[PointField.FLOAT32] = ('f', 4)
    _DATATYPES[PointField.FLOAT64] = ('d', 8)
    _NP_TYPES = {
        np.dtype('uint8')   :   (PointField.UINT8,  1),
        np.dtype('int8')    :   (PointField.INT8,   1),
        np.dtype('uint16')  :   (PointField.UINT16, 2),
        np.dtype('int16')   :   (PointField.INT16,  2),
        np.dtype('uint32')  :   (PointField.UINT32, 4),
        np.dtype('int32')   :   (PointField.INT32,  4),
        np.dtype('float32') :   (PointField.FLOAT32,4),
        np.dtype('float64') :   (PointField.FLOAT64,8)
    }

    def __init__(self, **kwargs):
        self._fields = None
        self._x_idx = None
        self._y_idx = None
        self._fmt = ""

    def _get_struct_fmt(self, msg, field_names=None):
        fmt = ">" if msg.is_bigendian else "<"
        offset = 0
        for field in (f for f in self._fields if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += "x" * (field.offset - offset)
                offset = field.offset
            datatype_fmt, datatype_length = PointCloudParser._DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length
        return fmt

    def parse(self, msg):
        # TODO: handling z
        if not self._fmt:
            self._fields = sorted(msg.fields, key=lambda f: f.offset)
            self._fmt = self._get_struct_fmt(msg)
            lx = [f.name == "x" for f in self._fields]
            ly = [f.name == "y" for f in self._fields]
            self._x_idx = lx.index(True)
            self._y_idx = ly.index(True)
        fmt_full = ">" if msg.is_bigendian else "<" + self._fmt.strip("<>") * msg.width * msg.height
        unpacker = struct.Struct(fmt_full)
        unpacked = np.asarray(unpacker.unpack_from(msg.data))
        full_data = zip(*[iter(unpacked)]*len(self._fields))
        w = np.vstack([np.array((data[self._x_idx], data[self._y_idx])).reshape(1, 2) for data in full_data])
        return w
