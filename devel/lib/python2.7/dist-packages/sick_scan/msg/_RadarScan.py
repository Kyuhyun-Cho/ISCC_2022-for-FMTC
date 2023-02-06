# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from sick_scan/RadarScan.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import geometry_msgs.msg
import sensor_msgs.msg
import sick_scan.msg
import std_msgs.msg

class RadarScan(genpy.Message):
  _md5sum = "db9483dce93673bbf6148baa3d967315"
  _type = "sick_scan/RadarScan"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """#
# The message is divided into three parts:
# 1. Header: containing information about general radar parameters
# 2. RawTargets: List of targets containing maximum values in the range-doppler-matrix - used for tracking
# 3. Objects: List of objects generated by the internal tracking algorithm - based on raw targets
Header header
RadarPreHeader radarPreHeader
sensor_msgs/PointCloud2 targets
sick_scan/RadarObject[] objects
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sick_scan/RadarPreHeader
# Version information for this whole structure (MeasurementData)
uint16 uiVersionNo
sick_scan/RadarPreHeaderDeviceBlock radarPreHeaderDeviceBlock
sick_scan/RadarPreHeaderStatusBlock radarPreHeaderStatusBlock
sick_scan/RadarPreHeaderMeasurementParam1Block radarPreHeaderMeasurementParam1Block
sick_scan/RadarPreHeaderEncoderBlock[] radarPreHeaderArrayEncoderBlock


================================================================================
MSG: sick_scan/RadarPreHeaderDeviceBlock
#
#
#
#

# Logical number of the device"
uint32 uiIdent
# Serial number of the device
uint32 udiSerialNo
# State of the device
bool bDeviceError
# Contamination Warning
bool bContaminationWarning
# Contamination Error
bool bContaminationError
================================================================================
MSG: sick_scan/RadarPreHeaderStatusBlock
#
#
#
uint32 uiTelegramCount # telegram number
uint32 uiCycleCount # "scan number"
uint32 udiSystemCountScan # system time since power on of scan gen. [us]
uint32 udiSystemCountTransmit # system time since power on of scan transmission [us]
uint16 uiInputs # state of digital inputs
uint16 uiOutputs # state of digital outputs

================================================================================
MSG: sick_scan/RadarPreHeaderMeasurementParam1Block
uint32 uiCycleDuration # Time in microseconds to detect this items
uint32 uiNoiseLevel # [1/100dB]

================================================================================
MSG: sick_scan/RadarPreHeaderEncoderBlock
# Array with connected encoder sensors
uint32 udiEncoderPos  # encoder position [inc]
int16 iEncoderSpeed   # encoder speed [inc/mm]
================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: sick_scan/RadarObject
int32 id

time tracking_time                          # since when the object is tracked
time last_seen

geometry_msgs/TwistWithCovariance velocity

geometry_msgs/Pose bounding_box_center
geometry_msgs/Vector3 bounding_box_size

geometry_msgs/PoseWithCovariance object_box_center
geometry_msgs/Vector3 object_box_size

geometry_msgs/Point[] contour_points

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
"""
  __slots__ = ['header','radarPreHeader','targets','objects']
  _slot_types = ['std_msgs/Header','sick_scan/RadarPreHeader','sensor_msgs/PointCloud2','sick_scan/RadarObject[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,radarPreHeader,targets,objects

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(RadarScan, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.radarPreHeader is None:
        self.radarPreHeader = sick_scan.msg.RadarPreHeader()
      if self.targets is None:
        self.targets = sensor_msgs.msg.PointCloud2()
      if self.objects is None:
        self.objects = []
    else:
      self.header = std_msgs.msg.Header()
      self.radarPreHeader = sick_scan.msg.RadarPreHeader()
      self.targets = sensor_msgs.msg.PointCloud2()
      self.objects = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_H2I3B4I2H2I().pack(_x.radarPreHeader.uiVersionNo, _x.radarPreHeader.radarPreHeaderDeviceBlock.uiIdent, _x.radarPreHeader.radarPreHeaderDeviceBlock.udiSerialNo, _x.radarPreHeader.radarPreHeaderDeviceBlock.bDeviceError, _x.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationWarning, _x.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationError, _x.radarPreHeader.radarPreHeaderStatusBlock.uiTelegramCount, _x.radarPreHeader.radarPreHeaderStatusBlock.uiCycleCount, _x.radarPreHeader.radarPreHeaderStatusBlock.udiSystemCountScan, _x.radarPreHeader.radarPreHeaderStatusBlock.udiSystemCountTransmit, _x.radarPreHeader.radarPreHeaderStatusBlock.uiInputs, _x.radarPreHeader.radarPreHeaderStatusBlock.uiOutputs, _x.radarPreHeader.radarPreHeaderMeasurementParam1Block.uiCycleDuration, _x.radarPreHeader.radarPreHeaderMeasurementParam1Block.uiNoiseLevel))
      length = len(self.radarPreHeader.radarPreHeaderArrayEncoderBlock)
      buff.write(_struct_I.pack(length))
      for val1 in self.radarPreHeader.radarPreHeaderArrayEncoderBlock:
        _x = val1
        buff.write(_get_struct_Ih().pack(_x.udiEncoderPos, _x.iEncoderSpeed))
      _x = self
      buff.write(_get_struct_3I().pack(_x.targets.header.seq, _x.targets.header.stamp.secs, _x.targets.header.stamp.nsecs))
      _x = self.targets.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.targets.height, _x.targets.width))
      length = len(self.targets.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.targets.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_IBI().pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_get_struct_B2I().pack(_x.targets.is_bigendian, _x.targets.point_step, _x.targets.row_step))
      _x = self.targets.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.Struct('<I%sB'%length).pack(length, *_x))
      else:
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.targets.is_dense
      buff.write(_get_struct_B().pack(_x))
      length = len(self.objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.objects:
        _x = val1.id
        buff.write(_get_struct_i().pack(_x))
        _v1 = val1.tracking_time
        _x = _v1
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _v2 = val1.last_seen
        _x = _v2
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _v3 = val1.velocity
        _v4 = _v3.twist
        _v5 = _v4.linear
        _x = _v5
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v6 = _v4.angular
        _x = _v6
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        buff.write(_get_struct_36d().pack(*_v3.covariance))
        _v7 = val1.bounding_box_center
        _v8 = _v7.position
        _x = _v8
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v9 = _v7.orientation
        _x = _v9
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
        _v10 = val1.bounding_box_size
        _x = _v10
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v11 = val1.object_box_center
        _v12 = _v11.pose
        _v13 = _v12.position
        _x = _v13
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v14 = _v12.orientation
        _x = _v14
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_get_struct_36d().pack(*_v11.covariance))
        _v15 = val1.object_box_size
        _x = _v15
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        length = len(val1.contour_points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.contour_points:
          _x = val2
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.radarPreHeader is None:
        self.radarPreHeader = sick_scan.msg.RadarPreHeader()
      if self.targets is None:
        self.targets = sensor_msgs.msg.PointCloud2()
      if self.objects is None:
        self.objects = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 41
      (_x.radarPreHeader.uiVersionNo, _x.radarPreHeader.radarPreHeaderDeviceBlock.uiIdent, _x.radarPreHeader.radarPreHeaderDeviceBlock.udiSerialNo, _x.radarPreHeader.radarPreHeaderDeviceBlock.bDeviceError, _x.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationWarning, _x.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationError, _x.radarPreHeader.radarPreHeaderStatusBlock.uiTelegramCount, _x.radarPreHeader.radarPreHeaderStatusBlock.uiCycleCount, _x.radarPreHeader.radarPreHeaderStatusBlock.udiSystemCountScan, _x.radarPreHeader.radarPreHeaderStatusBlock.udiSystemCountTransmit, _x.radarPreHeader.radarPreHeaderStatusBlock.uiInputs, _x.radarPreHeader.radarPreHeaderStatusBlock.uiOutputs, _x.radarPreHeader.radarPreHeaderMeasurementParam1Block.uiCycleDuration, _x.radarPreHeader.radarPreHeaderMeasurementParam1Block.uiNoiseLevel,) = _get_struct_H2I3B4I2H2I().unpack(str[start:end])
      self.radarPreHeader.radarPreHeaderDeviceBlock.bDeviceError = bool(self.radarPreHeader.radarPreHeaderDeviceBlock.bDeviceError)
      self.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationWarning = bool(self.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationWarning)
      self.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationError = bool(self.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationError)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.radarPreHeader.radarPreHeaderArrayEncoderBlock = []
      for i in range(0, length):
        val1 = sick_scan.msg.RadarPreHeaderEncoderBlock()
        _x = val1
        start = end
        end += 6
        (_x.udiEncoderPos, _x.iEncoderSpeed,) = _get_struct_Ih().unpack(str[start:end])
        self.radarPreHeader.radarPreHeaderArrayEncoderBlock.append(val1)
      _x = self
      start = end
      end += 12
      (_x.targets.header.seq, _x.targets.header.stamp.secs, _x.targets.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.targets.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.targets.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.targets.height, _x.targets.width,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.targets.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _get_struct_IBI().unpack(str[start:end])
        self.targets.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.targets.is_bigendian, _x.targets.point_step, _x.targets.row_step,) = _get_struct_B2I().unpack(str[start:end])
      self.targets.is_bigendian = bool(self.targets.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.targets.data = str[start:end]
      start = end
      end += 1
      (self.targets.is_dense,) = _get_struct_B().unpack(str[start:end])
      self.targets.is_dense = bool(self.targets.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.objects = []
      for i in range(0, length):
        val1 = sick_scan.msg.RadarObject()
        start = end
        end += 4
        (val1.id,) = _get_struct_i().unpack(str[start:end])
        _v16 = val1.tracking_time
        _x = _v16
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        _v17 = val1.last_seen
        _x = _v17
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        _v18 = val1.velocity
        _v19 = _v18.twist
        _v20 = _v19.linear
        _x = _v20
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v21 = _v19.angular
        _x = _v21
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        start = end
        end += 288
        _v18.covariance = _get_struct_36d().unpack(str[start:end])
        _v22 = val1.bounding_box_center
        _v23 = _v22.position
        _x = _v23
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v24 = _v22.orientation
        _x = _v24
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        _v25 = val1.bounding_box_size
        _x = _v25
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v26 = val1.object_box_center
        _v27 = _v26.pose
        _v28 = _v27.position
        _x = _v28
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v29 = _v27.orientation
        _x = _v29
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        start = end
        end += 288
        _v26.covariance = _get_struct_36d().unpack(str[start:end])
        _v30 = val1.object_box_size
        _x = _v30
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.contour_points = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Point()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          val1.contour_points.append(val2)
        self.objects.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_H2I3B4I2H2I().pack(_x.radarPreHeader.uiVersionNo, _x.radarPreHeader.radarPreHeaderDeviceBlock.uiIdent, _x.radarPreHeader.radarPreHeaderDeviceBlock.udiSerialNo, _x.radarPreHeader.radarPreHeaderDeviceBlock.bDeviceError, _x.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationWarning, _x.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationError, _x.radarPreHeader.radarPreHeaderStatusBlock.uiTelegramCount, _x.radarPreHeader.radarPreHeaderStatusBlock.uiCycleCount, _x.radarPreHeader.radarPreHeaderStatusBlock.udiSystemCountScan, _x.radarPreHeader.radarPreHeaderStatusBlock.udiSystemCountTransmit, _x.radarPreHeader.radarPreHeaderStatusBlock.uiInputs, _x.radarPreHeader.radarPreHeaderStatusBlock.uiOutputs, _x.radarPreHeader.radarPreHeaderMeasurementParam1Block.uiCycleDuration, _x.radarPreHeader.radarPreHeaderMeasurementParam1Block.uiNoiseLevel))
      length = len(self.radarPreHeader.radarPreHeaderArrayEncoderBlock)
      buff.write(_struct_I.pack(length))
      for val1 in self.radarPreHeader.radarPreHeaderArrayEncoderBlock:
        _x = val1
        buff.write(_get_struct_Ih().pack(_x.udiEncoderPos, _x.iEncoderSpeed))
      _x = self
      buff.write(_get_struct_3I().pack(_x.targets.header.seq, _x.targets.header.stamp.secs, _x.targets.header.stamp.nsecs))
      _x = self.targets.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.targets.height, _x.targets.width))
      length = len(self.targets.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.targets.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_IBI().pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_get_struct_B2I().pack(_x.targets.is_bigendian, _x.targets.point_step, _x.targets.row_step))
      _x = self.targets.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.Struct('<I%sB'%length).pack(length, *_x))
      else:
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.targets.is_dense
      buff.write(_get_struct_B().pack(_x))
      length = len(self.objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.objects:
        _x = val1.id
        buff.write(_get_struct_i().pack(_x))
        _v31 = val1.tracking_time
        _x = _v31
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _v32 = val1.last_seen
        _x = _v32
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _v33 = val1.velocity
        _v34 = _v33.twist
        _v35 = _v34.linear
        _x = _v35
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v36 = _v34.angular
        _x = _v36
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        buff.write(_v33.covariance.tostring())
        _v37 = val1.bounding_box_center
        _v38 = _v37.position
        _x = _v38
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v39 = _v37.orientation
        _x = _v39
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
        _v40 = val1.bounding_box_size
        _x = _v40
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v41 = val1.object_box_center
        _v42 = _v41.pose
        _v43 = _v42.position
        _x = _v43
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v44 = _v42.orientation
        _x = _v44
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_v41.covariance.tostring())
        _v45 = val1.object_box_size
        _x = _v45
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        length = len(val1.contour_points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.contour_points:
          _x = val2
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.radarPreHeader is None:
        self.radarPreHeader = sick_scan.msg.RadarPreHeader()
      if self.targets is None:
        self.targets = sensor_msgs.msg.PointCloud2()
      if self.objects is None:
        self.objects = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 41
      (_x.radarPreHeader.uiVersionNo, _x.radarPreHeader.radarPreHeaderDeviceBlock.uiIdent, _x.radarPreHeader.radarPreHeaderDeviceBlock.udiSerialNo, _x.radarPreHeader.radarPreHeaderDeviceBlock.bDeviceError, _x.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationWarning, _x.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationError, _x.radarPreHeader.radarPreHeaderStatusBlock.uiTelegramCount, _x.radarPreHeader.radarPreHeaderStatusBlock.uiCycleCount, _x.radarPreHeader.radarPreHeaderStatusBlock.udiSystemCountScan, _x.radarPreHeader.radarPreHeaderStatusBlock.udiSystemCountTransmit, _x.radarPreHeader.radarPreHeaderStatusBlock.uiInputs, _x.radarPreHeader.radarPreHeaderStatusBlock.uiOutputs, _x.radarPreHeader.radarPreHeaderMeasurementParam1Block.uiCycleDuration, _x.radarPreHeader.radarPreHeaderMeasurementParam1Block.uiNoiseLevel,) = _get_struct_H2I3B4I2H2I().unpack(str[start:end])
      self.radarPreHeader.radarPreHeaderDeviceBlock.bDeviceError = bool(self.radarPreHeader.radarPreHeaderDeviceBlock.bDeviceError)
      self.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationWarning = bool(self.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationWarning)
      self.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationError = bool(self.radarPreHeader.radarPreHeaderDeviceBlock.bContaminationError)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.radarPreHeader.radarPreHeaderArrayEncoderBlock = []
      for i in range(0, length):
        val1 = sick_scan.msg.RadarPreHeaderEncoderBlock()
        _x = val1
        start = end
        end += 6
        (_x.udiEncoderPos, _x.iEncoderSpeed,) = _get_struct_Ih().unpack(str[start:end])
        self.radarPreHeader.radarPreHeaderArrayEncoderBlock.append(val1)
      _x = self
      start = end
      end += 12
      (_x.targets.header.seq, _x.targets.header.stamp.secs, _x.targets.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.targets.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.targets.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.targets.height, _x.targets.width,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.targets.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _get_struct_IBI().unpack(str[start:end])
        self.targets.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.targets.is_bigendian, _x.targets.point_step, _x.targets.row_step,) = _get_struct_B2I().unpack(str[start:end])
      self.targets.is_bigendian = bool(self.targets.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.targets.data = str[start:end]
      start = end
      end += 1
      (self.targets.is_dense,) = _get_struct_B().unpack(str[start:end])
      self.targets.is_dense = bool(self.targets.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.objects = []
      for i in range(0, length):
        val1 = sick_scan.msg.RadarObject()
        start = end
        end += 4
        (val1.id,) = _get_struct_i().unpack(str[start:end])
        _v46 = val1.tracking_time
        _x = _v46
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        _v47 = val1.last_seen
        _x = _v47
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        _v48 = val1.velocity
        _v49 = _v48.twist
        _v50 = _v49.linear
        _x = _v50
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v51 = _v49.angular
        _x = _v51
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        start = end
        end += 288
        _v48.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
        _v52 = val1.bounding_box_center
        _v53 = _v52.position
        _x = _v53
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v54 = _v52.orientation
        _x = _v54
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        _v55 = val1.bounding_box_size
        _x = _v55
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v56 = val1.object_box_center
        _v57 = _v56.pose
        _v58 = _v57.position
        _x = _v58
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v59 = _v57.orientation
        _x = _v59
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        start = end
        end += 288
        _v56.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
        _v60 = val1.object_box_size
        _x = _v60
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.contour_points = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Point()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          val1.contour_points.append(val2)
        self.objects.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_36d = None
def _get_struct_36d():
    global _struct_36d
    if _struct_36d is None:
        _struct_36d = struct.Struct("<36d")
    return _struct_36d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
_struct_B2I = None
def _get_struct_B2I():
    global _struct_B2I
    if _struct_B2I is None:
        _struct_B2I = struct.Struct("<B2I")
    return _struct_B2I
_struct_H2I3B4I2H2I = None
def _get_struct_H2I3B4I2H2I():
    global _struct_H2I3B4I2H2I
    if _struct_H2I3B4I2H2I is None:
        _struct_H2I3B4I2H2I = struct.Struct("<H2I3B4I2H2I")
    return _struct_H2I3B4I2H2I
_struct_IBI = None
def _get_struct_IBI():
    global _struct_IBI
    if _struct_IBI is None:
        _struct_IBI = struct.Struct("<IBI")
    return _struct_IBI
_struct_Ih = None
def _get_struct_Ih():
    global _struct_Ih
    if _struct_Ih is None:
        _struct_Ih = struct.Struct("<Ih")
    return _struct_Ih
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
