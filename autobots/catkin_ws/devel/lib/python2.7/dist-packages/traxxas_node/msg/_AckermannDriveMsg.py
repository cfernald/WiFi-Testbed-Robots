# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from traxxas_node/AckermannDriveMsg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class AckermannDriveMsg(genpy.Message):
  _md5sum = "3512e91b48d69674a0e86fadf1ea8231"
  _type = "traxxas_node/AckermannDriveMsg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# Average angle of the front wheels in radians (left is positive).
# This angle is the true turning angle for bicycles and motorcycles. 
# It is an approximation for a car, computed by taking the average 
# of the angles of the 2 front wheels.
float32 steering_angle

# Desired rate of change of the steering angle in radians/second.
float32 steering_angle_velocity

# Speed of the center of the rear axle in meters/second (reverse is negative)
float32 speed

# Desired acceleration of the center of the rear axle in meters/second^2
float32 acceleration

# Desired jerk of the center of the rear axle in meters/second^3
float32 jerk

"""
  __slots__ = ['steering_angle','steering_angle_velocity','speed','acceleration','jerk']
  _slot_types = ['float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       steering_angle,steering_angle_velocity,speed,acceleration,jerk

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AckermannDriveMsg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.steering_angle is None:
        self.steering_angle = 0.
      if self.steering_angle_velocity is None:
        self.steering_angle_velocity = 0.
      if self.speed is None:
        self.speed = 0.
      if self.acceleration is None:
        self.acceleration = 0.
      if self.jerk is None:
        self.jerk = 0.
    else:
      self.steering_angle = 0.
      self.steering_angle_velocity = 0.
      self.speed = 0.
      self.acceleration = 0.
      self.jerk = 0.

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
      buff.write(_struct_5f.pack(_x.steering_angle, _x.steering_angle_velocity, _x.speed, _x.acceleration, _x.jerk))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 20
      (_x.steering_angle, _x.steering_angle_velocity, _x.speed, _x.acceleration, _x.jerk,) = _struct_5f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_5f.pack(_x.steering_angle, _x.steering_angle_velocity, _x.speed, _x.acceleration, _x.jerk))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 20
      (_x.steering_angle, _x.steering_angle_velocity, _x.speed, _x.acceleration, _x.jerk,) = _struct_5f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_5f = struct.Struct("<5f")
