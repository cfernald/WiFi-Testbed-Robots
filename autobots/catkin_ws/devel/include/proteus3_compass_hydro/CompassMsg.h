// Generated by gencpp from file proteus3_compass_hydro/CompassMsg.msg
// DO NOT EDIT!


#ifndef PROTEUS3_COMPASS_HYDRO_MESSAGE_COMPASSMSG_H
#define PROTEUS3_COMPASS_HYDRO_MESSAGE_COMPASSMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace proteus3_compass_hydro
{
template <class ContainerAllocator>
struct CompassMsg_
{
  typedef CompassMsg_<ContainerAllocator> Type;

  CompassMsg_()
    : heading(0.0)
    , pitch(0.0)
    , roll(0.0)  {
    }
  CompassMsg_(const ContainerAllocator& _alloc)
    : heading(0.0)
    , pitch(0.0)
    , roll(0.0)  {
    }



   typedef float _heading_type;
  _heading_type heading;

   typedef float _pitch_type;
  _pitch_type pitch;

   typedef float _roll_type;
  _roll_type roll;




  typedef boost::shared_ptr< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> const> ConstPtr;

}; // struct CompassMsg_

typedef ::proteus3_compass_hydro::CompassMsg_<std::allocator<void> > CompassMsg;

typedef boost::shared_ptr< ::proteus3_compass_hydro::CompassMsg > CompassMsgPtr;
typedef boost::shared_ptr< ::proteus3_compass_hydro::CompassMsg const> CompassMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace proteus3_compass_hydro

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'proteus3_compass_hydro': ['/home/blue/catkin_ws/src/proteus3_compass_hydro/msg', '/home/blue/catkin_ws/src/proteus3_compass_hydro/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a0a24f94640b168577ac5c59871cb550";
  }

  static const char* value(const ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa0a24f94640b1685ULL;
  static const uint64_t static_value2 = 0x77ac5c59871cb550ULL;
};

template<class ContainerAllocator>
struct DataType< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "proteus3_compass_hydro/CompassMsg";
  }

  static const char* value(const ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 heading\n\
float32 pitch\n\
float32 roll\n\
";
  }

  static const char* value(const ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.heading);
      stream.next(m.pitch);
      stream.next(m.roll);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct CompassMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::proteus3_compass_hydro::CompassMsg_<ContainerAllocator>& v)
  {
    s << indent << "heading: ";
    Printer<float>::stream(s, indent + "  ", v.heading);
    s << indent << "pitch: ";
    Printer<float>::stream(s, indent + "  ", v.pitch);
    s << indent << "roll: ";
    Printer<float>::stream(s, indent + "  ", v.roll);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PROTEUS3_COMPASS_HYDRO_MESSAGE_COMPASSMSG_H
