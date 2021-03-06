// Generated by gencpp from file traxxas_node/AckermannMonitorMsg.msg
// DO NOT EDIT!


#ifndef TRAXXAS_NODE_MESSAGE_ACKERMANNMONITORMSG_H
#define TRAXXAS_NODE_MESSAGE_ACKERMANNMONITORMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace traxxas_node
{
template <class ContainerAllocator>
struct AckermannMonitorMsg_
{
  typedef AckermannMonitorMsg_<ContainerAllocator> Type;

  AckermannMonitorMsg_()
    : speed(0.0)
    , angle(0.0)  {
    }
  AckermannMonitorMsg_(const ContainerAllocator& _alloc)
    : speed(0.0)
    , angle(0.0)  {
    }



   typedef float _speed_type;
  _speed_type speed;

   typedef float _angle_type;
  _angle_type angle;




  typedef boost::shared_ptr< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> const> ConstPtr;

}; // struct AckermannMonitorMsg_

typedef ::traxxas_node::AckermannMonitorMsg_<std::allocator<void> > AckermannMonitorMsg;

typedef boost::shared_ptr< ::traxxas_node::AckermannMonitorMsg > AckermannMonitorMsgPtr;
typedef boost::shared_ptr< ::traxxas_node::AckermannMonitorMsg const> AckermannMonitorMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace traxxas_node

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'traxxas_node': ['/home/blue/catkin_ws/src/traxxas_node/msg', '/home/blue/catkin_ws/src/traxxas_node/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e18a4dfdb52fee48fc2e3bc9e7b74071";
  }

  static const char* value(const ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe18a4dfdb52fee48ULL;
  static const uint64_t static_value2 = 0xfc2e3bc9e7b74071ULL;
};

template<class ContainerAllocator>
struct DataType< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "traxxas_node/AckermannMonitorMsg";
  }

  static const char* value(const ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 speed\n\
float32 angle\n\
";
  }

  static const char* value(const ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.speed);
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct AckermannMonitorMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::traxxas_node::AckermannMonitorMsg_<ContainerAllocator>& v)
  {
    s << indent << "speed: ";
    Printer<float>::stream(s, indent + "  ", v.speed);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRAXXAS_NODE_MESSAGE_ACKERMANNMONITORMSG_H
