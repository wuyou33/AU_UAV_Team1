/* Auto-generated by genmsg_cpp for file /home/hosea/ros_workspace/AU_UAV_ROS/msg/Command.msg */
#ifndef AU_UAV_ROS_MESSAGE_COMMAND_H
#define AU_UAV_ROS_MESSAGE_COMMAND_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace AU_UAV_ROS
{
template <class ContainerAllocator>
struct Command_ {
  typedef Command_<ContainerAllocator> Type;

  Command_()
  : commandHeader()
  , planeID(0)
  , latitude(0.0)
  , longitude(0.0)
  , altitude(0.0)
  {
  }

  Command_(const ContainerAllocator& _alloc)
  : commandHeader(_alloc)
  , planeID(0)
  , latitude(0.0)
  , longitude(0.0)
  , altitude(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _commandHeader_type;
   ::std_msgs::Header_<ContainerAllocator>  commandHeader;

  typedef int16_t _planeID_type;
  int16_t planeID;

  typedef double _latitude_type;
  double latitude;

  typedef double _longitude_type;
  double longitude;

  typedef double _altitude_type;
  double altitude;


  typedef boost::shared_ptr< ::AU_UAV_ROS::Command_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::AU_UAV_ROS::Command_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Command
typedef  ::AU_UAV_ROS::Command_<std::allocator<void> > Command;

typedef boost::shared_ptr< ::AU_UAV_ROS::Command> CommandPtr;
typedef boost::shared_ptr< ::AU_UAV_ROS::Command const> CommandConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::AU_UAV_ROS::Command_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::AU_UAV_ROS::Command_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace AU_UAV_ROS

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::AU_UAV_ROS::Command_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::AU_UAV_ROS::Command_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::AU_UAV_ROS::Command_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c53c94d81a0a5526e6ff4317b73721aa";
  }

  static const char* value(const  ::AU_UAV_ROS::Command_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc53c94d81a0a5526ULL;
  static const uint64_t static_value2 = 0xe6ff4317b73721aaULL;
};

template<class ContainerAllocator>
struct DataType< ::AU_UAV_ROS::Command_<ContainerAllocator> > {
  static const char* value() 
  {
    return "AU_UAV_ROS/Command";
  }

  static const char* value(const  ::AU_UAV_ROS::Command_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::AU_UAV_ROS::Command_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header commandHeader\n\
int16 planeID\n\
float64 latitude\n\
float64 longitude\n\
float64 altitude\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::AU_UAV_ROS::Command_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::AU_UAV_ROS::Command_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.commandHeader);
    stream.next(m.planeID);
    stream.next(m.latitude);
    stream.next(m.longitude);
    stream.next(m.altitude);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Command_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::AU_UAV_ROS::Command_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::AU_UAV_ROS::Command_<ContainerAllocator> & v) 
  {
    s << indent << "commandHeader: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.commandHeader);
    s << indent << "planeID: ";
    Printer<int16_t>::stream(s, indent + "  ", v.planeID);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "altitude: ";
    Printer<double>::stream(s, indent + "  ", v.altitude);
  }
};


} // namespace message_operations
} // namespace ros

#endif // AU_UAV_ROS_MESSAGE_COMMAND_H

