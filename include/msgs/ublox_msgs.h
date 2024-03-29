// Generated by gencpp from file clothoid_msgs/ublox_msgs.msg
// DO NOT EDIT!


#ifndef CLOTHOID_MSGS_MESSAGE_UBLOX_MSGS_H
#define CLOTHOID_MSGS_MESSAGE_UBLOX_MSGS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace clothoid_msgs
{
template <class ContainerAllocator>
struct ublox_msgs_
{
  typedef ublox_msgs_<ContainerAllocator> Type;

  ublox_msgs_()
    : header()
    , lon(0)
    , lat(0)
    , height(0)
    , hour(0)
    , min(0)
    , sec(0)
    , gpsFix(0)
    , gspeed(0)
    , headmot(0)
    , headveh(0)
    , gyro_x(0)
    , gyro_y(0)
    , gyro_z(0)
    , acc_x(0)
    , acc_y(0)
    , acc_z(0)
    , temperature(0)
    , sat_num(0)  {
    }
  ublox_msgs_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , lon(0)
    , lat(0)
    , height(0)
    , hour(0)
    , min(0)
    , sec(0)
    , gpsFix(0)
    , gspeed(0)
    , headmot(0)
    , headveh(0)
    , gyro_x(0)
    , gyro_y(0)
    , gyro_z(0)
    , acc_x(0)
    , acc_y(0)
    , acc_z(0)
    , temperature(0)
    , sat_num(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int64_t _lon_type;
  _lon_type lon;

   typedef int64_t _lat_type;
  _lat_type lat;

   typedef int64_t _height_type;
  _height_type height;

   typedef int64_t _hour_type;
  _hour_type hour;

   typedef int64_t _min_type;
  _min_type min;

   typedef int64_t _sec_type;
  _sec_type sec;

   typedef int8_t _gpsFix_type;
  _gpsFix_type gpsFix;

   typedef int64_t _gspeed_type;
  _gspeed_type gspeed;

   typedef int64_t _headmot_type;
  _headmot_type headmot;

   typedef int64_t _headveh_type;
  _headveh_type headveh;

   typedef int64_t _gyro_x_type;
  _gyro_x_type gyro_x;

   typedef int64_t _gyro_y_type;
  _gyro_y_type gyro_y;

   typedef int64_t _gyro_z_type;
  _gyro_z_type gyro_z;

   typedef int64_t _acc_x_type;
  _acc_x_type acc_x;

   typedef int64_t _acc_y_type;
  _acc_y_type acc_y;

   typedef int64_t _acc_z_type;
  _acc_z_type acc_z;

   typedef int64_t _temperature_type;
  _temperature_type temperature;

   typedef int8_t _sat_num_type;
  _sat_num_type sat_num;




  typedef boost::shared_ptr< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> const> ConstPtr;

}; // struct ublox_msgs_

typedef ::clothoid_msgs::ublox_msgs_<std::allocator<void> > ublox_msgs;

typedef boost::shared_ptr< ::clothoid_msgs::ublox_msgs > ublox_msgsPtr;
typedef boost::shared_ptr< ::clothoid_msgs::ublox_msgs const> ublox_msgsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::clothoid_msgs::ublox_msgs_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace clothoid_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'clothoid_msgs': ['/home/scv/clothoid_ws/src/clothoid_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "161e44786b206f716648ae9d8440652d";
  }

  static const char* value(const ::clothoid_msgs::ublox_msgs_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x161e44786b206f71ULL;
  static const uint64_t static_value2 = 0x6648ae9d8440652dULL;
};

template<class ContainerAllocator>
struct DataType< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "clothoid_msgs/ublox_msgs";
  }

  static const char* value(const ::clothoid_msgs::ublox_msgs_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#UBX_MSGS\n\
Header header\n\
int64 lon\n\
int64 lat\n\
int64 height\n\
int64 hour\n\
int64 min\n\
int64 sec\n\
int8 gpsFix\n\
int64 gspeed\n\
int64 headmot\n\
int64 headveh\n\
int64 gyro_x\n\
int64 gyro_y\n\
int64 gyro_z\n\
int64 acc_x\n\
int64 acc_y\n\
int64 acc_z\n\
int64 temperature\n\
int8 sat_num\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::clothoid_msgs::ublox_msgs_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.lon);
      stream.next(m.lat);
      stream.next(m.height);
      stream.next(m.hour);
      stream.next(m.min);
      stream.next(m.sec);
      stream.next(m.gpsFix);
      stream.next(m.gspeed);
      stream.next(m.headmot);
      stream.next(m.headveh);
      stream.next(m.gyro_x);
      stream.next(m.gyro_y);
      stream.next(m.gyro_z);
      stream.next(m.acc_x);
      stream.next(m.acc_y);
      stream.next(m.acc_z);
      stream.next(m.temperature);
      stream.next(m.sat_num);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ublox_msgs_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::clothoid_msgs::ublox_msgs_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::clothoid_msgs::ublox_msgs_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "lon: ";
    Printer<int64_t>::stream(s, indent + "  ", v.lon);
    s << indent << "lat: ";
    Printer<int64_t>::stream(s, indent + "  ", v.lat);
    s << indent << "height: ";
    Printer<int64_t>::stream(s, indent + "  ", v.height);
    s << indent << "hour: ";
    Printer<int64_t>::stream(s, indent + "  ", v.hour);
    s << indent << "min: ";
    Printer<int64_t>::stream(s, indent + "  ", v.min);
    s << indent << "sec: ";
    Printer<int64_t>::stream(s, indent + "  ", v.sec);
    s << indent << "gpsFix: ";
    Printer<int8_t>::stream(s, indent + "  ", v.gpsFix);
    s << indent << "gspeed: ";
    Printer<int64_t>::stream(s, indent + "  ", v.gspeed);
    s << indent << "headmot: ";
    Printer<int64_t>::stream(s, indent + "  ", v.headmot);
    s << indent << "headveh: ";
    Printer<int64_t>::stream(s, indent + "  ", v.headveh);
    s << indent << "gyro_x: ";
    Printer<int64_t>::stream(s, indent + "  ", v.gyro_x);
    s << indent << "gyro_y: ";
    Printer<int64_t>::stream(s, indent + "  ", v.gyro_y);
    s << indent << "gyro_z: ";
    Printer<int64_t>::stream(s, indent + "  ", v.gyro_z);
    s << indent << "acc_x: ";
    Printer<int64_t>::stream(s, indent + "  ", v.acc_x);
    s << indent << "acc_y: ";
    Printer<int64_t>::stream(s, indent + "  ", v.acc_y);
    s << indent << "acc_z: ";
    Printer<int64_t>::stream(s, indent + "  ", v.acc_z);
    s << indent << "temperature: ";
    Printer<int64_t>::stream(s, indent + "  ", v.temperature);
    s << indent << "sat_num: ";
    Printer<int8_t>::stream(s, indent + "  ", v.sat_num);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CLOTHOID_MSGS_MESSAGE_UBLOX_MSGS_H
