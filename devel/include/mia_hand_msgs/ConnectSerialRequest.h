// Generated by gencpp from file mia_hand_msgs/ConnectSerialRequest.msg
// DO NOT EDIT!


#ifndef MIA_HAND_MSGS_MESSAGE_CONNECTSERIALREQUEST_H
#define MIA_HAND_MSGS_MESSAGE_CONNECTSERIALREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mia_hand_msgs
{
template <class ContainerAllocator>
struct ConnectSerialRequest_
{
  typedef ConnectSerialRequest_<ContainerAllocator> Type;

  ConnectSerialRequest_()
    : port(0)  {
    }
  ConnectSerialRequest_(const ContainerAllocator& _alloc)
    : port(0)  {
  (void)_alloc;
    }



   typedef uint16_t _port_type;
  _port_type port;





  typedef boost::shared_ptr< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ConnectSerialRequest_

typedef ::mia_hand_msgs::ConnectSerialRequest_<std::allocator<void> > ConnectSerialRequest;

typedef boost::shared_ptr< ::mia_hand_msgs::ConnectSerialRequest > ConnectSerialRequestPtr;
typedef boost::shared_ptr< ::mia_hand_msgs::ConnectSerialRequest const> ConnectSerialRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator1> & lhs, const ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator2> & rhs)
{
  return lhs.port == rhs.port;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator1> & lhs, const ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mia_hand_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ced1f923b6ab76bb78a6eedb854d46af";
  }

  static const char* value(const ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xced1f923b6ab76bbULL;
  static const uint64_t static_value2 = 0x78a6eedb854d46afULL;
};

template<class ContainerAllocator>
struct DataType< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mia_hand_msgs/ConnectSerialRequest";
  }

  static const char* value(const ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16 port\n"
;
  }

  static const char* value(const ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.port);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ConnectSerialRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mia_hand_msgs::ConnectSerialRequest_<ContainerAllocator>& v)
  {
    s << indent << "port: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.port);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MIA_HAND_MSGS_MESSAGE_CONNECTSERIALREQUEST_H
