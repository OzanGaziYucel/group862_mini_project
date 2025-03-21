// Generated by gencpp from file mia_hand_msgs/GraspRef.msg
// DO NOT EDIT!


#ifndef MIA_HAND_MSGS_MESSAGE_GRASPREF_H
#define MIA_HAND_MSGS_MESSAGE_GRASPREF_H


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
struct GraspRef_
{
  typedef GraspRef_<ContainerAllocator> Type;

  GraspRef_()
    : rest(0)
    , pos(0)
    , delay(0)  {
    }
  GraspRef_(const ContainerAllocator& _alloc)
    : rest(0)
    , pos(0)
    , delay(0)  {
  (void)_alloc;
    }



   typedef int16_t _rest_type;
  _rest_type rest;

   typedef int16_t _pos_type;
  _pos_type pos;

   typedef int16_t _delay_type;
  _delay_type delay;





  typedef boost::shared_ptr< ::mia_hand_msgs::GraspRef_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mia_hand_msgs::GraspRef_<ContainerAllocator> const> ConstPtr;

}; // struct GraspRef_

typedef ::mia_hand_msgs::GraspRef_<std::allocator<void> > GraspRef;

typedef boost::shared_ptr< ::mia_hand_msgs::GraspRef > GraspRefPtr;
typedef boost::shared_ptr< ::mia_hand_msgs::GraspRef const> GraspRefConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mia_hand_msgs::GraspRef_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mia_hand_msgs::GraspRef_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mia_hand_msgs::GraspRef_<ContainerAllocator1> & lhs, const ::mia_hand_msgs::GraspRef_<ContainerAllocator2> & rhs)
{
  return lhs.rest == rhs.rest &&
    lhs.pos == rhs.pos &&
    lhs.delay == rhs.delay;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mia_hand_msgs::GraspRef_<ContainerAllocator1> & lhs, const ::mia_hand_msgs::GraspRef_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mia_hand_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mia_hand_msgs::GraspRef_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mia_hand_msgs::GraspRef_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mia_hand_msgs::GraspRef_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mia_hand_msgs::GraspRef_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mia_hand_msgs::GraspRef_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mia_hand_msgs::GraspRef_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mia_hand_msgs::GraspRef_<ContainerAllocator> >
{
  static const char* value()
  {
    return "09f519c8adf07285290a71d280a83d8c";
  }

  static const char* value(const ::mia_hand_msgs::GraspRef_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x09f519c8adf07285ULL;
  static const uint64_t static_value2 = 0x290a71d280a83d8cULL;
};

template<class ContainerAllocator>
struct DataType< ::mia_hand_msgs::GraspRef_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mia_hand_msgs/GraspRef";
  }

  static const char* value(const ::mia_hand_msgs::GraspRef_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mia_hand_msgs::GraspRef_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 rest\n"
"int16 pos\n"
"int16 delay\n"
;
  }

  static const char* value(const ::mia_hand_msgs::GraspRef_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mia_hand_msgs::GraspRef_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.rest);
      stream.next(m.pos);
      stream.next(m.delay);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GraspRef_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mia_hand_msgs::GraspRef_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mia_hand_msgs::GraspRef_<ContainerAllocator>& v)
  {
    s << indent << "rest: ";
    Printer<int16_t>::stream(s, indent + "  ", v.rest);
    s << indent << "pos: ";
    Printer<int16_t>::stream(s, indent + "  ", v.pos);
    s << indent << "delay: ";
    Printer<int16_t>::stream(s, indent + "  ", v.delay);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MIA_HAND_MSGS_MESSAGE_GRASPREF_H
