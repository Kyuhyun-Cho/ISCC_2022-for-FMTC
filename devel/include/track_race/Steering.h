// Generated by gencpp from file track_race/Steering.msg
// DO NOT EDIT!


#ifndef TRACK_RACE_MESSAGE_STEERING_H
#define TRACK_RACE_MESSAGE_STEERING_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace track_race
{
template <class ContainerAllocator>
struct Steering_
{
  typedef Steering_<ContainerAllocator> Type;

  Steering_()
    : steering(0.0)  {
    }
  Steering_(const ContainerAllocator& _alloc)
    : steering(0.0)  {
  (void)_alloc;
    }



   typedef float _steering_type;
  _steering_type steering;





  typedef boost::shared_ptr< ::track_race::Steering_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::track_race::Steering_<ContainerAllocator> const> ConstPtr;

}; // struct Steering_

typedef ::track_race::Steering_<std::allocator<void> > Steering;

typedef boost::shared_ptr< ::track_race::Steering > SteeringPtr;
typedef boost::shared_ptr< ::track_race::Steering const> SteeringConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::track_race::Steering_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::track_race::Steering_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::track_race::Steering_<ContainerAllocator1> & lhs, const ::track_race::Steering_<ContainerAllocator2> & rhs)
{
  return lhs.steering == rhs.steering;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::track_race::Steering_<ContainerAllocator1> & lhs, const ::track_race::Steering_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace track_race

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::track_race::Steering_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::track_race::Steering_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::track_race::Steering_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::track_race::Steering_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::track_race::Steering_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::track_race::Steering_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::track_race::Steering_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5e5c60c40f2709684823442181fe6011";
  }

  static const char* value(const ::track_race::Steering_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5e5c60c40f270968ULL;
  static const uint64_t static_value2 = 0x4823442181fe6011ULL;
};

template<class ContainerAllocator>
struct DataType< ::track_race::Steering_<ContainerAllocator> >
{
  static const char* value()
  {
    return "track_race/Steering";
  }

  static const char* value(const ::track_race::Steering_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::track_race::Steering_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 steering\n"
;
  }

  static const char* value(const ::track_race::Steering_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::track_race::Steering_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.steering);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Steering_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::track_race::Steering_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::track_race::Steering_<ContainerAllocator>& v)
  {
    s << indent << "steering: ";
    Printer<float>::stream(s, indent + "  ", v.steering);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRACK_RACE_MESSAGE_STEERING_H
