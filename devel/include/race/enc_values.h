// Generated by gencpp from file race/enc_values.msg
// DO NOT EDIT!


#ifndef RACE_MESSAGE_ENC_VALUES_H
#define RACE_MESSAGE_ENC_VALUES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace race
{
template <class ContainerAllocator>
struct enc_values_
{
  typedef enc_values_<ContainerAllocator> Type;

  enc_values_()
    : steering(0)
    , enc_val(0)  {
    }
  enc_values_(const ContainerAllocator& _alloc)
    : steering(0)
    , enc_val(0)  {
  (void)_alloc;
    }



   typedef int16_t _steering_type;
  _steering_type steering;

   typedef int32_t _enc_val_type;
  _enc_val_type enc_val;





  typedef boost::shared_ptr< ::race::enc_values_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::race::enc_values_<ContainerAllocator> const> ConstPtr;

}; // struct enc_values_

typedef ::race::enc_values_<std::allocator<void> > enc_values;

typedef boost::shared_ptr< ::race::enc_values > enc_valuesPtr;
typedef boost::shared_ptr< ::race::enc_values const> enc_valuesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::race::enc_values_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::race::enc_values_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::race::enc_values_<ContainerAllocator1> & lhs, const ::race::enc_values_<ContainerAllocator2> & rhs)
{
  return lhs.steering == rhs.steering &&
    lhs.enc_val == rhs.enc_val;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::race::enc_values_<ContainerAllocator1> & lhs, const ::race::enc_values_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace race

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::race::enc_values_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::race::enc_values_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::race::enc_values_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::race::enc_values_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::race::enc_values_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::race::enc_values_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::race::enc_values_<ContainerAllocator> >
{
  static const char* value()
  {
    return "261c425b19fc3c9973e23fc0e2a83f33";
  }

  static const char* value(const ::race::enc_values_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x261c425b19fc3c99ULL;
  static const uint64_t static_value2 = 0x73e23fc0e2a83f33ULL;
};

template<class ContainerAllocator>
struct DataType< ::race::enc_values_<ContainerAllocator> >
{
  static const char* value()
  {
    return "race/enc_values";
  }

  static const char* value(const ::race::enc_values_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::race::enc_values_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 steering\n"
"int32 enc_val\n"
;
  }

  static const char* value(const ::race::enc_values_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::race::enc_values_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.steering);
      stream.next(m.enc_val);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct enc_values_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::race::enc_values_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::race::enc_values_<ContainerAllocator>& v)
  {
    s << indent << "steering: ";
    Printer<int16_t>::stream(s, indent + "  ", v.steering);
    s << indent << "enc_val: ";
    Printer<int32_t>::stream(s, indent + "  ", v.enc_val);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RACE_MESSAGE_ENC_VALUES_H
