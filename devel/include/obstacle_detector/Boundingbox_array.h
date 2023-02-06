// Generated by gencpp from file obstacle_detector/Boundingbox_array.msg
// DO NOT EDIT!


#ifndef OBSTACLE_DETECTOR_MESSAGE_BOUNDINGBOX_ARRAY_H
#define OBSTACLE_DETECTOR_MESSAGE_BOUNDINGBOX_ARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <obstacle_detector/Boundingbox.h>

namespace obstacle_detector
{
template <class ContainerAllocator>
struct Boundingbox_array_
{
  typedef Boundingbox_array_<ContainerAllocator> Type;

  Boundingbox_array_()
    : boundingbox_array()  {
    }
  Boundingbox_array_(const ContainerAllocator& _alloc)
    : boundingbox_array(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::obstacle_detector::Boundingbox_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::obstacle_detector::Boundingbox_<ContainerAllocator> >::other >  _boundingbox_array_type;
  _boundingbox_array_type boundingbox_array;





  typedef boost::shared_ptr< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> const> ConstPtr;

}; // struct Boundingbox_array_

typedef ::obstacle_detector::Boundingbox_array_<std::allocator<void> > Boundingbox_array;

typedef boost::shared_ptr< ::obstacle_detector::Boundingbox_array > Boundingbox_arrayPtr;
typedef boost::shared_ptr< ::obstacle_detector::Boundingbox_array const> Boundingbox_arrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::obstacle_detector::Boundingbox_array_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::obstacle_detector::Boundingbox_array_<ContainerAllocator1> & lhs, const ::obstacle_detector::Boundingbox_array_<ContainerAllocator2> & rhs)
{
  return lhs.boundingbox_array == rhs.boundingbox_array;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::obstacle_detector::Boundingbox_array_<ContainerAllocator1> & lhs, const ::obstacle_detector::Boundingbox_array_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace obstacle_detector

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "72f4806878af1954f3e5c66182495641";
  }

  static const char* value(const ::obstacle_detector::Boundingbox_array_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x72f4806878af1954ULL;
  static const uint64_t static_value2 = 0xf3e5c66182495641ULL;
};

template<class ContainerAllocator>
struct DataType< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "obstacle_detector/Boundingbox_array";
  }

  static const char* value(const ::obstacle_detector::Boundingbox_array_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Boundingbox[] boundingbox_array\n"
"\n"
"================================================================================\n"
"MSG: obstacle_detector/Boundingbox\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 volume\n"
"float64 distance\n"
;
  }

  static const char* value(const ::obstacle_detector::Boundingbox_array_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.boundingbox_array);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Boundingbox_array_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::obstacle_detector::Boundingbox_array_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::obstacle_detector::Boundingbox_array_<ContainerAllocator>& v)
  {
    s << indent << "boundingbox_array[]" << std::endl;
    for (size_t i = 0; i < v.boundingbox_array.size(); ++i)
    {
      s << indent << "  boundingbox_array[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::obstacle_detector::Boundingbox_<ContainerAllocator> >::stream(s, indent + "    ", v.boundingbox_array[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // OBSTACLE_DETECTOR_MESSAGE_BOUNDINGBOX_ARRAY_H