// Generated by gencpp from file waypoint_maker/Boundingbox.msg
// DO NOT EDIT!


#ifndef WAYPOINT_MAKER_MESSAGE_BOUNDINGBOX_H
#define WAYPOINT_MAKER_MESSAGE_BOUNDINGBOX_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace waypoint_maker
{
template <class ContainerAllocator>
struct Boundingbox_
{
  typedef Boundingbox_<ContainerAllocator> Type;

  Boundingbox_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , volume(0.0)
    , distance(0.0)  {
    }
  Boundingbox_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , volume(0.0)
    , distance(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _volume_type;
  _volume_type volume;

   typedef double _distance_type;
  _distance_type distance;





  typedef boost::shared_ptr< ::waypoint_maker::Boundingbox_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::waypoint_maker::Boundingbox_<ContainerAllocator> const> ConstPtr;

}; // struct Boundingbox_

typedef ::waypoint_maker::Boundingbox_<std::allocator<void> > Boundingbox;

typedef boost::shared_ptr< ::waypoint_maker::Boundingbox > BoundingboxPtr;
typedef boost::shared_ptr< ::waypoint_maker::Boundingbox const> BoundingboxConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::waypoint_maker::Boundingbox_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::waypoint_maker::Boundingbox_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::waypoint_maker::Boundingbox_<ContainerAllocator1> & lhs, const ::waypoint_maker::Boundingbox_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.volume == rhs.volume &&
    lhs.distance == rhs.distance;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::waypoint_maker::Boundingbox_<ContainerAllocator1> & lhs, const ::waypoint_maker::Boundingbox_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace waypoint_maker

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::waypoint_maker::Boundingbox_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::waypoint_maker::Boundingbox_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::waypoint_maker::Boundingbox_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::waypoint_maker::Boundingbox_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::waypoint_maker::Boundingbox_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::waypoint_maker::Boundingbox_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::waypoint_maker::Boundingbox_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a9ca733902fb20b3437213619205592f";
  }

  static const char* value(const ::waypoint_maker::Boundingbox_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa9ca733902fb20b3ULL;
  static const uint64_t static_value2 = 0x437213619205592fULL;
};

template<class ContainerAllocator>
struct DataType< ::waypoint_maker::Boundingbox_<ContainerAllocator> >
{
  static const char* value()
  {
    return "waypoint_maker/Boundingbox";
  }

  static const char* value(const ::waypoint_maker::Boundingbox_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::waypoint_maker::Boundingbox_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 volume\n"
"float64 distance\n"
;
  }

  static const char* value(const ::waypoint_maker::Boundingbox_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::waypoint_maker::Boundingbox_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.volume);
      stream.next(m.distance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Boundingbox_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::waypoint_maker::Boundingbox_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::waypoint_maker::Boundingbox_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "volume: ";
    Printer<double>::stream(s, indent + "  ", v.volume);
    s << indent << "distance: ";
    Printer<double>::stream(s, indent + "  ", v.distance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WAYPOINT_MAKER_MESSAGE_BOUNDINGBOX_H
