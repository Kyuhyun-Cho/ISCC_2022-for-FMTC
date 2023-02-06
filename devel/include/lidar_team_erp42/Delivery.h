// Generated by gencpp from file lidar_team_erp42/Delivery.msg
// DO NOT EDIT!


#ifndef LIDAR_TEAM_ERP42_MESSAGE_DELIVERY_H
#define LIDAR_TEAM_ERP42_MESSAGE_DELIVERY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lidar_team_erp42
{
template <class ContainerAllocator>
struct Delivery_
{
  typedef Delivery_<ContainerAllocator> Type;

  Delivery_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , angle(0.0)  {
    }
  Delivery_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , angle(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _angle_type;
  _angle_type angle;





  typedef boost::shared_ptr< ::lidar_team_erp42::Delivery_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_team_erp42::Delivery_<ContainerAllocator> const> ConstPtr;

}; // struct Delivery_

typedef ::lidar_team_erp42::Delivery_<std::allocator<void> > Delivery;

typedef boost::shared_ptr< ::lidar_team_erp42::Delivery > DeliveryPtr;
typedef boost::shared_ptr< ::lidar_team_erp42::Delivery const> DeliveryConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_team_erp42::Delivery_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_team_erp42::Delivery_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lidar_team_erp42::Delivery_<ContainerAllocator1> & lhs, const ::lidar_team_erp42::Delivery_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.angle == rhs.angle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lidar_team_erp42::Delivery_<ContainerAllocator1> & lhs, const ::lidar_team_erp42::Delivery_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lidar_team_erp42

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::lidar_team_erp42::Delivery_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_team_erp42::Delivery_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_team_erp42::Delivery_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_team_erp42::Delivery_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_team_erp42::Delivery_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_team_erp42::Delivery_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_team_erp42::Delivery_<ContainerAllocator> >
{
  static const char* value()
  {
    return "496d96d869521d54a901ba63c507999a";
  }

  static const char* value(const ::lidar_team_erp42::Delivery_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x496d96d869521d54ULL;
  static const uint64_t static_value2 = 0xa901ba63c507999aULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_team_erp42::Delivery_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_team_erp42/Delivery";
  }

  static const char* value(const ::lidar_team_erp42::Delivery_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_team_erp42::Delivery_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 angle\n"
;
  }

  static const char* value(const ::lidar_team_erp42::Delivery_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_team_erp42::Delivery_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Delivery_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_team_erp42::Delivery_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lidar_team_erp42::Delivery_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "angle: ";
    Printer<double>::stream(s, indent + "  ", v.angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_TEAM_ERP42_MESSAGE_DELIVERY_H
