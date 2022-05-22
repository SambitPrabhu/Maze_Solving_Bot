// Generated by gencpp from file package1234/dest.msg
// DO NOT EDIT!


#ifndef PACKAGE1234_MESSAGE_DEST_H
#define PACKAGE1234_MESSAGE_DEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace package1234
{
template <class ContainerAllocator>
struct dest_
{
  typedef dest_<ContainerAllocator> Type;

  dest_()
    : dest_x_coordinate(0.0)
    , dest_y_coordinate(0.0)  {
    }
  dest_(const ContainerAllocator& _alloc)
    : dest_x_coordinate(0.0)
    , dest_y_coordinate(0.0)  {
  (void)_alloc;
    }



   typedef double _dest_x_coordinate_type;
  _dest_x_coordinate_type dest_x_coordinate;

   typedef double _dest_y_coordinate_type;
  _dest_y_coordinate_type dest_y_coordinate;





  typedef boost::shared_ptr< ::package1234::dest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::package1234::dest_<ContainerAllocator> const> ConstPtr;

}; // struct dest_

typedef ::package1234::dest_<std::allocator<void> > dest;

typedef boost::shared_ptr< ::package1234::dest > destPtr;
typedef boost::shared_ptr< ::package1234::dest const> destConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::package1234::dest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::package1234::dest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::package1234::dest_<ContainerAllocator1> & lhs, const ::package1234::dest_<ContainerAllocator2> & rhs)
{
  return lhs.dest_x_coordinate == rhs.dest_x_coordinate &&
    lhs.dest_y_coordinate == rhs.dest_y_coordinate;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::package1234::dest_<ContainerAllocator1> & lhs, const ::package1234::dest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace package1234

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::package1234::dest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::package1234::dest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::package1234::dest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::package1234::dest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::package1234::dest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::package1234::dest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::package1234::dest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cf363d99a20f66b06e0d4259cb1930ec";
  }

  static const char* value(const ::package1234::dest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcf363d99a20f66b0ULL;
  static const uint64_t static_value2 = 0x6e0d4259cb1930ecULL;
};

template<class ContainerAllocator>
struct DataType< ::package1234::dest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "package1234/dest";
  }

  static const char* value(const ::package1234::dest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::package1234::dest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 dest_x_coordinate\n"
"float64 dest_y_coordinate\n"
;
  }

  static const char* value(const ::package1234::dest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::package1234::dest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.dest_x_coordinate);
      stream.next(m.dest_y_coordinate);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct dest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::package1234::dest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::package1234::dest_<ContainerAllocator>& v)
  {
    s << indent << "dest_x_coordinate: ";
    Printer<double>::stream(s, indent + "  ", v.dest_x_coordinate);
    s << indent << "dest_y_coordinate: ";
    Printer<double>::stream(s, indent + "  ", v.dest_y_coordinate);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PACKAGE1234_MESSAGE_DEST_H
