// Generated by gencpp from file unitree_legged_msgs/MotorState.msg
// DO NOT EDIT!


#ifndef UNITREE_LEGGED_MSGS_MESSAGE_MOTORSTATE_H
#define UNITREE_LEGGED_MSGS_MESSAGE_MOTORSTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace unitree_legged_msgs
{
template <class ContainerAllocator>
struct MotorState_
{
  typedef MotorState_<ContainerAllocator> Type;

  MotorState_()
    : mode(0)
    , q(0.0)
    , dq(0.0)
    , ddq(0.0)
    , tauEst(0.0)
    , q_raw(0.0)
    , dq_raw(0.0)
    , ddq_raw(0.0)
    , temperature(0)
    , reserve()  {
      reserve.assign(0);
  }
  MotorState_(const ContainerAllocator& _alloc)
    : mode(0)
    , q(0.0)
    , dq(0.0)
    , ddq(0.0)
    , tauEst(0.0)
    , q_raw(0.0)
    , dq_raw(0.0)
    , ddq_raw(0.0)
    , temperature(0)
    , reserve()  {
  (void)_alloc;
      reserve.assign(0);
  }



   typedef uint8_t _mode_type;
  _mode_type mode;

   typedef float _q_type;
  _q_type q;

   typedef float _dq_type;
  _dq_type dq;

   typedef float _ddq_type;
  _ddq_type ddq;

   typedef float _tauEst_type;
  _tauEst_type tauEst;

   typedef float _q_raw_type;
  _q_raw_type q_raw;

   typedef float _dq_raw_type;
  _dq_raw_type dq_raw;

   typedef float _ddq_raw_type;
  _ddq_raw_type ddq_raw;

   typedef int8_t _temperature_type;
  _temperature_type temperature;

   typedef boost::array<uint32_t, 2>  _reserve_type;
  _reserve_type reserve;





  typedef boost::shared_ptr< ::unitree_legged_msgs::MotorState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::unitree_legged_msgs::MotorState_<ContainerAllocator> const> ConstPtr;

}; // struct MotorState_

typedef ::unitree_legged_msgs::MotorState_<std::allocator<void> > MotorState;

typedef boost::shared_ptr< ::unitree_legged_msgs::MotorState > MotorStatePtr;
typedef boost::shared_ptr< ::unitree_legged_msgs::MotorState const> MotorStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::unitree_legged_msgs::MotorState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::unitree_legged_msgs::MotorState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::unitree_legged_msgs::MotorState_<ContainerAllocator1> & lhs, const ::unitree_legged_msgs::MotorState_<ContainerAllocator2> & rhs)
{
  return lhs.mode == rhs.mode &&
    lhs.q == rhs.q &&
    lhs.dq == rhs.dq &&
    lhs.ddq == rhs.ddq &&
    lhs.tauEst == rhs.tauEst &&
    lhs.q_raw == rhs.q_raw &&
    lhs.dq_raw == rhs.dq_raw &&
    lhs.ddq_raw == rhs.ddq_raw &&
    lhs.temperature == rhs.temperature &&
    lhs.reserve == rhs.reserve;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::unitree_legged_msgs::MotorState_<ContainerAllocator1> & lhs, const ::unitree_legged_msgs::MotorState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace unitree_legged_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::unitree_legged_msgs::MotorState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::unitree_legged_msgs::MotorState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::unitree_legged_msgs::MotorState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::unitree_legged_msgs::MotorState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::unitree_legged_msgs::MotorState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::unitree_legged_msgs::MotorState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::unitree_legged_msgs::MotorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "94c55ee3b7852be2bd437b20ce12a254";
  }

  static const char* value(const ::unitree_legged_msgs::MotorState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x94c55ee3b7852be2ULL;
  static const uint64_t static_value2 = 0xbd437b20ce12a254ULL;
};

template<class ContainerAllocator>
struct DataType< ::unitree_legged_msgs::MotorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "unitree_legged_msgs/MotorState";
  }

  static const char* value(const ::unitree_legged_msgs::MotorState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::unitree_legged_msgs::MotorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 mode           # motor current mode \n"
"float32 q            # motor current position（rad）\n"
"float32 dq           # motor current speed（rad/s）\n"
"float32 ddq          # motor current speed（rad/s）\n"
"float32 tauEst       # current estimated output torque（N*m）\n"
"float32 q_raw        # motor current position（rad）\n"
"float32 dq_raw       # motor current speed（rad/s）\n"
"float32 ddq_raw      # motor current speed（rad/s）\n"
"int8 temperature     # motor temperature（slow conduction of temperature leads to lag）\n"
"uint32[2] reserve\n"
;
  }

  static const char* value(const ::unitree_legged_msgs::MotorState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::unitree_legged_msgs::MotorState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
      stream.next(m.q);
      stream.next(m.dq);
      stream.next(m.ddq);
      stream.next(m.tauEst);
      stream.next(m.q_raw);
      stream.next(m.dq_raw);
      stream.next(m.ddq_raw);
      stream.next(m.temperature);
      stream.next(m.reserve);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotorState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::unitree_legged_msgs::MotorState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::unitree_legged_msgs::MotorState_<ContainerAllocator>& v)
  {
    if (false || !indent.empty())
      s << std::endl;
    s << indent << "mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mode);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "q: ";
    Printer<float>::stream(s, indent + "  ", v.q);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "dq: ";
    Printer<float>::stream(s, indent + "  ", v.dq);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "ddq: ";
    Printer<float>::stream(s, indent + "  ", v.ddq);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "tauEst: ";
    Printer<float>::stream(s, indent + "  ", v.tauEst);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "q_raw: ";
    Printer<float>::stream(s, indent + "  ", v.q_raw);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "dq_raw: ";
    Printer<float>::stream(s, indent + "  ", v.dq_raw);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "ddq_raw: ";
    Printer<float>::stream(s, indent + "  ", v.ddq_raw);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "temperature: ";
    Printer<int8_t>::stream(s, indent + "  ", v.temperature);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "reserve: ";
    if (v.reserve.empty() || true)
      s << "[";
    for (size_t i = 0; i < v.reserve.size(); ++i)
    {
      if (true && i > 0)
        s << ", ";
      else if (!true)
        s << std::endl << indent << "  -";
      Printer<uint32_t>::stream(s, true ? std::string() : indent + "    ", v.reserve[i]);
    }
    if (v.reserve.empty() || true)
      s << "]";
  }
};

} // namespace message_operations
} // namespace ros

#endif // UNITREE_LEGGED_MSGS_MESSAGE_MOTORSTATE_H
