// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vesc_msgs:msg/VescImu.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_IMU__TRAITS_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_IMU__TRAITS_HPP_

#include "vesc_msgs/msg/detail/vesc_imu__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'ypr'
// Member 'linear_acceleration'
// Member 'angular_velocity'
// Member 'compass'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const vesc_msgs::msg::VescImu & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ypr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ypr:\n";
    to_yaml(msg.ypr, out, indentation + 2);
  }

  // member: linear_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_acceleration:\n";
    to_yaml(msg.linear_acceleration, out, indentation + 2);
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity:\n";
    to_yaml(msg.angular_velocity, out, indentation + 2);
  }

  // member: compass
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compass:\n";
    to_yaml(msg.compass, out, indentation + 2);
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation:\n";
    to_yaml(msg.orientation, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const vesc_msgs::msg::VescImu & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<vesc_msgs::msg::VescImu>()
{
  return "vesc_msgs::msg::VescImu";
}

template<>
inline const char * name<vesc_msgs::msg::VescImu>()
{
  return "vesc_msgs/msg/VescImu";
}

template<>
struct has_fixed_size<vesc_msgs::msg::VescImu>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<vesc_msgs::msg::VescImu>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<vesc_msgs::msg::VescImu>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VESC_MSGS__MSG__DETAIL__VESC_IMU__TRAITS_HPP_
