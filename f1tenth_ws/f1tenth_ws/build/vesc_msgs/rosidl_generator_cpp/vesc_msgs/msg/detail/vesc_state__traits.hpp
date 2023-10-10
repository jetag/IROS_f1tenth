// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vesc_msgs:msg/VescState.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_STATE__TRAITS_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_STATE__TRAITS_HPP_

#include "vesc_msgs/msg/detail/vesc_state__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const vesc_msgs::msg::VescState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: temp_fet
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temp_fet: ";
    value_to_yaml(msg.temp_fet, out);
    out << "\n";
  }

  // member: temp_motor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temp_motor: ";
    value_to_yaml(msg.temp_motor, out);
    out << "\n";
  }

  // member: current_motor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_motor: ";
    value_to_yaml(msg.current_motor, out);
    out << "\n";
  }

  // member: current_input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_input: ";
    value_to_yaml(msg.current_input, out);
    out << "\n";
  }

  // member: avg_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avg_id: ";
    value_to_yaml(msg.avg_id, out);
    out << "\n";
  }

  // member: avg_iq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avg_iq: ";
    value_to_yaml(msg.avg_iq, out);
    out << "\n";
  }

  // member: duty_cycle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duty_cycle: ";
    value_to_yaml(msg.duty_cycle, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: voltage_input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage_input: ";
    value_to_yaml(msg.voltage_input, out);
    out << "\n";
  }

  // member: charge_drawn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charge_drawn: ";
    value_to_yaml(msg.charge_drawn, out);
    out << "\n";
  }

  // member: charge_regen
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charge_regen: ";
    value_to_yaml(msg.charge_regen, out);
    out << "\n";
  }

  // member: energy_drawn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "energy_drawn: ";
    value_to_yaml(msg.energy_drawn, out);
    out << "\n";
  }

  // member: energy_regen
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "energy_regen: ";
    value_to_yaml(msg.energy_regen, out);
    out << "\n";
  }

  // member: displacement
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "displacement: ";
    value_to_yaml(msg.displacement, out);
    out << "\n";
  }

  // member: distance_traveled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_traveled: ";
    value_to_yaml(msg.distance_traveled, out);
    out << "\n";
  }

  // member: fault_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_code: ";
    value_to_yaml(msg.fault_code, out);
    out << "\n";
  }

  // member: pid_pos_now
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pid_pos_now: ";
    value_to_yaml(msg.pid_pos_now, out);
    out << "\n";
  }

  // member: controller_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "controller_id: ";
    value_to_yaml(msg.controller_id, out);
    out << "\n";
  }

  // member: ntc_temp_mos1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ntc_temp_mos1: ";
    value_to_yaml(msg.ntc_temp_mos1, out);
    out << "\n";
  }

  // member: ntc_temp_mos2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ntc_temp_mos2: ";
    value_to_yaml(msg.ntc_temp_mos2, out);
    out << "\n";
  }

  // member: ntc_temp_mos3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ntc_temp_mos3: ";
    value_to_yaml(msg.ntc_temp_mos3, out);
    out << "\n";
  }

  // member: avg_vd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avg_vd: ";
    value_to_yaml(msg.avg_vd, out);
    out << "\n";
  }

  // member: avg_vq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avg_vq: ";
    value_to_yaml(msg.avg_vq, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const vesc_msgs::msg::VescState & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<vesc_msgs::msg::VescState>()
{
  return "vesc_msgs::msg::VescState";
}

template<>
inline const char * name<vesc_msgs::msg::VescState>()
{
  return "vesc_msgs/msg/VescState";
}

template<>
struct has_fixed_size<vesc_msgs::msg::VescState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vesc_msgs::msg::VescState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vesc_msgs::msg::VescState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VESC_MSGS__MSG__DETAIL__VESC_STATE__TRAITS_HPP_
