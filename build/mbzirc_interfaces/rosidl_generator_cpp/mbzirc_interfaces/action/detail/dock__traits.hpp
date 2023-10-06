// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mbzirc_interfaces:action/Dock.idl
// generated code does not contain a copyright notice

#ifndef MBZIRC_INTERFACES__ACTION__DETAIL__DOCK__TRAITS_HPP_
#define MBZIRC_INTERFACES__ACTION__DETAIL__DOCK__TRAITS_HPP_

#include "mbzirc_interfaces/action/detail/dock__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbzirc_interfaces::action::Dock_Goal>()
{
  return "mbzirc_interfaces::action::Dock_Goal";
}

template<>
inline const char * name<mbzirc_interfaces::action::Dock_Goal>()
{
  return "mbzirc_interfaces/action/Dock_Goal";
}

template<>
struct has_fixed_size<mbzirc_interfaces::action::Dock_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mbzirc_interfaces::action::Dock_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mbzirc_interfaces::action::Dock_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbzirc_interfaces::action::Dock_Result>()
{
  return "mbzirc_interfaces::action::Dock_Result";
}

template<>
inline const char * name<mbzirc_interfaces::action::Dock_Result>()
{
  return "mbzirc_interfaces/action/Dock_Result";
}

template<>
struct has_fixed_size<mbzirc_interfaces::action::Dock_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mbzirc_interfaces::action::Dock_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mbzirc_interfaces::action::Dock_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbzirc_interfaces::action::Dock_Feedback>()
{
  return "mbzirc_interfaces::action::Dock_Feedback";
}

template<>
inline const char * name<mbzirc_interfaces::action::Dock_Feedback>()
{
  return "mbzirc_interfaces/action/Dock_Feedback";
}

template<>
struct has_fixed_size<mbzirc_interfaces::action::Dock_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mbzirc_interfaces::action::Dock_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mbzirc_interfaces::action::Dock_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "mbzirc_interfaces/action/detail/dock__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbzirc_interfaces::action::Dock_SendGoal_Request>()
{
  return "mbzirc_interfaces::action::Dock_SendGoal_Request";
}

template<>
inline const char * name<mbzirc_interfaces::action::Dock_SendGoal_Request>()
{
  return "mbzirc_interfaces/action/Dock_SendGoal_Request";
}

template<>
struct has_fixed_size<mbzirc_interfaces::action::Dock_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<mbzirc_interfaces::action::Dock_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<mbzirc_interfaces::action::Dock_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<mbzirc_interfaces::action::Dock_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<mbzirc_interfaces::action::Dock_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbzirc_interfaces::action::Dock_SendGoal_Response>()
{
  return "mbzirc_interfaces::action::Dock_SendGoal_Response";
}

template<>
inline const char * name<mbzirc_interfaces::action::Dock_SendGoal_Response>()
{
  return "mbzirc_interfaces/action/Dock_SendGoal_Response";
}

template<>
struct has_fixed_size<mbzirc_interfaces::action::Dock_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<mbzirc_interfaces::action::Dock_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<mbzirc_interfaces::action::Dock_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbzirc_interfaces::action::Dock_SendGoal>()
{
  return "mbzirc_interfaces::action::Dock_SendGoal";
}

template<>
inline const char * name<mbzirc_interfaces::action::Dock_SendGoal>()
{
  return "mbzirc_interfaces/action/Dock_SendGoal";
}

template<>
struct has_fixed_size<mbzirc_interfaces::action::Dock_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<mbzirc_interfaces::action::Dock_SendGoal_Request>::value &&
    has_fixed_size<mbzirc_interfaces::action::Dock_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<mbzirc_interfaces::action::Dock_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<mbzirc_interfaces::action::Dock_SendGoal_Request>::value &&
    has_bounded_size<mbzirc_interfaces::action::Dock_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<mbzirc_interfaces::action::Dock_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<mbzirc_interfaces::action::Dock_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mbzirc_interfaces::action::Dock_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbzirc_interfaces::action::Dock_GetResult_Request>()
{
  return "mbzirc_interfaces::action::Dock_GetResult_Request";
}

template<>
inline const char * name<mbzirc_interfaces::action::Dock_GetResult_Request>()
{
  return "mbzirc_interfaces/action/Dock_GetResult_Request";
}

template<>
struct has_fixed_size<mbzirc_interfaces::action::Dock_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<mbzirc_interfaces::action::Dock_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<mbzirc_interfaces::action::Dock_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "mbzirc_interfaces/action/detail/dock__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbzirc_interfaces::action::Dock_GetResult_Response>()
{
  return "mbzirc_interfaces::action::Dock_GetResult_Response";
}

template<>
inline const char * name<mbzirc_interfaces::action::Dock_GetResult_Response>()
{
  return "mbzirc_interfaces/action/Dock_GetResult_Response";
}

template<>
struct has_fixed_size<mbzirc_interfaces::action::Dock_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<mbzirc_interfaces::action::Dock_Result>::value> {};

template<>
struct has_bounded_size<mbzirc_interfaces::action::Dock_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<mbzirc_interfaces::action::Dock_Result>::value> {};

template<>
struct is_message<mbzirc_interfaces::action::Dock_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbzirc_interfaces::action::Dock_GetResult>()
{
  return "mbzirc_interfaces::action::Dock_GetResult";
}

template<>
inline const char * name<mbzirc_interfaces::action::Dock_GetResult>()
{
  return "mbzirc_interfaces/action/Dock_GetResult";
}

template<>
struct has_fixed_size<mbzirc_interfaces::action::Dock_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<mbzirc_interfaces::action::Dock_GetResult_Request>::value &&
    has_fixed_size<mbzirc_interfaces::action::Dock_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<mbzirc_interfaces::action::Dock_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<mbzirc_interfaces::action::Dock_GetResult_Request>::value &&
    has_bounded_size<mbzirc_interfaces::action::Dock_GetResult_Response>::value
  >
{
};

template<>
struct is_service<mbzirc_interfaces::action::Dock_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<mbzirc_interfaces::action::Dock_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mbzirc_interfaces::action::Dock_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "mbzirc_interfaces/action/detail/dock__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbzirc_interfaces::action::Dock_FeedbackMessage>()
{
  return "mbzirc_interfaces::action::Dock_FeedbackMessage";
}

template<>
inline const char * name<mbzirc_interfaces::action::Dock_FeedbackMessage>()
{
  return "mbzirc_interfaces/action/Dock_FeedbackMessage";
}

template<>
struct has_fixed_size<mbzirc_interfaces::action::Dock_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<mbzirc_interfaces::action::Dock_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<mbzirc_interfaces::action::Dock_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<mbzirc_interfaces::action::Dock_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<mbzirc_interfaces::action::Dock_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<mbzirc_interfaces::action::Dock>
  : std::true_type
{
};

template<>
struct is_action_goal<mbzirc_interfaces::action::Dock_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<mbzirc_interfaces::action::Dock_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<mbzirc_interfaces::action::Dock_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // MBZIRC_INTERFACES__ACTION__DETAIL__DOCK__TRAITS_HPP_
