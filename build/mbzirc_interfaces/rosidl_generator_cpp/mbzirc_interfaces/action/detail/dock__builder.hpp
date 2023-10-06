// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbzirc_interfaces:action/Dock.idl
// generated code does not contain a copyright notice

#ifndef MBZIRC_INTERFACES__ACTION__DETAIL__DOCK__BUILDER_HPP_
#define MBZIRC_INTERFACES__ACTION__DETAIL__DOCK__BUILDER_HPP_

#include "mbzirc_interfaces/action/detail/dock__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace mbzirc_interfaces
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbzirc_interfaces::action::Dock_Goal>()
{
  return ::mbzirc_interfaces::action::Dock_Goal(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace mbzirc_interfaces


namespace mbzirc_interfaces
{

namespace action
{

namespace builder
{

class Init_Dock_Result_success
{
public:
  Init_Dock_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mbzirc_interfaces::action::Dock_Result success(::mbzirc_interfaces::action::Dock_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbzirc_interfaces::action::Dock_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbzirc_interfaces::action::Dock_Result>()
{
  return mbzirc_interfaces::action::builder::Init_Dock_Result_success();
}

}  // namespace mbzirc_interfaces


namespace mbzirc_interfaces
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbzirc_interfaces::action::Dock_Feedback>()
{
  return ::mbzirc_interfaces::action::Dock_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace mbzirc_interfaces


namespace mbzirc_interfaces
{

namespace action
{

namespace builder
{

class Init_Dock_SendGoal_Request_goal
{
public:
  explicit Init_Dock_SendGoal_Request_goal(::mbzirc_interfaces::action::Dock_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::mbzirc_interfaces::action::Dock_SendGoal_Request goal(::mbzirc_interfaces::action::Dock_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbzirc_interfaces::action::Dock_SendGoal_Request msg_;
};

class Init_Dock_SendGoal_Request_goal_id
{
public:
  Init_Dock_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dock_SendGoal_Request_goal goal_id(::mbzirc_interfaces::action::Dock_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Dock_SendGoal_Request_goal(msg_);
  }

private:
  ::mbzirc_interfaces::action::Dock_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbzirc_interfaces::action::Dock_SendGoal_Request>()
{
  return mbzirc_interfaces::action::builder::Init_Dock_SendGoal_Request_goal_id();
}

}  // namespace mbzirc_interfaces


namespace mbzirc_interfaces
{

namespace action
{

namespace builder
{

class Init_Dock_SendGoal_Response_stamp
{
public:
  explicit Init_Dock_SendGoal_Response_stamp(::mbzirc_interfaces::action::Dock_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::mbzirc_interfaces::action::Dock_SendGoal_Response stamp(::mbzirc_interfaces::action::Dock_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbzirc_interfaces::action::Dock_SendGoal_Response msg_;
};

class Init_Dock_SendGoal_Response_accepted
{
public:
  Init_Dock_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dock_SendGoal_Response_stamp accepted(::mbzirc_interfaces::action::Dock_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Dock_SendGoal_Response_stamp(msg_);
  }

private:
  ::mbzirc_interfaces::action::Dock_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbzirc_interfaces::action::Dock_SendGoal_Response>()
{
  return mbzirc_interfaces::action::builder::Init_Dock_SendGoal_Response_accepted();
}

}  // namespace mbzirc_interfaces


namespace mbzirc_interfaces
{

namespace action
{

namespace builder
{

class Init_Dock_GetResult_Request_goal_id
{
public:
  Init_Dock_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mbzirc_interfaces::action::Dock_GetResult_Request goal_id(::mbzirc_interfaces::action::Dock_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbzirc_interfaces::action::Dock_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbzirc_interfaces::action::Dock_GetResult_Request>()
{
  return mbzirc_interfaces::action::builder::Init_Dock_GetResult_Request_goal_id();
}

}  // namespace mbzirc_interfaces


namespace mbzirc_interfaces
{

namespace action
{

namespace builder
{

class Init_Dock_GetResult_Response_result
{
public:
  explicit Init_Dock_GetResult_Response_result(::mbzirc_interfaces::action::Dock_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::mbzirc_interfaces::action::Dock_GetResult_Response result(::mbzirc_interfaces::action::Dock_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbzirc_interfaces::action::Dock_GetResult_Response msg_;
};

class Init_Dock_GetResult_Response_status
{
public:
  Init_Dock_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dock_GetResult_Response_result status(::mbzirc_interfaces::action::Dock_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Dock_GetResult_Response_result(msg_);
  }

private:
  ::mbzirc_interfaces::action::Dock_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbzirc_interfaces::action::Dock_GetResult_Response>()
{
  return mbzirc_interfaces::action::builder::Init_Dock_GetResult_Response_status();
}

}  // namespace mbzirc_interfaces


namespace mbzirc_interfaces
{

namespace action
{

namespace builder
{

class Init_Dock_FeedbackMessage_feedback
{
public:
  explicit Init_Dock_FeedbackMessage_feedback(::mbzirc_interfaces::action::Dock_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::mbzirc_interfaces::action::Dock_FeedbackMessage feedback(::mbzirc_interfaces::action::Dock_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbzirc_interfaces::action::Dock_FeedbackMessage msg_;
};

class Init_Dock_FeedbackMessage_goal_id
{
public:
  Init_Dock_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dock_FeedbackMessage_feedback goal_id(::mbzirc_interfaces::action::Dock_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Dock_FeedbackMessage_feedback(msg_);
  }

private:
  ::mbzirc_interfaces::action::Dock_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbzirc_interfaces::action::Dock_FeedbackMessage>()
{
  return mbzirc_interfaces::action::builder::Init_Dock_FeedbackMessage_goal_id();
}

}  // namespace mbzirc_interfaces

#endif  // MBZIRC_INTERFACES__ACTION__DETAIL__DOCK__BUILDER_HPP_
