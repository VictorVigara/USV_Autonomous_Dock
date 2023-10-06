// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbzirc_interfaces:action/Dock.idl
// generated code does not contain a copyright notice

#ifndef MBZIRC_INTERFACES__ACTION__DETAIL__DOCK__STRUCT_H_
#define MBZIRC_INTERFACES__ACTION__DETAIL__DOCK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in action/Dock in the package mbzirc_interfaces.
typedef struct mbzirc_interfaces__action__Dock_Goal
{
  uint8_t structure_needs_at_least_one_member;
} mbzirc_interfaces__action__Dock_Goal;

// Struct for a sequence of mbzirc_interfaces__action__Dock_Goal.
typedef struct mbzirc_interfaces__action__Dock_Goal__Sequence
{
  mbzirc_interfaces__action__Dock_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbzirc_interfaces__action__Dock_Goal__Sequence;


// Constants defined in the message

// Struct defined in action/Dock in the package mbzirc_interfaces.
typedef struct mbzirc_interfaces__action__Dock_Result
{
  bool success;
} mbzirc_interfaces__action__Dock_Result;

// Struct for a sequence of mbzirc_interfaces__action__Dock_Result.
typedef struct mbzirc_interfaces__action__Dock_Result__Sequence
{
  mbzirc_interfaces__action__Dock_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbzirc_interfaces__action__Dock_Result__Sequence;


// Constants defined in the message

// Struct defined in action/Dock in the package mbzirc_interfaces.
typedef struct mbzirc_interfaces__action__Dock_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} mbzirc_interfaces__action__Dock_Feedback;

// Struct for a sequence of mbzirc_interfaces__action__Dock_Feedback.
typedef struct mbzirc_interfaces__action__Dock_Feedback__Sequence
{
  mbzirc_interfaces__action__Dock_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbzirc_interfaces__action__Dock_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "mbzirc_interfaces/action/detail/dock__struct.h"

// Struct defined in action/Dock in the package mbzirc_interfaces.
typedef struct mbzirc_interfaces__action__Dock_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  mbzirc_interfaces__action__Dock_Goal goal;
} mbzirc_interfaces__action__Dock_SendGoal_Request;

// Struct for a sequence of mbzirc_interfaces__action__Dock_SendGoal_Request.
typedef struct mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence
{
  mbzirc_interfaces__action__Dock_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/Dock in the package mbzirc_interfaces.
typedef struct mbzirc_interfaces__action__Dock_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} mbzirc_interfaces__action__Dock_SendGoal_Response;

// Struct for a sequence of mbzirc_interfaces__action__Dock_SendGoal_Response.
typedef struct mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence
{
  mbzirc_interfaces__action__Dock_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/Dock in the package mbzirc_interfaces.
typedef struct mbzirc_interfaces__action__Dock_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} mbzirc_interfaces__action__Dock_GetResult_Request;

// Struct for a sequence of mbzirc_interfaces__action__Dock_GetResult_Request.
typedef struct mbzirc_interfaces__action__Dock_GetResult_Request__Sequence
{
  mbzirc_interfaces__action__Dock_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbzirc_interfaces__action__Dock_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "mbzirc_interfaces/action/detail/dock__struct.h"

// Struct defined in action/Dock in the package mbzirc_interfaces.
typedef struct mbzirc_interfaces__action__Dock_GetResult_Response
{
  int8_t status;
  mbzirc_interfaces__action__Dock_Result result;
} mbzirc_interfaces__action__Dock_GetResult_Response;

// Struct for a sequence of mbzirc_interfaces__action__Dock_GetResult_Response.
typedef struct mbzirc_interfaces__action__Dock_GetResult_Response__Sequence
{
  mbzirc_interfaces__action__Dock_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbzirc_interfaces__action__Dock_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "mbzirc_interfaces/action/detail/dock__struct.h"

// Struct defined in action/Dock in the package mbzirc_interfaces.
typedef struct mbzirc_interfaces__action__Dock_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  mbzirc_interfaces__action__Dock_Feedback feedback;
} mbzirc_interfaces__action__Dock_FeedbackMessage;

// Struct for a sequence of mbzirc_interfaces__action__Dock_FeedbackMessage.
typedef struct mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence
{
  mbzirc_interfaces__action__Dock_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBZIRC_INTERFACES__ACTION__DETAIL__DOCK__STRUCT_H_
