// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mbzirc_interfaces:action/Dock.idl
// generated code does not contain a copyright notice
#include "mbzirc_interfaces/action/detail/dock__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
mbzirc_interfaces__action__Dock_Goal__init(mbzirc_interfaces__action__Dock_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
mbzirc_interfaces__action__Dock_Goal__fini(mbzirc_interfaces__action__Dock_Goal * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
mbzirc_interfaces__action__Dock_Goal__are_equal(const mbzirc_interfaces__action__Dock_Goal * lhs, const mbzirc_interfaces__action__Dock_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_Goal__copy(
  const mbzirc_interfaces__action__Dock_Goal * input,
  mbzirc_interfaces__action__Dock_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

mbzirc_interfaces__action__Dock_Goal *
mbzirc_interfaces__action__Dock_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_Goal * msg = (mbzirc_interfaces__action__Dock_Goal *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbzirc_interfaces__action__Dock_Goal));
  bool success = mbzirc_interfaces__action__Dock_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbzirc_interfaces__action__Dock_Goal__destroy(mbzirc_interfaces__action__Dock_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbzirc_interfaces__action__Dock_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbzirc_interfaces__action__Dock_Goal__Sequence__init(mbzirc_interfaces__action__Dock_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_Goal * data = NULL;

  if (size) {
    data = (mbzirc_interfaces__action__Dock_Goal *)allocator.zero_allocate(size, sizeof(mbzirc_interfaces__action__Dock_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbzirc_interfaces__action__Dock_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbzirc_interfaces__action__Dock_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbzirc_interfaces__action__Dock_Goal__Sequence__fini(mbzirc_interfaces__action__Dock_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbzirc_interfaces__action__Dock_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbzirc_interfaces__action__Dock_Goal__Sequence *
mbzirc_interfaces__action__Dock_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_Goal__Sequence * array = (mbzirc_interfaces__action__Dock_Goal__Sequence *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbzirc_interfaces__action__Dock_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbzirc_interfaces__action__Dock_Goal__Sequence__destroy(mbzirc_interfaces__action__Dock_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbzirc_interfaces__action__Dock_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbzirc_interfaces__action__Dock_Goal__Sequence__are_equal(const mbzirc_interfaces__action__Dock_Goal__Sequence * lhs, const mbzirc_interfaces__action__Dock_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_Goal__Sequence__copy(
  const mbzirc_interfaces__action__Dock_Goal__Sequence * input,
  mbzirc_interfaces__action__Dock_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbzirc_interfaces__action__Dock_Goal);
    mbzirc_interfaces__action__Dock_Goal * data =
      (mbzirc_interfaces__action__Dock_Goal *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbzirc_interfaces__action__Dock_Goal__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mbzirc_interfaces__action__Dock_Goal__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
mbzirc_interfaces__action__Dock_Result__init(mbzirc_interfaces__action__Dock_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
mbzirc_interfaces__action__Dock_Result__fini(mbzirc_interfaces__action__Dock_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
mbzirc_interfaces__action__Dock_Result__are_equal(const mbzirc_interfaces__action__Dock_Result * lhs, const mbzirc_interfaces__action__Dock_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_Result__copy(
  const mbzirc_interfaces__action__Dock_Result * input,
  mbzirc_interfaces__action__Dock_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

mbzirc_interfaces__action__Dock_Result *
mbzirc_interfaces__action__Dock_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_Result * msg = (mbzirc_interfaces__action__Dock_Result *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbzirc_interfaces__action__Dock_Result));
  bool success = mbzirc_interfaces__action__Dock_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbzirc_interfaces__action__Dock_Result__destroy(mbzirc_interfaces__action__Dock_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbzirc_interfaces__action__Dock_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbzirc_interfaces__action__Dock_Result__Sequence__init(mbzirc_interfaces__action__Dock_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_Result * data = NULL;

  if (size) {
    data = (mbzirc_interfaces__action__Dock_Result *)allocator.zero_allocate(size, sizeof(mbzirc_interfaces__action__Dock_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbzirc_interfaces__action__Dock_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbzirc_interfaces__action__Dock_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbzirc_interfaces__action__Dock_Result__Sequence__fini(mbzirc_interfaces__action__Dock_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbzirc_interfaces__action__Dock_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbzirc_interfaces__action__Dock_Result__Sequence *
mbzirc_interfaces__action__Dock_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_Result__Sequence * array = (mbzirc_interfaces__action__Dock_Result__Sequence *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbzirc_interfaces__action__Dock_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbzirc_interfaces__action__Dock_Result__Sequence__destroy(mbzirc_interfaces__action__Dock_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbzirc_interfaces__action__Dock_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbzirc_interfaces__action__Dock_Result__Sequence__are_equal(const mbzirc_interfaces__action__Dock_Result__Sequence * lhs, const mbzirc_interfaces__action__Dock_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_Result__Sequence__copy(
  const mbzirc_interfaces__action__Dock_Result__Sequence * input,
  mbzirc_interfaces__action__Dock_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbzirc_interfaces__action__Dock_Result);
    mbzirc_interfaces__action__Dock_Result * data =
      (mbzirc_interfaces__action__Dock_Result *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbzirc_interfaces__action__Dock_Result__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mbzirc_interfaces__action__Dock_Result__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
mbzirc_interfaces__action__Dock_Feedback__init(mbzirc_interfaces__action__Dock_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
mbzirc_interfaces__action__Dock_Feedback__fini(mbzirc_interfaces__action__Dock_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
mbzirc_interfaces__action__Dock_Feedback__are_equal(const mbzirc_interfaces__action__Dock_Feedback * lhs, const mbzirc_interfaces__action__Dock_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_Feedback__copy(
  const mbzirc_interfaces__action__Dock_Feedback * input,
  mbzirc_interfaces__action__Dock_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

mbzirc_interfaces__action__Dock_Feedback *
mbzirc_interfaces__action__Dock_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_Feedback * msg = (mbzirc_interfaces__action__Dock_Feedback *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbzirc_interfaces__action__Dock_Feedback));
  bool success = mbzirc_interfaces__action__Dock_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbzirc_interfaces__action__Dock_Feedback__destroy(mbzirc_interfaces__action__Dock_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbzirc_interfaces__action__Dock_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbzirc_interfaces__action__Dock_Feedback__Sequence__init(mbzirc_interfaces__action__Dock_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_Feedback * data = NULL;

  if (size) {
    data = (mbzirc_interfaces__action__Dock_Feedback *)allocator.zero_allocate(size, sizeof(mbzirc_interfaces__action__Dock_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbzirc_interfaces__action__Dock_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbzirc_interfaces__action__Dock_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbzirc_interfaces__action__Dock_Feedback__Sequence__fini(mbzirc_interfaces__action__Dock_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbzirc_interfaces__action__Dock_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbzirc_interfaces__action__Dock_Feedback__Sequence *
mbzirc_interfaces__action__Dock_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_Feedback__Sequence * array = (mbzirc_interfaces__action__Dock_Feedback__Sequence *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbzirc_interfaces__action__Dock_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbzirc_interfaces__action__Dock_Feedback__Sequence__destroy(mbzirc_interfaces__action__Dock_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbzirc_interfaces__action__Dock_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbzirc_interfaces__action__Dock_Feedback__Sequence__are_equal(const mbzirc_interfaces__action__Dock_Feedback__Sequence * lhs, const mbzirc_interfaces__action__Dock_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_Feedback__Sequence__copy(
  const mbzirc_interfaces__action__Dock_Feedback__Sequence * input,
  mbzirc_interfaces__action__Dock_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbzirc_interfaces__action__Dock_Feedback);
    mbzirc_interfaces__action__Dock_Feedback * data =
      (mbzirc_interfaces__action__Dock_Feedback *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbzirc_interfaces__action__Dock_Feedback__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mbzirc_interfaces__action__Dock_Feedback__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "mbzirc_interfaces/action/detail/dock__functions.h"

bool
mbzirc_interfaces__action__Dock_SendGoal_Request__init(mbzirc_interfaces__action__Dock_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    mbzirc_interfaces__action__Dock_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!mbzirc_interfaces__action__Dock_Goal__init(&msg->goal)) {
    mbzirc_interfaces__action__Dock_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
mbzirc_interfaces__action__Dock_SendGoal_Request__fini(mbzirc_interfaces__action__Dock_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  mbzirc_interfaces__action__Dock_Goal__fini(&msg->goal);
}

bool
mbzirc_interfaces__action__Dock_SendGoal_Request__are_equal(const mbzirc_interfaces__action__Dock_SendGoal_Request * lhs, const mbzirc_interfaces__action__Dock_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!mbzirc_interfaces__action__Dock_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_SendGoal_Request__copy(
  const mbzirc_interfaces__action__Dock_SendGoal_Request * input,
  mbzirc_interfaces__action__Dock_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!mbzirc_interfaces__action__Dock_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

mbzirc_interfaces__action__Dock_SendGoal_Request *
mbzirc_interfaces__action__Dock_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_SendGoal_Request * msg = (mbzirc_interfaces__action__Dock_SendGoal_Request *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbzirc_interfaces__action__Dock_SendGoal_Request));
  bool success = mbzirc_interfaces__action__Dock_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbzirc_interfaces__action__Dock_SendGoal_Request__destroy(mbzirc_interfaces__action__Dock_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbzirc_interfaces__action__Dock_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence__init(mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_SendGoal_Request * data = NULL;

  if (size) {
    data = (mbzirc_interfaces__action__Dock_SendGoal_Request *)allocator.zero_allocate(size, sizeof(mbzirc_interfaces__action__Dock_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbzirc_interfaces__action__Dock_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbzirc_interfaces__action__Dock_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence__fini(mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbzirc_interfaces__action__Dock_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence *
mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence * array = (mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence__destroy(mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence__are_equal(const mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence * lhs, const mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence__copy(
  const mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence * input,
  mbzirc_interfaces__action__Dock_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbzirc_interfaces__action__Dock_SendGoal_Request);
    mbzirc_interfaces__action__Dock_SendGoal_Request * data =
      (mbzirc_interfaces__action__Dock_SendGoal_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbzirc_interfaces__action__Dock_SendGoal_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mbzirc_interfaces__action__Dock_SendGoal_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
mbzirc_interfaces__action__Dock_SendGoal_Response__init(mbzirc_interfaces__action__Dock_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    mbzirc_interfaces__action__Dock_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
mbzirc_interfaces__action__Dock_SendGoal_Response__fini(mbzirc_interfaces__action__Dock_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
mbzirc_interfaces__action__Dock_SendGoal_Response__are_equal(const mbzirc_interfaces__action__Dock_SendGoal_Response * lhs, const mbzirc_interfaces__action__Dock_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_SendGoal_Response__copy(
  const mbzirc_interfaces__action__Dock_SendGoal_Response * input,
  mbzirc_interfaces__action__Dock_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

mbzirc_interfaces__action__Dock_SendGoal_Response *
mbzirc_interfaces__action__Dock_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_SendGoal_Response * msg = (mbzirc_interfaces__action__Dock_SendGoal_Response *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbzirc_interfaces__action__Dock_SendGoal_Response));
  bool success = mbzirc_interfaces__action__Dock_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbzirc_interfaces__action__Dock_SendGoal_Response__destroy(mbzirc_interfaces__action__Dock_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbzirc_interfaces__action__Dock_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence__init(mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_SendGoal_Response * data = NULL;

  if (size) {
    data = (mbzirc_interfaces__action__Dock_SendGoal_Response *)allocator.zero_allocate(size, sizeof(mbzirc_interfaces__action__Dock_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbzirc_interfaces__action__Dock_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbzirc_interfaces__action__Dock_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence__fini(mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbzirc_interfaces__action__Dock_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence *
mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence * array = (mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence__destroy(mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence__are_equal(const mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence * lhs, const mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence__copy(
  const mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence * input,
  mbzirc_interfaces__action__Dock_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbzirc_interfaces__action__Dock_SendGoal_Response);
    mbzirc_interfaces__action__Dock_SendGoal_Response * data =
      (mbzirc_interfaces__action__Dock_SendGoal_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbzirc_interfaces__action__Dock_SendGoal_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mbzirc_interfaces__action__Dock_SendGoal_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
mbzirc_interfaces__action__Dock_GetResult_Request__init(mbzirc_interfaces__action__Dock_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    mbzirc_interfaces__action__Dock_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
mbzirc_interfaces__action__Dock_GetResult_Request__fini(mbzirc_interfaces__action__Dock_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
mbzirc_interfaces__action__Dock_GetResult_Request__are_equal(const mbzirc_interfaces__action__Dock_GetResult_Request * lhs, const mbzirc_interfaces__action__Dock_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_GetResult_Request__copy(
  const mbzirc_interfaces__action__Dock_GetResult_Request * input,
  mbzirc_interfaces__action__Dock_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

mbzirc_interfaces__action__Dock_GetResult_Request *
mbzirc_interfaces__action__Dock_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_GetResult_Request * msg = (mbzirc_interfaces__action__Dock_GetResult_Request *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbzirc_interfaces__action__Dock_GetResult_Request));
  bool success = mbzirc_interfaces__action__Dock_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbzirc_interfaces__action__Dock_GetResult_Request__destroy(mbzirc_interfaces__action__Dock_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbzirc_interfaces__action__Dock_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbzirc_interfaces__action__Dock_GetResult_Request__Sequence__init(mbzirc_interfaces__action__Dock_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_GetResult_Request * data = NULL;

  if (size) {
    data = (mbzirc_interfaces__action__Dock_GetResult_Request *)allocator.zero_allocate(size, sizeof(mbzirc_interfaces__action__Dock_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbzirc_interfaces__action__Dock_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbzirc_interfaces__action__Dock_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbzirc_interfaces__action__Dock_GetResult_Request__Sequence__fini(mbzirc_interfaces__action__Dock_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbzirc_interfaces__action__Dock_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbzirc_interfaces__action__Dock_GetResult_Request__Sequence *
mbzirc_interfaces__action__Dock_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_GetResult_Request__Sequence * array = (mbzirc_interfaces__action__Dock_GetResult_Request__Sequence *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbzirc_interfaces__action__Dock_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbzirc_interfaces__action__Dock_GetResult_Request__Sequence__destroy(mbzirc_interfaces__action__Dock_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbzirc_interfaces__action__Dock_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbzirc_interfaces__action__Dock_GetResult_Request__Sequence__are_equal(const mbzirc_interfaces__action__Dock_GetResult_Request__Sequence * lhs, const mbzirc_interfaces__action__Dock_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_GetResult_Request__Sequence__copy(
  const mbzirc_interfaces__action__Dock_GetResult_Request__Sequence * input,
  mbzirc_interfaces__action__Dock_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbzirc_interfaces__action__Dock_GetResult_Request);
    mbzirc_interfaces__action__Dock_GetResult_Request * data =
      (mbzirc_interfaces__action__Dock_GetResult_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbzirc_interfaces__action__Dock_GetResult_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mbzirc_interfaces__action__Dock_GetResult_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "mbzirc_interfaces/action/detail/dock__functions.h"

bool
mbzirc_interfaces__action__Dock_GetResult_Response__init(mbzirc_interfaces__action__Dock_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!mbzirc_interfaces__action__Dock_Result__init(&msg->result)) {
    mbzirc_interfaces__action__Dock_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
mbzirc_interfaces__action__Dock_GetResult_Response__fini(mbzirc_interfaces__action__Dock_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  mbzirc_interfaces__action__Dock_Result__fini(&msg->result);
}

bool
mbzirc_interfaces__action__Dock_GetResult_Response__are_equal(const mbzirc_interfaces__action__Dock_GetResult_Response * lhs, const mbzirc_interfaces__action__Dock_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!mbzirc_interfaces__action__Dock_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_GetResult_Response__copy(
  const mbzirc_interfaces__action__Dock_GetResult_Response * input,
  mbzirc_interfaces__action__Dock_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!mbzirc_interfaces__action__Dock_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

mbzirc_interfaces__action__Dock_GetResult_Response *
mbzirc_interfaces__action__Dock_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_GetResult_Response * msg = (mbzirc_interfaces__action__Dock_GetResult_Response *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbzirc_interfaces__action__Dock_GetResult_Response));
  bool success = mbzirc_interfaces__action__Dock_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbzirc_interfaces__action__Dock_GetResult_Response__destroy(mbzirc_interfaces__action__Dock_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbzirc_interfaces__action__Dock_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbzirc_interfaces__action__Dock_GetResult_Response__Sequence__init(mbzirc_interfaces__action__Dock_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_GetResult_Response * data = NULL;

  if (size) {
    data = (mbzirc_interfaces__action__Dock_GetResult_Response *)allocator.zero_allocate(size, sizeof(mbzirc_interfaces__action__Dock_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbzirc_interfaces__action__Dock_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbzirc_interfaces__action__Dock_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbzirc_interfaces__action__Dock_GetResult_Response__Sequence__fini(mbzirc_interfaces__action__Dock_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbzirc_interfaces__action__Dock_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbzirc_interfaces__action__Dock_GetResult_Response__Sequence *
mbzirc_interfaces__action__Dock_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_GetResult_Response__Sequence * array = (mbzirc_interfaces__action__Dock_GetResult_Response__Sequence *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbzirc_interfaces__action__Dock_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbzirc_interfaces__action__Dock_GetResult_Response__Sequence__destroy(mbzirc_interfaces__action__Dock_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbzirc_interfaces__action__Dock_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbzirc_interfaces__action__Dock_GetResult_Response__Sequence__are_equal(const mbzirc_interfaces__action__Dock_GetResult_Response__Sequence * lhs, const mbzirc_interfaces__action__Dock_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_GetResult_Response__Sequence__copy(
  const mbzirc_interfaces__action__Dock_GetResult_Response__Sequence * input,
  mbzirc_interfaces__action__Dock_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbzirc_interfaces__action__Dock_GetResult_Response);
    mbzirc_interfaces__action__Dock_GetResult_Response * data =
      (mbzirc_interfaces__action__Dock_GetResult_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbzirc_interfaces__action__Dock_GetResult_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mbzirc_interfaces__action__Dock_GetResult_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "mbzirc_interfaces/action/detail/dock__functions.h"

bool
mbzirc_interfaces__action__Dock_FeedbackMessage__init(mbzirc_interfaces__action__Dock_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    mbzirc_interfaces__action__Dock_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!mbzirc_interfaces__action__Dock_Feedback__init(&msg->feedback)) {
    mbzirc_interfaces__action__Dock_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
mbzirc_interfaces__action__Dock_FeedbackMessage__fini(mbzirc_interfaces__action__Dock_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  mbzirc_interfaces__action__Dock_Feedback__fini(&msg->feedback);
}

bool
mbzirc_interfaces__action__Dock_FeedbackMessage__are_equal(const mbzirc_interfaces__action__Dock_FeedbackMessage * lhs, const mbzirc_interfaces__action__Dock_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!mbzirc_interfaces__action__Dock_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_FeedbackMessage__copy(
  const mbzirc_interfaces__action__Dock_FeedbackMessage * input,
  mbzirc_interfaces__action__Dock_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!mbzirc_interfaces__action__Dock_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

mbzirc_interfaces__action__Dock_FeedbackMessage *
mbzirc_interfaces__action__Dock_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_FeedbackMessage * msg = (mbzirc_interfaces__action__Dock_FeedbackMessage *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbzirc_interfaces__action__Dock_FeedbackMessage));
  bool success = mbzirc_interfaces__action__Dock_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbzirc_interfaces__action__Dock_FeedbackMessage__destroy(mbzirc_interfaces__action__Dock_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbzirc_interfaces__action__Dock_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence__init(mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_FeedbackMessage * data = NULL;

  if (size) {
    data = (mbzirc_interfaces__action__Dock_FeedbackMessage *)allocator.zero_allocate(size, sizeof(mbzirc_interfaces__action__Dock_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbzirc_interfaces__action__Dock_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbzirc_interfaces__action__Dock_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence__fini(mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbzirc_interfaces__action__Dock_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence *
mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence * array = (mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence *)allocator.allocate(sizeof(mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence__destroy(mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence__are_equal(const mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence * lhs, const mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence__copy(
  const mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence * input,
  mbzirc_interfaces__action__Dock_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbzirc_interfaces__action__Dock_FeedbackMessage);
    mbzirc_interfaces__action__Dock_FeedbackMessage * data =
      (mbzirc_interfaces__action__Dock_FeedbackMessage *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbzirc_interfaces__action__Dock_FeedbackMessage__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mbzirc_interfaces__action__Dock_FeedbackMessage__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbzirc_interfaces__action__Dock_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
