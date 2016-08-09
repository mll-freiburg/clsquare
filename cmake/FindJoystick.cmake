# checks if linux/input.h and linux/joystick.h exist

find_path(LINUX_INPUT NAMES linux/input.h HINT /usr/include)
find_path(LINUX_JOYSTICK NAMES linux/joystick.h HINT /usr/include)
if(LINUX_INPUT AND LINUX_JOYSTICK)
  set (JOYSTICK_INCLUDE_DIR ${LINUX_JOYSTICK};${LINUX_INPUT})
  set (FOUND_JOYSTICK TRUE)
  mark_as_advanced(EIGEN3_INCLUDE_DIR)
endif()
