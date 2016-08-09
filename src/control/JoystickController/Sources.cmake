CHECK_EXTERNAL(JoystickController Joystick)

IF(NOT MISSING_DEPENDENCY)
  INCLUDE_DIRECTORIES(${CURRENT_LIST_DIR})
  LIST(APPEND controller_srcs 
    ${CURRENT_LIST_DIR}/Joystick.cc
    ${CURRENT_LIST_DIR}/joystickcontroller.cpp
    ${CURRENT_LIST_DIR}/joystickcontroller_ex.cpp
  )
ENDIF()
