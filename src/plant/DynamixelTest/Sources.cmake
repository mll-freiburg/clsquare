CHECK_INTERNAL ( DynamixelTest Dynamixel )

IF ( NOT MISSING_DEPENDENCY )
  LIST ( APPEND plant_srcs ${CURRENT_LIST_DIR}/encoderbased.cpp )
ENDIF ()
