CHECK_EXTERNAL ( Dynamixel ftd2xx OPTIONAL )
CHECK_EXTERNAL ( Dynamixel ftdi OPTIONAL )
CHECK_EXTERNAL ( Dynamixel boost OPTIONAL )

SET ( lib_src
  dynamixel.cpp
  robotis.cpp
  morpheus.cpp
  ftdidevice.cpp
  ftdxxdevice.cpp
  dynabutler.cpp
  bioloid.cpp
  simloid.cpp
)

SET ( lib_interfaces
  dynamixel_all.h
  dynamixel.h
  ftdevice.h
  loid.h
)

ADD_LIBRARY(Dynamixel STATIC ${lib_src})

#IF ( INSTALL_LIBS )
  INSTALL ( FILES ${lib_interfaces} DESTINATION include )
  INSTALL ( TARGETS Dynamixel
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  add_executable ( DynamixelTest test.cpp )
  target_link_libraries ( DynamixelTest Dynamixel ${CLSQUARE_LIB_EXTERNAL} )
  INSTALL ( TARGETS DynamixelTest
    RUNTIME DESTINATION bin
  )
#ENDIF ()
