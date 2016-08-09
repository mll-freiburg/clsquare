SET ( lib_src
  carreracam.cpp
  tapir.cpp
  Socket.cc
)

SET ( lib_interfaces
  carreracam.h
  tapir.h
)

CHECK_INTERNAL ( KatanaSimCam FLI OPTIONAL )
IF ( WITHLIB_FLI )
  LIST ( APPEND lib_interfaces katanasimcam.h )
  LIST ( APPEND lib_src katanasimcam.cpp )
ENDIF ()

ADD_LIBRARY ( Tapir STATIC ${lib_src} )

#IF ( INSTALL_LIBS )
  INSTALL ( FILES ${lib_interfaces} DESTINATION include)
  INSTALL ( TARGETS Tapir
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
  )
  target_link_libraries ( Tapir )
#ENDIF ()
