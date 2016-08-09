CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )

FILE ( GLOB TOOLS * )

MESSAGE( STATUS "-- Building tools -------------------------------------------------------------" )
FOREACH ( TOOL ${TOOLS} )
  IF ( EXISTS ${TOOL}/CMakeLists.txt )
    ADD_SUBDIRECTORY ( ${TOOL} )
  ENDIF ()
ENDFOREACH ()
MESSAGE ( STATUS )
