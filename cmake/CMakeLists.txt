### find optional packages
# 2.8 only, or did old ones have these as well?

# Variable naming conventions:
# For every library that was found, a variable FOUND_<lib> should be defined,
# with <lib> being the name that is passed to the linker, such as "boost" for
# libboost (as opposed to uppercase "Boost", which is set by the Find script).
# There should also be a compile definition of the form -DFOUND_<lib> of the
# same style, i.e. "-DFOUND_boost", though with any colons and hyphens replaced
# by an underscore. Use the FIND_ALIAS macro to automatically set both.
#
# General procedure:
#  FIND_PACKAGE( NAME1 )       # with NAME1 as in the name of the script
#                              # FindNAME1.cmake
#  FIND_RESULT ( NAME2 )       # with NAME2 as in the variable FOUND_NAME2 or
#                              # NAME2_FOUND set by the script
#  FIND_ALIAS  ( NAME2 ALIAS ) # with ALIAS being an arbitrary alternate name
#                              # you want to use
#  IF ( FOUND_NAME2 )
#    INCLUDE_DIRECTORIES ( ${INCLUDEPATH} )
#    LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${LIBPATH} )
#  ENDIF ()                    # with INCLUDEPATH and LIBPATH being the ones
#                              # set by the find script, usually
#                              # NAME2_INCLUDE_DIR and NAME2_LIBRARY

### utility macro for output
MACRO ( FIND_RESULT LIB )
  IF ( ${LIB}_FOUND )
    SET ( FOUND_${LIB} TRUE )
  ENDIF ()
  IF ( SUPPRESS_${LIB} )
    MESSAGE ( STATUS "actively suppressing ${LIB}" )
    SET ( FOUND_${LIB} FALSE )
  ELSEIF ( FOUND_${LIB} )
    FIND_DEFINITION ( ${LIB} )
    MESSAGE ( STATUS "found ${LIB}" )
  ELSE ()
    MESSAGE ( STATUS "could NOT find ${LIB}" )
  ENDIF ()
ENDMACRO ()

### utility macro for setting aliases
MACRO ( FIND_ALIAS LIB ALIAS )
  SET ( FOUND_${ALIAS} ${FOUND_${LIB}} )
  FIND_DEFINITION ( ${ALIAS} )
ENDMACRO ()

### utility macro to set definitions
MACRO ( FIND_DEFINITION NAME )
  IF ( FOUND_${NAME} )
    STRING ( REPLACE "." "_" TEMP_STR ${NAME} )
    STRING ( REPLACE "-" "_" TEMP_STR ${TEMP_STR} )
    ADD_DEFINITIONS ( -DFOUND_${TEMP_STR} )
  ENDIF ()
ENDMACRO ()

MESSAGE ( STATUS "-- Searching for additional packages ------------------------------------------" )

SET ( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH} )

### included in standard build environment, no check needed
LIST ( APPEND CLSQUARE_LIB_EXTERNAL "pthread" )

### included with cmake by default

FIND_PACKAGE( Boost COMPONENTS system regex thread ) # TODO also a local version present, anything different?
FIND_RESULT ( Boost )
FIND_ALIAS  ( Boost boost )

MACRO ( BOOST_MODULE_CHECK UPPER LOWER )
  FIND_RESULT ( Boost_${UPPER} )
  FIND_ALIAS  ( Boost_${UPPER} boost_${LOWER} )
  IF ( FOUND_Boost_${UPPER} )
    INCLUDE_DIRECTORIES ( ${BOOST_INCLUDEDIR} )
    LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${Boost_${UPPER}_LIBRARY} )
  ENDIF ()
ENDMACRO ()

BOOST_MODULE_CHECK ( THREAD thread )
BOOST_MODULE_CHECK ( SYSTEM system )
BOOST_MODULE_CHECK ( REGEX regex )

FIND_PACKAGE( GTest )
FIND_RESULT ( GTEST )
FIND_ALIAS  ( GTEST GTest )
IF ( FOUND_GTEST )
  INCLUDE_DIRECTORIES ( ${GTEST_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${GTEST_BOTH_LIBRARIES} )
ENDIF ()

FIND_PACKAGE( X11 )
FIND_RESULT ( X11 )
FIND_ALIAS  ( X11 x11 )
IF ( FOUND_X11 )
  INCLUDE_DIRECTORIES ( ${X11_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${X11_LIBRARIES} )
ENDIF ()

### local scripts

FIND_PACKAGE( ANN )
FIND_RESULT ( ANN )
IF ( FOUND_ANN )
  INCLUDE_DIRECTORIES ( ${ANN_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${ANN_LIBRARY} )
ENDIF ()

# BLAS also has a default script in 2.8, but that one does not always work
FIND_PACKAGE( BLAS )
FIND_RESULT ( BLAS )
FIND_ALIAS  ( BLAS blas )
IF ( FOUND_BLAS )
  INCLUDE_DIRECTORIES ( ${BLAS_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${BLAS_LIBRARIES} )
ENDIF ()

FIND_PACKAGE( CBLAS )
FIND_RESULT ( CBLAS )
FIND_ALIAS  ( CBLAS cblas )
IF ( FOUND_CBLAS )
  INCLUDE_DIRECTORIES ( ${CBLAS_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${CBLAS_LIBRARY} )
ENDIF ()

FIND_PACKAGE( DC1394 )
FIND_RESULT ( DC1394 )
FIND_ALIAS  ( DC1394 dc1394 )
IF ( FOUND_DC1394 )
  INCLUDE_DIRECTORIES ( ${DC1394_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${DC1394_LIBRARY} )
ENDIF ()

FIND_PACKAGE( Eigen3 )
FIND_RESULT ( EIGEN3 )
FIND_ALIAS  ( EIGEN3 Eigen3 )
IF ( FOUND_EIGEN3 )
  INCLUDE_DIRECTORIES ( ${EIGEN3_INCLUDE_DIR} )
  # Eigen3 is header-only
ENDIF ()

FIND_PACKAGE( FTD2XX )
FIND_RESULT ( FTD2XX )
FIND_ALIAS  ( FTD2XX ftd2xx )
IF ( FOUND_FTD2XX )
  INCLUDE_DIRECTORIES ( ${FTD2XX_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${FTD2XX_LIBRARY} )
ENDIF ()

FIND_PACKAGE( FTDI )
FIND_RESULT ( FTDI )
FIND_ALIAS  ( FTDI ftdi )
IF ( FOUND_FTDI )
  INCLUDE_DIRECTORIES ( ${FTDI_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${FTDI_LIBRARY} )
ENDIF ()

FIND_PACKAGE( Mono2 )
FIND_RESULT ( MONO2 )
FIND_ALIAS  ( MONO2 mono-2.0 )
IF ( FOUND_MONO2 )
  INCLUDE_DIRECTORIES ( ${MONO2_INCLUDE_DIR}/mono-2.0 )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${MONO2_LIBRARY} )
ENDIF ()

FIND_PACKAGE( Plotter )
FIND_RESULT ( PLOTTER )
FIND_ALIAS  ( PLOTTER plotter )
IF ( FOUND_PLOTTER )
  INCLUDE_DIRECTORIES ( ${PLOTTER_INCLUDE_DIRS} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${PLOTTER_LIBRARIES} )
ENDIF ()

FIND_PACKAGE( PCan )
FIND_RESULT ( PCAN )
FIND_ALIAS  ( PCAN pcan )
IF ( FOUND_PCAN )
  INCLUDE_DIRECTORIES ( ${PCAN_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${PCAN_LIBRARY} )
ENDIF ()

FIND_PACKAGE( Usb )
FIND_RESULT ( USB )
FIND_ALIAS  ( USB usb )
FIND_ALIAS  ( USB usb-0.1 )
IF ( FOUND_USB )
  INCLUDE_DIRECTORIES ( ${USB_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${USB_LIBRARY} )
ENDIF ()

FIND_PACKAGE( Usb-1.0 )
FIND_RESULT ( USB_1 )
FIND_ALIAS  ( USB_1 usb-1.0 )
IF ( FOUND_USB_1 )
  INCLUDE_DIRECTORIES ( ${USB_1_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${USB_1_LIBRARY} )
ENDIF ()

FIND_PACKAGE( Joystick )
FIND_RESULT ( JOYSTICK )
FIND_ALIAS  ( JOYSTICK Joystick )
IF ( FOUND_JOYSTICK )
  INCLUDE_DIRECTORIES ( ${JOYSTICK_INCLUDE_DIR} )
ENDIF ()

FIND_PACKAGE( HIDAPI )
FIND_RESULT ( HIDAPI )
FIND_ALIAS  ( HIDAPI hidapi )
IF ( FOUND_HIDAPI )
  INCLUDE_DIRECTORIES ( ${HIDAPI_INCLUDE_DIRS} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${HIDAPI_LIBRARIES} )
  LINK_DIRECTORIES(/usr/local/lib)
ENDIF ()

SET ( Mcrypt_FIND_QUIETLY TRUE )
FIND_PACKAGE( Mcrypt )
FIND_RESULT ( Mcrypt )
FIND_ALIAS  ( Mcrypt MCrypt )
IF ( FOUND_Mcrypt )
  INCLUDE_DIRECTORIES ( ${Mcrypt_INCLUDE_DIR} )
  LIST ( APPEND CLSQUARE_LIB_EXTERNAL ${Mcrypt_LIBS} )
ENDIF ()

