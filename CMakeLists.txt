CMAKE_MINIMUM_REQUIRED(VERSION 2.6.3)

project(CLSQUARE_PACKAGE CXX C)

INCLUDE( ManageString.cmake )
INCLUDE( configuration.cmake.in )
MESSAGE( STATUS )

INCLUDE( cmake/CMakeLists.txt )

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
SET(CMAKE_INSTALL_PREFIX
  "${CLSQUARE_PACKAGE_SOURCE_DIR}" CACHE PATH "CLSQUARE install prefix" FORCE
)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-gnu")

OPTION( TOOLS "Set to OFF to prevent compilation of tools." ON )

MESSAGE( STATUS "-- CLSquare package options ---------------------------------------------------" )
MESSAGE( STATUS "CMAKE_INSTALL_PREFIX   = ${CMAKE_INSTALL_PREFIX}" )
MESSAGE( STATUS "TOOLS                  = ${TOOLS}" )
MESSAGE( STATUS "CMAKE_BUILD_TYPE       = ${CMAKE_BUILD_TYPE}")
MESSAGE( STATUS "Change a value with: cmake -D<VAR>=<VALUE>" )
MESSAGE( STATUS )

SET( TOOLS "${TOOLS}" CACHE BOOL "Set to OFF to prevent compilation of tools." FORCE )

add_subdirectory(src)
add_subdirectory(demos)

IF ( TOOLS )
	add_subdirectory(tools)
ENDIF()

# doc target
FIND_PACKAGE( Doxygen )
IF(DOXYGEN_FOUND)
  ADD_CUSTOM_TARGET(doc COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE} 
	                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
ENDIF()

MESSAGE( STATUS "-- Installation summary -------------------------------------------------------" )
IF ( ANY_MODULE_MISSING )
  MESSAGE ( STATUS "ATTENTION: Some modules were not built because of missing dependencies." )
  MESSAGE ( STATUS "Please check the listings above if everything you need was installed." )
  MESSAGE ( STATUS "" )
ENDIF ()
