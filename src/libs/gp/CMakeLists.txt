# Created by Manuel Blum on 2011-05-25.
# Copyright 2011 University of Freiburg.

CHECK_EXTERNAL(gp Eigen3)
IF(NOT MISSING_DEPENDENCY)

  INCLUDE(ExternalProject)

  # ----- Download and build gtest -----
  ExternalProject_Add(libgp
    URL http://hg.bluman.de/libgp/get/libgp-0.1.4.tar.gz
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
      -DBUILD_TESTS:BOOL=OFF 
      -DBUILD_EXAMPLES:BOOL=OFF
      -DCMAKE_BUILD_TYPE=Release
      -DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}
    
    #INSTALL_COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/src/libgp-build/libgp.a ${CMAKE_CURRENT_BINARY_DIR}
    #INSTALL_DIR ${CMAKE_SOURCE_DIR}
    PREFIX "."
  )

  #LINK_DIRECTORIES (${CMAKE_CURRENT_BINARY_DIR}/src/libgp-build)
  #MESSAGE (Libpath:  ${CMAKE_CURRENT_BINARY_DIR}/src/libgp-build)

  LIST(APPEND CLSQUARE_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/src/libgp/include)
  SET(CLSQUARE_INCLUDE_DIRS ${CLSQUARE_INCLUDE_DIRS} PARENT_SCOPE)

ENDIF()
