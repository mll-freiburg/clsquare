CMAKE_MINIMUM_REQUIRED(VERSION 2.6.3)

project(RANDOMGEN)

add_executable(randomgen randomgen.cc)


INSTALL(TARGETS randomgen
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
message(STATUS "building application RandomGen")
