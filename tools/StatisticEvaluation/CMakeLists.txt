CMAKE_MINIMUM_REQUIRED(VERSION 2.6.3)

project(STATEVAL)

add_executable(stateval stateval.cc)

INSTALL(TARGETS stateval
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
message(STATUS "building application StatisticsEvaluation")
