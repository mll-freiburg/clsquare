cmake_minimum_required(VERSION 2.8)

# Init mono globals
macro(mono_init)
  set(MONO_GCS "mcs")
endmacro()

# Explode a list into "A B C .. "
function(mono_explode TARGET RETURN)
  set(RTN "")
  foreach(ITEM ${TARGET})
    set(RTN "${RTN} ${ITEM}")
  endforeach()
  set(${RETURN} ${RTN} PARENT_SCOPE)
endfunction()

# Creates a target called TARGET
# @param TARGET The name of the target
# @param SOURCE The set of source files
# @param LIB If this is a library 
function(add_mono_target TARGET SOURCES LIB)
  set(MONO_BUILD_TARGET "${PROJECT_BINARY_DIR}/builders/${TARGET}/build.c")
  set(MONO_BUILD_PATH "${PROJECT_BINARY_DIR}/builders/${TARGET}/")
  set(MONO_BUILD_RUNNER "${TARGET}_runner")
  set(MONO_BUILD_LIB "${TARGET}")

  file(MAKE_DIRECTORY ${MONO_BUILD_PATH})
  mono_explode("${SOURCES}" MONO_SRC)
  if(${LIB}) 
    set(MONO_BUILD_COMMAND "${MONO_GCS} ${MONO_SRC} -r:${MONO_RFLAGS} -target:library -out:${TARGET}.dll")
  else()
    set(MONO_BUILD_COMMAND "${MONO_GCS} ${MONO_SRC} -r:${MONO_RFLAGS} -out:${TARGET}.exe")

  endif()
  configure_file("${MONO_HELPER_DIR}/build.c.in" ${MONO_BUILD_TARGET})

  add_library(${MONO_BUILD_LIB} ${MONO_BUILD_TARGET})
  add_executable(${MONO_BUILD_RUNNER} "${MONO_HELPER_DIR}/runner.c")
  target_link_libraries(${MONO_BUILD_RUNNER} ${MONO_BUILD_LIB})

  get_target_property(MONO_RUNNER ${MONO_BUILD_RUNNER} LOCATION)
  get_filename_component(MONO_RUNNER ${MONO_RUNNER} ABSOLUTE)
  add_custom_command(TARGET ${MONO_BUILD_RUNNER} POST_BUILD COMMAND ${MONO_RUNNER})
  add_custom_command(TARGET ${MONO_BUILD_RUNNER} POST_BUILD COMMAND ${CMAKE_COMMAND} -E remove ${MONO_RUNNER})
endfunction()

# Creates a library called TARGET
# @param TARGET The name of the library
# @param SOURCE The set of source files
function(add_mono_library TARGET SOURCES)
  add_mono_target(${TARGET} "${SOURCES}" 1)
endfunction()

# Creates a binary called TARGET
# @param TARGET The name of the binary
# @param SOURCE The set of source files
function(add_mono_executable TARGET SOURCES)
  add_mono_target(${TARGET} "${SOURCES}" 0)
endfunction()
