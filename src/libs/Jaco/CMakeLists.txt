CHECK_EXTERNAL ( Jaco mono-2.0 )

# find all Kinova libraries; we don't use CHECK_EXTERNAL here since they are
# DLLs and should not be available normally
if ( NOT MISSING_DEPENDENCY )

  set ( NEEDED_DLLS "Kinova.API.Jaco.dll;Kinova.DLL.Data.dll;Kinova.DLL.SafeGate.dll;Kinova.DLL.TestData.dll;Kinova.DLL.Tools.dll;Kinova.DLL.USBManager.dll;LibUsbDotNet.dll" )
  foreach ( DLL ${NEEDED_DLLS} )
    find_path (
      ${DLL}_DIR
      "${DLL}"
      HINTS
        /opt/local/include
        /usr/local/include
        ${CMAKE_INSTALL_PREFIX}/src/libs/Jaco
        ${CMAKE_INSTALL_PREFIX}/src/libs/Jaco/dll
    )
    if ( ${DLL}_DIR )
      list ( APPEND MONO_DLLS ${${DLL}_DIR}/${DLL} )
      set ( MONO_RFLAGS "${MONO_RFLAGS},${${DLL}_DIR}/${DLL}" )
    else ()
      list ( APPEND MONO_NOTFOUND ${DLL} )
      set ( MISSING_DEPENDENCY TRUE )
    endif ()
  endforeach ()

  if ( MISSING_DEPENDENCY )
    message ( STATUS "skipping module Jaco because some mono DLLs are missing (${MONO_NOTFOUND})" )
    message ( STATUS "  please ensure that all libraries are located at a location where CMake can find them," )
    message ( STATUS "  such as /opt/local/include or ${CMAKE_INSTALL_PREFIX}/src/libs/Jaco" )
    set ( MISSING_DEPENDENCY TRUE PARENT_SCOPE )
    set ( WITHLIB_Jaco FALSE PARENT_SCOPE )
  endif ()

endif ()

if ( NOT MISSING_DEPENDENCY )

  # call wrapper that compiles the mono application
  set ( MONO_HELPER_DIR "${CURRENT_LIST_DIR}/helpers" )
  include ( ${MONO_HELPER_DIR}/MonoWrapper.cmake )
  mono_init ()

  # copy libs and link to not-yet-existent binary
  set ( JACO_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib" )
  file ( COPY ${MONO_DLLS} DESTINATION ${JACO_INSTALL_DIR} )
  execute_process ( COMMAND ${CMAKE_COMMAND} -E create_symlink ${CLSQUARE_BINARY_DIR}/libs/Jaco/Jaco.exe "${CMAKE_INSTALL_PREFIX}/lib/Jaco.exe" )

  # set up mono compiler
  string ( REGEX REPLACE "^," "" MONO_RFLAGS ${MONO_RFLAGS} )
  add_mono_executable(Jaco "${CURRENT_LIST_DIR}/interface.cs")

endif ()
