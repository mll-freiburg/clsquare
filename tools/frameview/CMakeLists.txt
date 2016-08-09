CMAKE_MINIMUM_REQUIRED(VERSION 2.6.3)

find_package(X11)

if(X11_FOUND)

  project(FRAMEVIEW)

  add_executable(frameview 2dview.C area2d.C conv.C display_x11.C frame2d.C rgbcolor.C structure.C visobject.C rgb_db.C smonitor_dev.C ascii_dev.C joystick_dev.C builder_direct.C ascii_processor.C menu_x11.C tools.C conv_area2d.C popup_x11.C angle.C vector2d.C valueparser.C str2val.C udpsocket.C tcpsocket.C)

  target_link_libraries(frameview ${X11_LIBRARIES})

  INSTALL(TARGETS frameview 
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  message(STATUS "building application frameview")
else()
  message(STATUS "will not build application frameview: missing libx11-dev")
endif()
