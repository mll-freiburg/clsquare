project(GraphicalUserInput) # the name of your project

cmake_minimum_required(VERSION 2.6.3)

FIND_PACKAGE( Qt4 )

IF ( QT_FOUND )

  SET(QT_USE_QTNETWORK TRUE)

  include(${QT_USE_FILE})

  SET(GraphicalUserInput_SRCS 
    inputwidget.cpp
    main.cpp
    mainwindow.cpp
  )

  SET(GraphicalUserInput_MOC_HDRS 
    inputwidget.h 
    mainwindow.h
  )

  SET(GraphicalUserInput_UIS
      inputwidget.ui
      mainwindow.ui
  )

  QT4_WRAP_CPP(GraphicalUserInput_MOC_SRCS ${GraphicalUserInput_MOC_HDRS})

  QT4_WRAP_UI(GraphicalUserInput_UIS_H ${GraphicalUserInput_UIS})

  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  add_executable(GraphicalUserInput ${GraphicalUserInput_SRCS} ${GraphicalUserInput_MOC_SRCS} ${GraphicalUserInput_UIS_H})
  target_link_libraries(GraphicalUserInput ${QT_LIBRARIES})
  INSTALL(TARGETS GraphicalUserInput
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  add_executable(GraphicalUserInputTestClient testclient/main.cc testclient/Socket.cc)
  target_link_libraries(GraphicalUserInputTestClient ${QT_LIBRARIES})
  INSTALL(TARGETS GraphicalUserInputTestClient
    RUNTIME DESTINATION bin
  )

    message(STATUS "building application GraphicalUserInput")
  ELSE()
    message(STATUS "will not build application GraphicalUserInput: missing Qt4")

ENDIF()
