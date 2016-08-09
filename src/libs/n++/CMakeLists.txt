SET ( lib_src n++.cc PatternSet.cc aux.cc )
SET ( lib_interfaces n++.h PatternSet.h aux.h )

ADD_LIBRARY ( n++ STATIC ${lib_src} )

#IF ( INSTALL_LIBS )
  INSTALL ( FILES ${lib_interfaces} DESTINATION include )
  INSTALL ( TARGETS n++
    RUNTIME DESTINATION lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
#ENDIF ()
