# search for open source FTDI lib
find_path ( FTDI_INCLUDE_DIR ftdi.h HINTS /opt/local/include /usr/local/include )
find_library ( FTDI_LIBRARY NAMES ftdi
             HINTS  /opt/local/lib /usr/local/lib )
if ( FTDI_LIBRARY AND FTDI_INCLUDE_DIR )
  set ( FOUND_FTDI TRUE )
#else ()
#  message ( WARNING " could not find FTDI (LIBRARY=${FTDI_LIBRARY}, INCLUDE=${FTDI_INCLUDE_DIR})" )
endif ()
mark_as_advanced ( FTDI_INCLUDE_DIR FTDI_LIBRARY )
