# search for proprietary FTD2xx lib
find_path ( FTD2XX_INCLUDE_DIR ftd2xx.h HINTS /opt/local/include /usr/local/include )
find_library ( FTD2XX_LIBRARY NAMES ftd2xx
             HINTS  /opt/local/lib /usr/local/lib )
if ( FTD2XX_LIBRARY AND FTD2XX_INCLUDE_DIR )
  set ( FOUND_FTD2XX TRUE )
#else ()
#  message ( WARNING " could not find FTD2xx (LIBRARY=${FTD2XX_LIBRARY}, INCLUDE=${FTD2XX_INCLUDE_DIR})" )
endif ()
mark_as_advanced(FTD2XX_INCLUDE_DIR FTD2XX_LIBRARY )
