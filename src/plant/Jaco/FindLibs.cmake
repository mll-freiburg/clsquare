
find_path ( MONO_INCLUDE_DIR mono/jit/jit.h HINTS /opt/local/include /usr/local/include )
find_library ( MONO_LIBRARY NAMES libmono-2.0 HINTS  /opt/local/lib /usr/local/lib )
if ( MONO_LIBRARY AND MONO_INCLUDE_DIR )
  set ( FOUND_MONO TRUE )
  
else ()
  message ( WARNING " could not find MONO (LIBRARY=${MONO_LIBRARY}, INCLUDE=${MONO_INCLUDE_DIR})" )
endif ()
mark_as_advanced ( MONO_INCLUDE_DIR MONO_LIBRARY )

