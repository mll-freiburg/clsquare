find_path ( CBLAS_INCLUDE_DIR NAMES cblas.h PATHS /opt/local/include /usr/local/include /usr/include )
find_library ( CBLAS_LIBRARY NAMES cblas HINTS /opt/local/lib /usr/local/lib )
if ( CBLAS_LIBRARY AND CBLAS_INCLUDE_DIR )
  set ( FOUND_CBLAS TRUE )
endif ()
