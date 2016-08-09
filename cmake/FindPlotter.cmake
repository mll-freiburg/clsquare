# - Try to find libplotter
# Once done this will define
#  PLOTTER_FOUND - System has libplotter
#  PLOTTER_INCLUDE_DIRS - The libplotter include directories
#  PLOTTER_LIBRARIES - The libraries needed to use libplotter

find_path(PLOTTER_INCLUDE_DIR plotter.h
          HINTS /opt/local/include )

find_library(PLOTTER_LIBRARY NAMES plotter
             HINTS  /opt/local/lib )

set(PLOTTER_LIBRARIES ${PLOTTER_LIBRARY} )
set(PLOTTER_INCLUDE_DIRS ${PLOTTER_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set PLOTTER_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(plotter  DEFAULT_MSG
                                  PLOTTER_LIBRARY PLOTTER_INCLUDE_DIR)

mark_as_advanced(PLOTTER_INCLUDE_DIR PLOTTER_LIBRARY )