CHECK_INTERNAL ( NeuroPredictor n++ )

IF ( NOT MISSING_DEPENDENCY )
  INCLUDE_DIRECTORIES ( ${CURRENT_LIST_DIR} )
  LIST ( APPEND observer_srcs ${CURRENT_LIST_DIR}/neuroobserver.cpp )
  if ( EXISTS "${CURRENT_LIST_DIR}/neuropredictor.cpp" )
    LIST ( APPEND observer_srcs ${CURRENT_LIST_DIR}/neuropredictor.cpp )
  ENDIF ()
ENDIF ()
