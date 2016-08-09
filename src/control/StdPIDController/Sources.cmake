INCLUDE_DIRECTORIES(${CURRENT_LIST_DIR})

LIST(APPEND controller_srcs 
	${CURRENT_LIST_DIR}/pidcontroller.cc
#	${CURRENT_LIST_DIR}/auxactioncontroller.cc
)
