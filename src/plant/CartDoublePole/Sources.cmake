INCLUDE_DIRECTORIES(${CURRENT_LIST_DIR})

LIST(APPEND plant_srcs 
	${CURRENT_LIST_DIR}/cartdoublepole.cc
	${CURRENT_LIST_DIR}/cartdoublepole_dynamics.cc
	${CURRENT_LIST_DIR}/cartdoublepole_graphic.cc
)
