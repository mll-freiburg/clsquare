INCLUDE_DIRECTORIES(${CURRENT_LIST_DIR})

LIST(APPEND plant_srcs 
	${CURRENT_LIST_DIR}/cartpole.cc
	${CURRENT_LIST_DIR}/cartpole_dynamics.cc
	${CURRENT_LIST_DIR}/cartpole_graphic.cc
)
