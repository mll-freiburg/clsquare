INCLUDE_DIRECTORIES(${CURRENT_LIST_DIR})

LIST(APPEND plant_srcs 
	${CURRENT_LIST_DIR}/parallelpole.cpp
	${CURRENT_LIST_DIR}/parallelpole_dynamics.cpp
	${CURRENT_LIST_DIR}/parallelpole_graphic.cpp
	${CURRENT_LIST_DIR}/dynamics_original.cpp
	${CURRENT_LIST_DIR}/dynamics_gomez.cpp
)
