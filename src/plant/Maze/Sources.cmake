INCLUDE_DIRECTORIES(${CURRENT_LIST_DIR})

#Attention: this plant is not transferred yet to CLS2 4.0

LIST(APPEND plant_srcs 
	${CURRENT_LIST_DIR}/cell.cc
	${CURRENT_LIST_DIR}/maze.cc
	${CURRENT_LIST_DIR}/mazedata.cc
	${CURRENT_LIST_DIR}/mazegraphic.cc
)
