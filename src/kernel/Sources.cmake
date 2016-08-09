include_directories(${CLSQUARE_SOURCE_DIR}/kernel)
LIST(APPEND kernel_srcs 
	${CLSQUARE_SOURCE_DIR}/kernel/Socket.cc
	${CLSQUARE_SOURCE_DIR}/kernel/mainloop.cc
	${CLSQUARE_SOURCE_DIR}/kernel/setdef.cc
	${CLSQUARE_SOURCE_DIR}/kernel/statistics.cc
	${CLSQUARE_SOURCE_DIR}/kernel/funcgen1d.cc
	${CLSQUARE_SOURCE_DIR}/kernel/plant.cc
	${CLSQUARE_SOURCE_DIR}/kernel/str2val.cc
	${CLSQUARE_SOURCE_DIR}/kernel/registry.cc
	${CLSQUARE_SOURCE_DIR}/kernel/valueparser.cc
	${CLSQUARE_SOURCE_DIR}/kernel/reward.cc
	${CLSQUARE_SOURCE_DIR}/kernel/observer.cc
  ${CLSQUARE_SOURCE_DIR}/kernel/kerneldatatypes.cc
) 

LIST(APPEND kernel_headers
	${CLSQUARE_SOURCE_DIR}/kernel/setdef.h
	${CLSQUARE_SOURCE_DIR}/kernel/output.h
	${CLSQUARE_SOURCE_DIR}/kernel/statistics.h
	${CLSQUARE_SOURCE_DIR}/kernel/plant.h
	${CLSQUARE_SOURCE_DIR}/kernel/str2val.h
	${CLSQUARE_SOURCE_DIR}/kernel/input.h
	${CLSQUARE_SOURCE_DIR}/kernel/registry.h
	${CLSQUARE_SOURCE_DIR}/kernel/valueparser.h
	${CLSQUARE_SOURCE_DIR}/kernel/reward.h
	${CLSQUARE_SOURCE_DIR}/kernel/observer.h
	${CLSQUARE_SOURCE_DIR}/kernel/controller.h
	${CLSQUARE_SOURCE_DIR}/kernel/mainloop.h
)
