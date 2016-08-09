include_directories(${CLSQUARE_SOURCE_DIR}/utils)
include_directories(${CLSQUARE_SOURCE_DIR}/utils/BatchUtils)
LIST(APPEND utils_srcs
	${CLSQUARE_SOURCE_DIR}/utils/aset.cc
	${CLSQUARE_SOURCE_DIR}/utils/filter.cc
	${CLSQUARE_SOURCE_DIR}/utils/random.cc
	${CLSQUARE_SOURCE_DIR}/utils/sysmodel.cc
	${CLSQUARE_SOURCE_DIR}/utils/tcpsocket.cc
	${CLSQUARE_SOURCE_DIR}/utils/udpsocket.cc
	${CLSQUARE_SOURCE_DIR}/utils/BatchUtils/BatchData.cc
	${CLSQUARE_SOURCE_DIR}/utils/BatchUtils/AbstractBatchController.cc
	${CLSQUARE_SOURCE_DIR}/utils/BatchUtils/VectorArithmetic.cc
)
