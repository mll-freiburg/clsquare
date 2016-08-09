CHECK_EXTERNAL(KernelBasedBatchRLController ANN)

if(NOT MISSING_DEPENDENCY)

  INCLUDE_DIRECTORIES(${CURRENT_LIST_DIR})

  LIST(APPEND controller_srcs 
	  ${CURRENT_LIST_DIR}/KernelBasedBatchRLController.cc
  )

  LIST(APPEND controller_headers
	  ${CURRENT_LIST_DIR}/KernelBasedBatchRLController.h
  )

  source_group (Headers FILES ${CURRENT_LIST_DIR}/KernelBasedBatchRLController.h )

endif()
