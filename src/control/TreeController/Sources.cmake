IF(NOT MISSING_DEPENDENCY)

  INCLUDE_DIRECTORIES(${CURRENT_LIST_DIR})

  # ADD_EXECUTABLE(extratree_test 
  #     ${CURRENT_LIST_DIR}/extratree_test.cc 
  #     ${CURRENT_LIST_DIR}/extratree.cc
  # )

  LIST(APPEND controller_srcs
    ${CURRENT_LIST_DIR}/extratree.cc
    ${CURRENT_LIST_DIR}/treecontroller.cc
  )

ENDIF()
