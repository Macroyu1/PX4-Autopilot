add_subdirectory(AttCtrl)

px4_add_module(
  MODULE modules__att_ctrl
  MAIN att_ctrl
  COMPILE_FLAGS
    ${MAX_CUSTOM_OPT_LEVEL}
  SRCS
    att_ctrl.cpp
    att_ctrl.hpp
  DEPENDS
    px4_work_queue
    ladrc
    AttCtrl
  )
