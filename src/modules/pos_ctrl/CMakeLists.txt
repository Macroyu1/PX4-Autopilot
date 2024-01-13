add_subdirectory(PosCtrl)
px4_add_module(
  MODULE modules__pos_ctrl
  MAIN pos_ctrl
  COMPILE_FLAGS
    ${MAX_CUSTOM_OPT_LEVEL}
  SRCS
    pos_ctrl.cpp
    pos_ctrl.hpp
  DEPENDS
    px4_work_queue
    ladrc
    PosCtrl
    Takeoff
  )
