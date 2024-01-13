add_subdirectory(VelCtrl)

px4_add_module(
  MODULE modules__vel_ctrl
  MAIN vel_ctrl
  COMPILE_FLAGS
    ${MAX_CUSTOM_OPT_LEVEL}
  SRCS
    vel_ctrl.cpp
    vel_ctrl.hpp
  DEPENDS
    px4_work_queue
    ladrc
    VelCtrl
  )
