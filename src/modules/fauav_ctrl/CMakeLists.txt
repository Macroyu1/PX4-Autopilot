add_subdirectory(FaCtrl)

px4_add_module(
  MODULE modules__fauav_ctrl
  MAIN fauav_ctrl
  COMPILE_FLAGS
    ${MAX_CUSTOM_OPT_LEVEL}
  SRCS
    fauav_ctrl.cpp
    fauav_ctrl.hpp
  DEPENDS
    px4_work_queue
    ladrc
    FaCtrl
  )
