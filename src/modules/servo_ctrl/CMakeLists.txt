px4_add_module(
  MODULE modules__servo_ctrl
  MAIN servo_ctrl
  COMPILE_FLAGS
    #-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
    #-O0             # uncomment when debugging
  SRCS
    servo_ctrl.cpp
    servo_ctrl.hpp
    uart.h
  DEPENDS
    px4_work_queue
  )
