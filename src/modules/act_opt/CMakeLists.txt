px4_add_module(
  MODULE modules__act_opt
  MAIN act_opt
  COMPILE_FLAGS
    #-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
    #-O0             # uncomment when debugging
  SRCS
    act_opt.cpp
  DEPENDS
    px4_work_queue
  )
