px4_add_module(
  MODULE modules__fault
  MAIN fault
  COMPILE_FLAGS
    ${MAX_CUSTOM_OPT_LEVEL}
  SRCS
    fault.cpp
    fault.hpp
  DEPENDS
    px4_work_queue
  )
