px4_add_library(PosCtrl
	ControlMath.cpp
	ControlMath.hpp
	PosCtrl.cpp
	PosCtrl.hpp
)
target_compile_options(PosCtrl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(PosCtrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(PosCtrl PRIVATE mathlib)
target_link_libraries(PosCtrl PUBLIC ladrc)
