px4_add_library(VelCtrl
	VelCtrl.cpp
	VelCtrl.hpp
)
target_compile_options(VelCtrl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(VelCtrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(VelCtrl PRIVATE mathlib)
target_link_libraries(VelCtrl PUBLIC ladrc)
