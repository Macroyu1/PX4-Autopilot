px4_add_library(FaCtrl
	FaCtrl.cpp
	FaCtrl.hpp
)
target_compile_options(FaCtrl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(FaCtrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(FaCtrl PRIVATE mathlib)
target_link_libraries(FaCtrl PUBLIC ladrc)
