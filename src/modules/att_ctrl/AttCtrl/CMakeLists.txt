px4_add_library(AttCtrl
	AttCtrl.cpp
	AttCtrl.hpp
)
target_compile_options(AttCtrl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(AttCtrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(AttCtrl PRIVATE mathlib)
target_link_libraries(AttCtrl PUBLIC ladrc)
