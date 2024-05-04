# px4_add_module(
# 	MODULE examples__lqr_app_task
# 	MAIN lqr_app_task

# 	COMPILE_FLAGS
# 	-DTRACE_BUILD

# 	SRCS
# 		lqr_app_task.cpp
# 		lqr_app_task.hpp
# 	DEPENDS
# 		mathlib
# 		ct_optcon
# 	)

# find_package(Eigen3 REQUIRED)
# list(APPEND ct_core_target_include_dirs ${EIGEN3_INCLUDE_DIR})
# target_include_directories(examples__lqr_app_task PUBLIC
# 	${CMAKE_CURRENT_SOURCE_DIR}
# 	${CONTROL_TOOLBOX_PATH}/ct_core/include
# 	${CONTROL_TOOLBOX_PATH}/ct_optcon/include
# )
# find_package(OpenMP)
# if(OpenMP_CXX_FOUND)
#     target_link_libraries(examples__lqr_app_task PUBLIC OpenMP::OpenMP_CXX)
# endif()
