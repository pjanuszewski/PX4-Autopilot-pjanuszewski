############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Find the Eigen3 package
find_package(Eigen3 REQUIRED)
find_package(OpenMP)


px4_add_library(RateControl
	rate_control.cpp
	rate_control.hpp
)
target_compile_options(RateControl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
# Assuming ${CONTROL_TOOLBOX_PATH} is set to the parent directory of ct_core and ct_optcon
set(CONTROL_TOOLBOX_PATH "/home/pawelj/Git_repos/PX4-Autopilot/src/lib/control-toolbox")

# Apply -Wno-float-equal only if using GNU C++ or Clang compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(RateControl PRIVATE -Wno-float-equal)
endif()

target_include_directories(RateControl PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${CONTROL_TOOLBOX_PATH}/ct_core/include
	${CONTROL_TOOLBOX_PATH}/ct_optcon/include
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(RateControl PUBLIC OpenMP::OpenMP_CXX)
else()
    message(STATUS "OpenMP not found")
endif()

include_directories(${EIGEN3_INCLUDE_DIR})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(RateControl PRIVATE -Wno-float-equal)
endif()

target_link_libraries(RateControl PRIVATE mathlib)
target_link_libraries(RateControl PRIVATE ct_optcon)

px4_add_unit_gtest(SRC rate_control_test.cpp LINKLIBS RateControl)
