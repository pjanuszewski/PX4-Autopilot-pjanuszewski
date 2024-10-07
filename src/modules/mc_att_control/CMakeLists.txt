############################################################################
#
#   Copyright (c) 2015-2019 PX4 Development Team. All rights reserved.
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

add_subdirectory(AttitudeControl)
find_package(LAPACK REQUIRED)
find_package(ct_optcon REQUIRED)  # Find the ct_optcon package

px4_add_module(
	MODULE modules__mc_att_control
	MAIN mc_att_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		mc_att_control_main.cpp
		mc_att_control.hpp
		LQR.hpp
		CARE.hpp
		LQR-impl.hpp
		CARE-impl.hpp
	DEPENDS
		AttitudeControl
		mathlib
		px4_work_queue
	)

target_compile_definitions(modules__mc_att_control PRIVATE CT_USE_LAPACK)

if (LAPACK_FOUND)
	include_directories(${LAPACK_INCLUDE_DIRS})
	target_link_libraries(modules__mc_att_control PUBLIC ${LAPACK_LIBRARIES})
else()
	message(FATAL_ERROR "LAPACK library not found.")
endif()

find_package(ct_optcon REQUIRED)  # Find the ct_optcon package
if(ct_optcon_FOUND)
	include_directories(${ct_optcon_INCLUDE_DIRS})
	target_link_libraries(modules__mc_att_control PUBLIC ${ct_optcon_LIBRARIES})
endif()
