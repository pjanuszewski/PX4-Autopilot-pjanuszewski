############################################################################
#
#   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

find_package(LAPACK REQUIRED)

# Manually specify the LAPACK library and include directories
set(LAPACK_LIBRARIES /usr/local/lib/liblapack.a)
set(LAPACK_INCLUDE_DIRS /usr/local/include)

# Specify the BLAS library
set(BLAS_LIBRARIES /usr/local/lib/libblas.a)

# Specify the Fortran runtime library
set(FORTRAN_LIBRARIES gfortran)

px4_add_module(
    MODULE modules__lqr_module
    MAIN lqr_module
    COMPILE_FLAGS
    -DDEBUG_BUILD
    ${MAX_CUSTOM_OPT_LEVEL}
    SRCS
        lqr_module_main.cpp
        lqr_module.hpp
        LQR.hpp
		CARE.hpp
		LQR-impl.hpp
		CARE-impl.hpp
        lqr_quaternion.cpp
        lqr_quaternion.hpp
        declarations_quaternion.hpp
    DEPENDS
        mathlib
        px4_work_queue
        # Add dependencies here if needed
)

target_compile_definitions(modules__lqr_module PRIVATE CT_USE_LAPACK)

if (LAPACK_FOUND)
    include_directories(${LAPACK_INCLUDE_DIRS})
    target_link_libraries(modules__lqr_module PUBLIC ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${FORTRAN_LIBRARIES})
else()
    message(FATAL_ERROR "LAPACK library not found.")
endif()
