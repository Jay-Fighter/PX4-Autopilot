############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
#build our submodule as its own library so that we can avoid changing it at all
set(VERTIQ_INC_DIR ${PX4_SOURCE_DIR}/src/drivers/actuators/vertiq_io/iq-module-communication-cpp/inc)
set(VERTIQ_SRC_DIR ${PX4_SOURCE_DIR}/src/drivers/actuators/vertiq_io/iq-module-communication-cpp/src)
# message(STATUS "VERTIQ_INC_DIR = ${VERTIQ_INC_DIR}")
include_directories(
	${VERTIQ_INC_DIR}
    )

#add all of our submodule sources
set(SOURCES
	${VERTIQ_SRC_DIR}/generic_interface.cpp
	${VERTIQ_INC_DIR}/client_communication.cpp
	${VERTIQ_SRC_DIR}/packet_finder.c
	${VERTIQ_SRC_DIR}/crc_helper.c
	${VERTIQ_SRC_DIR}/byte_queue.c
	)

add_library(iq_communication ${SOURCES})
add_dependencies(iq_communication prebuild_targets)
include_directories(${VERTIQ_INC_DIR})
px4_add_module(
	MODULE examples__vertiq_actuator_dev
	MAIN voltage_superposition_helper
	SRCS
		voltage_superposition_helper.cpp
		voltage_superposition_helper.hpp
	DEPENDS
		iq_communication
		px4_work_queue
	)

include_directories(/usr/include/serial)
target_link_libraries(
	examples__vertiq_actuator_dev
	PUBLIC
		serial
)
