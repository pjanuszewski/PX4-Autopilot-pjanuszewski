/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <control-toolbox/ct_optcon/include/ct/optcon/optcon.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

	static const size_t stateDim = 8;
    	static const size_t controlDim = 4;

	static constexpr float A_z = 0.1f;
	static constexpr float A_r = 0.2f;
	static constexpr float I_xx = 0.02f; // Inertia around the X-axis
	static constexpr float I_yy = 0.02f; // Inertia around the Y-axis
	static constexpr float I_zz = 0.04f; // Inertia around the Z-axis
	static constexpr float mass = 1.5f;  // Mass of the vehicle

	ct::optcon::LQR<stateDim, controlDim> lqr;

	ct::optcon::LQR<stateDim, controlDim>::state_matrix_t A;
	ct::optcon::LQR<stateDim, controlDim>::control_gain_matrix_t B;
	ct::optcon::LQR<stateDim, controlDim>::state_matrix_t Q;
    	ct::optcon::LQR<stateDim, controlDim>::control_matrix_t R;
	ct::core::FeedbackMatrix<stateDim, controlDim> K;
	ct::optcon::LQR<stateDim, controlDim>::control_feedback_t K_iterative;

void setLqrMatrices()
{

	Q << 0.5, 0, 0, 0, 0, 0, 0, 0,
	0, 0.5, 0, 0, 0, 0, 0, 0,
	0, 0, 0.5, 0, 0, 0, 0, 0,
	0, 0, 0, 0.5, 0, 0, 0, 0,
	0, 0, 0, 0, 0.5, 0, 0, 0,
	0, 0, 0, 0, 0, 0.5, 0, 0,
	0, 0, 0, 0, 0, 0, 0.5, 0,
	0, 0, 0, 0, 0, 0, 0, 0.5;

	R << 0.5, 0, 0, 0,
	0, 0.5, 0, 0,
	0, 0, 0.5, 0,
	0, 0, 0, 0.5;

	A << 0, 0, 0, 0, 1, 0, 0, 0,
	0, 0, 0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 1,
	0, 0, 0, 0, -A_z / mass, 0, 0, 0,
	0, 0, 0, 0, 0, -A_r / I_xx, 0, 0,
	0, 0, 0, 0, 0, 0, -A_r / I_yy, 0,
	0, 0, 0, 0, 0, 0, 0, -A_r / I_zz;

	B << 0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	1/mass, 0, 0, 0,
	0, 1/I_xx, 0, 0,
	0, 0, 1/I_yy, 0,
	0, 0, 0, 1/I_zz;

	K.setZero();
	K_iterative.setZero();
	//lqr.compute(Q, R, A, B, K);
}

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* subscribe to vehicle_acceleration topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_acceleration));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/* advertise attitude topic */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct vehicle_acceleration_s accel;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_acceleration), sensor_sub_fd, &accel);
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)accel.xyz[0],
					 (double)accel.xyz[1],
					 (double)accel.xyz[2]);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
				att.q[0] = accel.xyz[0];
				att.q[1] = accel.xyz[1];
				att.q[2] = accel.xyz[2];

				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
