/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * @file rate_control.hpp
 *
 * PID 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <tinyxml2.h>
//#include <control-toolbox/ct_optcon/include/ct/optcon/optcon.h>
#include <matrix/matrix/math.hpp>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <drivers/drv_hrt.h>

#include <mathlib/mathlib.h>
#include <uORB/topics/rate_ctrl_status.h>

class RateControl
{
public:

	RateControl() = default;
	~RateControl() = default;


	// void setLqrMatrices();
	// bool computeLqr();

	matrix::Vector3f lqrUpdate(const matrix::Vector3f &position, const matrix::Vector3f &position_sp,
				    const matrix::Vector3f &velocity, const matrix::Vector3f &velocity_sp,
				    const matrix::Vector3f &euler, const matrix::Vector3f &angles_sp,
				    const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				    const matrix::Vector3f &angular_accel, const hrt_abstime &current_time);
	void setLqrGains(const std::string &mode);
	void printWorkingDirectory();

	static const size_t stateDim = 12;
    	static const size_t controlDim = 4;
	std::vector<matrix::Matrix<float, controlDim, stateDim>> readMatricesFromFile(const std::string &filename);

	// Add this in rate_control.hpp or at the top of rate_control.cpp

	/**
	 * Set the rate control PID gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setPidGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the mximum absolute value of the integrator for all axes
	 * @param integrator_limit limit value for all axes x, y, z
	 */
	void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };

	/**
	 * Set direct rate to torque feed forward gain
	 * @see _gain_ff
	 * @param FF 3D vector of feed forward gains for body x,y,z axis
	 */
	void setFeedForwardGain(const matrix::Vector3f &FF) { _gain_ff = FF; };

	/**
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	void setSaturationStatus(const matrix::Vector3<bool> &saturation_positive,
				 const matrix::Vector3<bool> &saturation_negative);

	/**
	 * Set individual saturation flags
	 * @param axis 0 roll, 1 pitch, 2 yaw
	 * @param is_saturated value to update the flag with
	 */
	void setPositiveSaturationFlag(size_t axis, bool is_saturated);
	void setNegativeSaturationFlag(size_t axis, bool is_saturated);

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const float dt, const bool landed);

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	void resetIntegral() { _rate_int.zero(); }

	/**
	 * Set the integral term to 0 for specific axes
	 * @param  axis roll 0 / pitch 1 / yaw 2
	 * @see _rate_int
	 */
	void resetIntegral(size_t axis)
	{
		if (axis < 3) {
			_rate_int(axis) = 0.f;
		}
	}

	static constexpr float A_z = 0.1f;
	static constexpr float A_r = 0.2f;
	static constexpr float I_xx = 0.02f; // Inertia around the X-axis
	static constexpr float I_yy = 0.02f; // Inertia around the Y-axis
	static constexpr float I_zz = 0.04f; // Inertia around the Z-axisa
	static constexpr float mass = 1.5f;  // Mass of the vehicle

	// ct::optcon::LQR<stateDim, controlDim> lqr;

	// Eigen::Matrix<double, stateDim, stateDim> A;
	// Eigen::Matrix<double, stateDim, controlDim> B;
	// Eigen::Matrix<double, stateDim, stateDim> Q;
	// Eigen::Matrix<double, controlDim, controlDim> R;
	// Eigen::Matrix<double, controlDim, stateDim> K;
	matrix::Vector3f K_lqr;
	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status);
	matrix::Matrix<float, RateControl::controlDim, RateControl::stateDim> readMatrixFromFile(const std::string &filename);

	void updateKMatrix(const hrt_abstime &current_time_us);

	enum class LqrMode {
	Hover,
	Trajectory,
	Invalid
	};

	void updateMissionStartTime() {
           mission_start_time = hrt_absolute_time();
	}

	static void mavlink_quaternion_to_dcm(const float quaternion[4], float dcm[3][3]);
	static void mavlink_dcm_to_euler(const float dcm[3][3], float* roll, float* pitch, float* yaw);
	static void mavlink_quaternion_to_euler(const float quaternion[4], float* roll, float* pitch, float* yaw);

private:
	void updateIntegral(matrix::Vector3f &rate_error, const float dt);

	// Gains
	matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	matrix::Vector3f _gain_i; ///< rate control integral gain
	matrix::Vector3f _gain_d; ///< rate control derivative gain
	matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters

	// States
	matrix::Vector3f _rate_int; ///< integral term of the rate controller

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;

	std::string K_value_file_path = "/home/pawelj/Git_repos/PX4-Autopilot/src/lib/rate_control/K_value.csv";
	std::string K_matrices_file_path = "/home/pawelj/Git_repos/PX4-Autopilot/src/lib/rate_control/K_matrices.csv";

	std::vector<matrix::Matrix<float, controlDim, stateDim>> K_matrices;
	hrt_abstime mission_start_time;

	LqrMode current_mode;

	//LQR variables

	matrix::Matrix<float, controlDim, stateDim> K_matrix;
	//ct::optcon::LQR<stateDim, controlDim>::control_feedback_t K_iterative;
	matrix::Vector3f K = matrix::Vector3f(0.83923, 0.83923, 0.858301);
	matrix::Vector3f K_hover = matrix::Vector3f(0.9726599304, 0.9726599304, 0.9904326023);
	matrix::Vector3f K_trajectory = matrix::Vector3f(0.9726599304, 0.9726599304, 0.9904326023);
};
