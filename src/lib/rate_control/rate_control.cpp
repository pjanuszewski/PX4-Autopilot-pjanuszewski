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
 * @file RateControl.cpp
 */

#include <px4_platform_common/log.h>
#include <iostream>
#include "rate_control.hpp"
#include <px4_platform_common/defines.h>
#include <matrix/matrix/Matrix.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <sstream>

using namespace matrix;

// RateControl::RateControl()
// {
// 	setLqrMatrices();
// }

void RateControl::setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setSaturationStatus(const Vector3<bool> &saturation_positive,
				      const Vector3<bool> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void RateControl::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_positive(axis) = is_saturated;
	}
}

void RateControl::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_negative(axis) = is_saturated;
	}
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}

// Vector3f RateControl::lqrUpdate(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp, const hrt_abstime &current_time) {
//     // if (current_mode == LqrMode::Trajectory) {vehicle_torque_setpoint
//     //     updateKMatrix(current_time);
//     // }
//     Vector3f rate_error = rate_sp - rate;
//     const Vector3f torque = -K.emult(rate_error);
//     return torque;
// }

Vector3f RateControl::lqrUpdate(const Vector3f &position, const Vector3f &position_sp,
                                const Vector3f &velocity, const Vector3f &velocity_sp,
                                const Vector3f &euler, const Vector3f &angles_sp,
                                const Vector3f &rate, const Vector3f &rate_sp,
                                const Vector3f &angular_accel, const hrt_abstime &current_time) {

    // Calculate state error
    Vector3f position_error = position_sp - position;
    Vector3f velocity_error = velocity_sp - velocity;
    Vector3f attitude_error = angles_sp - euler; // Simplified, consider proper angular error calculation
    Vector3f rate_error = rate_sp - rate;

    // Combine errors into a state error vector (simplified example)
    matrix::Vector<float, 12> state_error;
    K_matrix.setZero();
    state_error.setZero();
    state_error(0) = position_error(0);
    state_error(1) = position_error(1);
    state_error(2) = position_error(2);
    state_error(3) = velocity_error(0);
    state_error(4) = velocity_error(1);
    state_error(5) = velocity_error(2);
    state_error(6) = attitude_error(0);
    state_error(7) = attitude_error(1);
    state_error(8) = attitude_error(2);
    state_error(9) = rate_error(0);
    state_error(10) = rate_error(1);
    state_error(11) = rate_error(2);

    Vector4f result = K_matrix * state_error;  // Assuming K is defined such that this multiplication is valid
    Vector3f torque(result(1), result(2), result(3));  // Extracting the first three elements

    return torque;
}

void RateControl::mavlink_quaternion_to_dcm(const float quaternion[4], float dcm[3][3])
{
    double a = (double)quaternion[0];
    double b = (double)quaternion[1];
    double c = (double)quaternion[2];
    double d = (double)quaternion[3];
    double aSq = a * a;
    double bSq = b * b;
    double cSq = c * c;
    double dSq = d * d;
    dcm[0][0] = aSq + bSq - cSq - dSq;
    dcm[0][1] = 2 * (b * c - a * d);
    dcm[0][2] = 2 * (a * c + b * d);
    dcm[1][0] = 2 * (b * c + a * d);
    dcm[1][1] = aSq - bSq + cSq - dSq;
    dcm[1][2] = 2 * (c * d - a * b);
    dcm[2][0] = 2 * (b * d - a * c);
    dcm[2][1] = 2 * (a * b + c * d);
    dcm[2][2] = aSq - bSq - cSq + dSq;
}

void RateControl::mavlink_dcm_to_euler(const float dcm[3][3], float* roll, float* pitch, float* yaw)
{
    float phi, theta, psi;
    theta = asinf(-dcm[2][0]);

    if (fabsf(theta - (float)M_PI_2) < 1.0e-3f) {
        phi = 0.0f;
        psi = (atan2f(dcm[1][2] - dcm[0][1],
                dcm[0][2] + dcm[1][1]) + phi);

    } else if (fabsf(theta + (float)M_PI_2) < 1.0e-3f) {
        phi = 0.0f;
        psi = atan2f(dcm[1][2] - dcm[0][1],
                  dcm[0][2] + dcm[1][1] - phi);

    } else {
        phi = atan2f(dcm[2][1], dcm[2][2]);
        psi = atan2f(dcm[1][0], dcm[0][0]);
    }

    *roll = phi;
    *pitch = theta;
    *yaw = psi;
}


/**
 * Converts a quaternion to euler angles
 *
 * @param quaternion a [w, x, y, z] ordered quaternion (null-rotation being 1 0 0 0)
 * @param roll the roll angle in radians
 * @param pitch the pitch angle in radians
 * @param yaw the yaw angle in radians
 */

void RateControl::mavlink_quaternion_to_euler(const float quaternion[4], float* roll, float* pitch, float* yaw)
{
    float dcm[3][3];
    mavlink_quaternion_to_dcm(quaternion, dcm);
    mavlink_dcm_to_euler((const float(*)[3])dcm, roll, pitch, yaw);
}

// void RateControl::setLqrMatrices()
// {
// 	A << 0, 0, 0, 1, 0, 0,
// 	0, 0, 0, 0, 1, 0,
// 	0, 0, 0, 0, 0, 1,
// 	0, 0, 0, -A_r / I_xx, 0, 0,
// 	0, 0, 0, 0, -A_r / I_yy, 0,
// 	0, 0, 0, 0, 0, -A_r / I_zz;

// 	B << 0, 0, 0, 0,
// 	0, 0, 0, 0,
// 	0, 0, 0, 0,
// 	0, 1/I_xx, 0, 0,
// 	0, 0, 1/I_yy, 0,
// 	0, 0, 0, 1/I_zz;

// 	Q << 0.5, 0, 0, 0, 0, 0,
// 	0, 0.5, 0, 0, 0, 0,
// 	0, 0, 0.5, 0, 0, 0,
// 	0, 0, 0, 0.5, 0, 0,
// 	0, 0, 0, 0, 0.5, 0,
// 	0, 0, 0, 0, 0, 0.5;

// 	R << 0.5, 0, 0, 0,
// 	0, 0.5, 0, 0,
// 	0, 0, 0.5, 0,
// 	0, 0, 0, 0.5;

// 	K.setZero();
// 	bool f = lqr.compute(Q, R, A, B, K, false, true);
// 	if (!f) {
// 	PX4_WARN("LQR computation failed");
// 	}
// 	PX4_INFO("K Matrix: \n");
// 	for (int i = 0; i < K.rows(); ++i) {
// 		for (int j = 0; j < K.cols(); ++j) {
// 			PX4_INFO("%.2f ", K(i, j)); // Adjust format specifier as needed
// 		}
// 	PX4_INFO("\n");
// 	}
//    	K_lqr = {static_cast<float>(K(3,1)), static_cast<float>(K(4,2)), static_cast<float>(K(5,3))};
// }

void printWorkingDir() {
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != nullptr) {
        PX4_INFO("Current working dir: %s", cwd);
    } else {
        perror("getcwd() error");
    }
}

std::vector<matrix::Matrix<float, RateControl::controlDim, RateControl::stateDim>> RateControl::readMatricesFromFile(const std::string &filename)
{
    printWorkingDir();
    std::ifstream file(filename);
    if (!file.is_open()) {
        // Use errno and strerror to get a string explaining why the file could not be opened
	std::string error_message = "Unable to open file: " + filename;
	error_message += "\nError: " + std::string(std::strerror(errno));

        // Optionally, include additional details such as the current user or file permissions
        error_message += "\nCheck if the file exists, the path is correct, and the permissions are adequate.";

        throw std::runtime_error(error_message);
    }

    std::vector<double> matrixEntries;
    std::string line;

    // Read each line from the file
    while (getline(file, line)) {
        std::istringstream iss(line);
        double num;
        while (iss >> num) {
            matrixEntries.push_back(num);
            // Skip the tab or comma
            iss.ignore(1);
        }
    }

    // Check if the number of entries matches the expected size of multiple matrices
    size_t expectedSize = controlDim * stateDim;
    if (matrixEntries.size() % expectedSize != 0) {
        throw std::runtime_error("Matrix data in file does not match expected size of multiple matrices");
    }

    size_t numMatrices = matrixEntries.size() / expectedSize;
    std::vector<matrix::Matrix<float, controlDim, stateDim>> matrices(numMatrices);

    for (size_t k = 0; k < numMatrices; ++k) {
        matrix::Matrix<float, controlDim, stateDim> matrix;
        for (size_t i = 0; i < controlDim; ++i) {
            for (size_t j = 0; j < stateDim; ++j) {
                matrix(i, j) = static_cast<float>(matrixEntries[k * expectedSize + i * stateDim + j]);
            }
        }
        matrices[k] = matrix;
    }

    return matrices;
}

matrix::Matrix<float, RateControl::controlDim, RateControl::stateDim> RateControl::readMatrixFromFile(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::string error_message = "Unable to open file: " + filename;
        error_message += "\nError: " + std::string(std::strerror(errno));
        throw std::runtime_error(error_message);
    }

    std::vector<float> matrixEntries;
    std::string line;

    // Read each line from the file
    while (getline(file, line)) {
        std::istringstream iss(line);
        float num;
        while (iss >> num) {
            matrixEntries.push_back(num);
            // Skip the tab or comma
            iss.ignore(1);
        }
    }

    // Check if the number of entries matches the expected size of the matrix
    size_t expectedSize = RateControl::controlDim * RateControl::stateDim;
    if (matrixEntries.size() != expectedSize) {
        throw std::runtime_error("Matrix data in file does not match expected size");
    }

    matrix::Matrix<float, RateControl::controlDim, RateControl::stateDim> matrix;
    for (size_t i = 0; i < RateControl::controlDim; ++i) {
        for (size_t j = 0; j < RateControl::stateDim; ++j) {
            matrix(i, j) = matrixEntries[i * RateControl::stateDim + j];
        }
    }

    return matrix;
}

void RateControl::setLqrGains(const std::string &lqr_mode)
{
    if (lqr_mode == "about_hover") {
        current_mode = LqrMode::Hover;
        K_matrix = this->readMatrixFromFile("/home/pawelj/Git_repos/PX4-Autopilot/src/lib/rate_control/K_value.csv");
    }
    else if (lqr_mode == "about_trajectory") {
        current_mode = LqrMode::Trajectory;
        K_matrices = this->readMatricesFromFile("/home/pawelj/Git_repos/PX4-Autopilot/src/lib/rate_control/K_matrices.csv");
        K_lqr = {K_matrices[0](1, 9), K_matrices[0](2, 10), K_matrices[0](3, 11)};
    }
    else {
        throw std::runtime_error("Invalid LQR mode");
    }
}

void RateControl::updateKMatrix(const hrt_abstime &current_time) {
    const hrt_abstime elapsed_time = current_time - mission_start_time;
    uint16_t _current_K_index = 0;
    size_t new_index = elapsed_time / 8000;
    if (new_index >= K_matrices.size()) {
        new_index = K_matrices.size() - 1;
    }
    if (new_index != _current_K_index) {
        _current_K_index = new_index;
        K_lqr = {K_matrices[_current_K_index](1, 9), K_matrices[_current_K_index](2, 10), K_matrices[_current_K_index](3, 11)};
    }
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
