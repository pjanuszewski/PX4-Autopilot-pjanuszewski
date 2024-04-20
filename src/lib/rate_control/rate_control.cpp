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

#include "rate_control.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

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

bool RateControl::parseSDFInertiaAndMass(const std::string &filename, float &I_xx, float &I_yy, float &I_zz, float &mass)
{
	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
		std::cerr << "Loading SDF file failed." << std::endl;
		return false;
	}

	tinyxml2::XMLElement * modelElement = doc.FirstChildElement("sdf")->FirstChildElement("model");
	if (!modelElement) {
		std::cerr << "Model element not found in SDF." << std::endl;
		return false;
	}

	tinyxml2::XMLElement * linkElement = modelElement->FirstChildElement("link");
	while (linkElement) {
		if (std::string(linkElement->Attribute("name")) == "base_link") { // Assuming 'base_link' contains the inertia and mass
		tinyxml2::XMLElement * inertialElement = linkElement->FirstChildElement("inertial");
		if (inertialElement) {
			tinyxml2::XMLElement * massElement = inertialElement->FirstChildElement("mass");
			if (massElement) {
				massElement->QueryFloatText(&mass);
			}
			tinyxml2::XMLElement * inertiaElement = inertialElement->FirstChildElement("inertia");
			if (inertiaElement) {
				inertiaElement->QueryFloatAttribute("ixx", &I_xx);
				inertiaElement->QueryFloatAttribute("iyy", &I_yy);
				inertiaElement->QueryFloatAttribute("izz", &I_zz);
				return true; // Successfully found and extracted inertia and mass values
			}
		}
		}
		linkElement = linkElement->NextSiblingElement("link");
	}

	std::cerr << "Inertia or mass element not found in SDF." << std::endl;
	return false;
}

void RateControl::setLqrMatrices()
{
	std::string sdfPath = MODEL_SDF_PATH; // Use the macro directly
	float A_z = 0.1;
	float A_r = 0.2;
	float I_xx, I_yy, I_zz, mass;
	parseSDFInertiaAndMass(sdfPath, I_xx, I_yy, I_zz, mass);

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
}

Vector3f RateControl::lqrUpdate(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
				const float dt, const bool landed)
{
	Eigen::Matrix<double, stateDim, 1> rate_error_extended = Eigen::Matrix<double, stateDim, 1>::Zero();
	Vector3f rate_error = rate_sp - rate;

	for (size_t i = 0; i < 3; ++i) {
    		rate_error_extended(i) = rate_error(i); // Copy the first three elements
	}

	lqr.compute(Q, R, A, B, K);
	Eigen::Matrix<double, 3, 1> torque_temp = (-K * rate_error_extended).topRows<3>();
	Vector3f torque(torque_temp(0), torque_temp(1), torque_temp(2));
	return torque;
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
