#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <control-toolbox/ct_optcon/include/ct/optcon/optcon.h>
#include <control-toolbox/ct_core/include/ct/core/core.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>

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

extern "C" __EXPORT int lqr_app_main(int argc, char *argv[]);
int lqr_app_main(int argc, char *argv[]) {

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
	lqr.compute(Q, R, A, B, K);

	for (size_t i = 0; i < stateDim; i++) {
    	PX4_INFO("K : %f %f %f %f \r\n", K(i, 0), K(i, 1), K(i, 2), K(i, 3));
	}

	return 0;

}

