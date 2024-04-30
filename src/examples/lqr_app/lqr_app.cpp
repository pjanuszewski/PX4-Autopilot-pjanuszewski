#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <control-toolbox/ct_optcon/include/ct/optcon/optcon.h>
//#include <control-toolbox/ct_core/include/ct/core/core.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>

	// static const size_t stateDim = 8;
    	// static const size_t controlDim = 4;

	// static constexpr float A_z = 0.1f;
	// static constexpr float A_r = 0.2f;
	// static constexpr float I_xx = 0.02f; // Inertia around the X-axis
	// static constexpr float I_yy = 0.02f; // Inertia around the Y-axis
	// static constexpr float I_zz = 0.04f; // Inertia around the Z-axis
	// static constexpr float mass = 1.5f;  // Mass of the vehicle

	// ct::optcon::LQR<stateDim, controlDim> lqr;


	//Eigen::Matrix<double, stateDim, stateDim> A;
	//ct::optcon::LQR<stateDim, controlDim>::state_matrix_t A;
	// Eigen::Matrix<double, stateDim, controlDim> B;
	// Eigen::Matrix<double, stateDim, stateDim> Q;
	// Eigen::Matrix<double, controlDim, controlDim> R;
	// Eigen::Matrix<double, controlDim, stateDim> K;
	// Eigen::Matrix<double, controlDim, stateDim> Kiterative;


extern "C" __EXPORT int lqr_app_main(int argc, char *argv[]);
int lqr_app_main(int argc, char *argv[]) {

	// const int stateDim = 8;
	// const int controlDim = 4;
	// ct::optcon::LQR<stateDim, controlDim>::state_matrix_t A;
	// ct::optcon::LQR<stateDim, controlDim>::control_gain_matrix_t B;
	// ct::optcon::LQR<stateDim, controlDim>::state_matrix_t Q;
    	// ct::optcon::LQR<stateDim, controlDim>::control_matrix_t R;
	// ct::core::FeedbackMatrix<stateDim, controlDim> K;
	//ct::optcon::LQR<stateDim, controlDim> lqr;

	// Q << 0.5, 0, 0, 0, 0, 0, 0, 0,
	// 0, 0.5, 0, 0, 0, 0, 0, 0,
	// 0, 0, 0.5, 0, 0, 0, 0, 0,
	// 0, 0, 0, 0.5, 0, 0, 0, 0,
	// 0, 0, 0, 0, 0.5, 0, 0, 0,
	// 0, 0, 0, 0, 0, 0.5, 0, 0,
	// 0, 0, 0, 0, 0, 0, 0.5, 0,
	// 0, 0, 0, 0, 0, 0, 0, 0.5;

	// R << 0.5, 0, 0, 0,
	// 0, 0.5, 0, 0,
	// 0, 0, 0.5, 0,
	// 0, 0, 0, 0.5;

	// A << 0, 0, 0, 0, 1, 0, 0, 0,
	// 0, 0, 0, 0, 0, 1, 0, 0,
	// 0, 0, 0, 0, 0, 0, 1, 0,
	// 0, 0, 0, 0, 0, 0, 0, 1,
	// 0, 0, 0, 0, -A_z / mass, 0, 0, 0,
	// 0, 0, 0, 0, 0, -A_r / I_xx, 0, 0,
	// 0, 0, 0, 0, 0, 0, -A_r / I_yy, 0,
	// 0, 0, 0, 0, 0, 0, 0, -A_r / I_zz;

	// B << 0, 0, 0, 0,
	// 0, 0, 0, 0,
	// 0, 0, 0, 0,
	// 1/mass, 0, 0, 0,
	// 0, 1/I_xx, 0, 0,
	// 0, 0, 1/I_yy, 0,
	// 0, 0, 0, 1/I_zz;

	// Q << 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2000, 0, 0, 0, 0, 0, 0, 0, 0,
        // 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0,
        // 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        // 0, 0.02;

    	// R << 100, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 100;


    	// A << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        // 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        // 0, 0, 9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0, -0, 0, 0, 0, 0, 0, 0, 0,
        // 0, 0, 0, 0, 0, -0, 0, 0, 0, 0, -0, -0, 0, 0, 0, 0, -0, -0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0, 0,
        // 0;


    	// B << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0, 0, 0, 0, 1.39665, 0, 0,
        // 0, 0, 142.857, -0, 0, 0, 0, 142.857, 0, 0, -0, 0, 83.3333;

	//K.setZero();
	//bool foundSolutionIterative = lqr.compute(Q, R, A, B, Kiterative, false, true);

	//lqr.compute(Q, R, A, B, K);
	//lqr.compute(Q, R, A, B, K, false);

	// for (size_t i = 0; i < stateDim; i++) {
    	// PX4_INFO("K : %f %f %f %f \r\n", K(i, 0), K(i, 1), K(i, 2), K(i, 3));
	// }

	// PX4_INFO("foundSolutionIterative : %d \r\n", foundSolutionIterative);
	// PX4_INFO("Kiterative : %f %f %f %f \r\n", Kiterative(0, 0), Kiterative(0, 1), Kiterative(0, 2), Kiterative(0, 3));

	return 0;

}

