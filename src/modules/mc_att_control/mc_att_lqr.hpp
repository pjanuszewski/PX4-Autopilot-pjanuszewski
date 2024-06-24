// #pragma once

// #include <Eigen/Dense>
// #include <iostream>
// #include <stdexcept>
// #include <px4_log.h>
// #include <fstream>
// #include <iomanip>

// #ifdef CT_USE_LAPACK
// extern "C" void dtrsen_(const char* JOB,
//     const char* COMPQ,
//     const int* SELECT,
//     const int* N,
//     const double* T,
//     const int* LDT,
//     const double* Q,
//     const int* LDQ,
//     double* WR,
//     double* WI,
//     int* M,
//     double* S,
//     double* SEP,
//     double* WORK,
//     const int* LWORK,
//     int* IWORK,
//     const int* LIWORK,
//     int* INFO);
// #endif

// template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
// class LQR
// {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     typedef Eigen::Matrix<double, 2 * STATE_DIM, 2 * STATE_DIM> schur_matrix_t;
//     typedef Eigen::Matrix<double, 2 * STATE_DIM, STATE_DIM> factor_matrix_t;

//     // Variable declarations
//     schur_matrix_t T = schur_matrix_t::Zero();
//     schur_matrix_t U = schur_matrix_t::Zero();

//     int SELECT[2 * STATE_DIM] = {0};
//     static int N;
//     double WR[T.ColsAtCompileTime] = {0};
//     double WI[T.ColsAtCompileTime] = {0};
//     int MS = 0;
//     double S = 0.0;
//     double SEP = 0.0;
//     double WORKDUMMY[1] = {0};
//     int LWORK = -1;
//     int IWORKQUERY[1] = {0};
//     int LIWORK = -1;
//     int INFO = 0;
//     int TCols = T.ColsAtCompileTime;

//     typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
//     typedef Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
//     typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
//     typedef Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> control_gain_matrix_t;
//     typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_feedback_t;

//     typedef Eigen::Matrix<double, 2 * STATE_DIM, 2 * STATE_DIM> schur_matrix_t;
//     typedef Eigen::Matrix<double, 2 * STATE_DIM, STATE_DIM> factor_matrix_t;

//     LQR() = default;>

//     int lqrtest();


//     bool solveSchurDirect(const schur_matrix_t& M, state_matrix_t& P);

//     bool compute(const state_matrix_t& Q,
//         const control_matrix_t& R,
//         const state_matrix_t& A,
//         const control_gain_matrix_t& B,
//         control_feedback_t& K,
//         bool RisDiagonal,
//         bool solveRiccatiIteratively);

//     bool solve(const state_matrix_t& Q,
//         const control_matrix_t& R,
//         const state_matrix_t& A,
//         const control_gain_matrix_t& B,
//         state_matrix_t& P,
//         bool RisDiagonal,
//         control_matrix_t& R_inverse,
//         bool useIterativeSolver);

// private:
//     int LWORK_;
//     int LIWORK_;
//     Eigen::RealSchur<schur_matrix_t> schur_;
//     Eigen::VectorXd WORK_;
//     Eigen::VectorXi IWORK_;

// }
