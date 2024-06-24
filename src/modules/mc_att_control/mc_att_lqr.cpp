// #include "mc_att_lqr.hpp"
// #include <fstream> // Include for file operations

// template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
// int LQR<STATE_DIM, CONTROL_DIM, SCALAR>::lqrtest()
// {
//     int LQR<STATE_DIM, CONTROL_DIM, SCALAR>::N = 2 * STATE_DIM;
//     Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> A;
//     Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> B;
//     Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> Q;
//     Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> R;
//     Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> P;  // Define P


//     A << SCALAR(0), SCALAR(1),
//          SCALAR(0.01), SCALAR(0);
//     B << SCALAR(0),
//          SCALAR(1);
//     Q << SCALAR(1), SCALAR(0),
//          SCALAR(0), SCALAR(2);
//     R << SCALAR(2);


//     LQR<STATE_DIM, CONTROL_DIM, SCALAR> lqr;
//     Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> K;

//     bool success = lqr.compute(Q, R, A, B, P, K);  // Pass the iterations variable

//     if (success)
//     {
//         PX4_INFO("LQR computation successful.");

//         // Write K matrix to a CSV file
//         std::ofstream file("/home/pawelj/Git_repos/PX4-Autopilot/src/modules/mc_att_control/K_matrix.csv");
//         if (file.is_open())
//         {
//             for (int i = 0; i < K.rows(); ++i)
//             {
//                 for (int j = 0; j < K.cols(); ++j)
//                 {
//                     file << K(i, j);
//                     if (j != K.cols() - 1)
//                         file << ", "; // CSV column separator
//                 }
//                 file << "\n"; // Newline for next row
//             }
//             file.close();
//             PX4_INFO("K matrix written to K_matrix.csv");
//         }
//         else
//         {
//             PX4_INFO("Failed to open file for writing K matrix.");
//         }
//     }

//     return 0;
// }

// bool solveSchurDirect(const schur_matrix_t& M, state_matrix_t& P)
// {
// #ifdef CT_USE_LAPACK
//     const bool computeU = true;
//     schur_.compute(M, computeU);

//     if (schur_.info() != Eigen::Success)
//     {
//         throw std::runtime_error(
//             "LQR Schur computation failed. Most likely problem is set up wrongly or not solvable.");
//     }

//     schur_matrix_t U(schur_.matrixU());
//     schur_matrix_t T(schur_.matrixT());

//     int SELECT[2 * STATE_DIM] = {0};
//     double WR[2 * STATE_DIM] = {0};
//     double WI[2 * STATE_DIM] = {0};
//     int MS = 0;
//     double S = 0;
//     double SEP = 0;
//     int INFO = 0;
//     int N = 2 * STATE_DIM;

//     for (size_t i = 0; i < 2 * STATE_DIM; i++)
//     {
//         // check if last row or eigenvalue is complex (2x2 block)
//         if (i == (2 * STATE_DIM - 1) || std::abs(T(i + 1, i)) < 1e-12)
//         {
//             SELECT[i] = static_cast<int>(T(i, i) < 0);
//         }
//         else
//         {
//             // we have a complex block
//             SELECT[i] = static_cast<int>((T(i, i) + T(i + 1, i + 1)) / 2.0 < 0);
//             SELECT[i + 1] = SELECT[i];
//             i++;
//         }
//     }

//     dtrsen_("N", "V", &SELECT[0], &N, T.data(), &N, U.data(), &N, &WR[0], &WI[0], &MS, &S, &SEP, WORK_.data(), &LWORK_,
//         IWORK_.data(), &LIWORK_, &INFO);

//     const state_matrix_t& U11 = U.template block<STATE_DIM, STATE_DIM>(0, 0);
//     const state_matrix_t& U21 = U.template block<STATE_DIM, STATE_DIM>(STATE_DIM, 0);

//     // solve here for better numerical properties
//     P.noalias() = U21 * U11.inverse();

//     if (INFO != 0)
//     {
//         return false;
//     }

//     return true;
// #else
//     throw std::runtime_error(
//         "solveSchurDirect() in CARE can only be used if the lapack library is installed on your system.");
//     return false;
// #endif
// }

// bool compute(const state_matrix_t& Q,
//     const control_matrix_t& R,
//     const state_matrix_t& A,
//     const control_gain_matrix_t& B,
//     control_feedback_t& K,
//     bool RisDiagonal,
//     bool solveRiccatiIteratively)
// {
//     control_matrix_t R_inverse;
//     state_matrix_t P;

//     bool success = care_.solve(Q, R, A, B, P, RisDiagonal, R_inverse, solveRiccatiIteratively);

//     K = (R_inverse * (B.transpose() * P));

//     return success;
// }

// bool solve(const state_matrix_t& Q,
//     const control_matrix_t& R,
//     const state_matrix_t& A,
//     const control_gain_matrix_t& B,
//     state_matrix_t& P,
//     bool RisDiagonal,
//     control_matrix_t& R_inverse,
//     bool useIterativeSolver)
// {
//     if (RisDiagonal)
//     {
//         R_inverse.setZero();
//         R_inverse.diagonal().noalias() = R.diagonal().cwiseInverse();
//     }
//     else
//     {
//         R_inverse.noalias() = R.inverse();
//     }

//     schur_matrix_t M;
//     M << A, -B * R_inverse * B.transpose(), -Q, -A.transpose();

//     if (useIterativeSolver)
//         return solveSchurIterative(M, P);
//     else
//         return solveSchurDirect(M, P);
// }

// template class LQR<2, 1, double>;
