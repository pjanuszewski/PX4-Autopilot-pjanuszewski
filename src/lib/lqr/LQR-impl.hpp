/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM>
bool LQR<STATE_DIM, CONTROL_DIM>::compute(const state_matrix_t& Q,
    const control_matrix_t& R,
    const state_matrix_t& A,
    const control_gain_matrix_t& B,
    control_feedback_t& K,
    bool RisDiagonal,
    bool solveRiccatiIteratively)
{
    control_matrix_t R_inverse;
    state_matrix_t P;

    bool success = care_.solve(Q, R, A, B, P, RisDiagonal, R_inverse, solveRiccatiIteratively);

    K = (R_inverse * (B.transpose() * P));

    return success;
}

}  // namespace optcon
}  // namespace ct
