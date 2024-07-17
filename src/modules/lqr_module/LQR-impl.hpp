/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once
#include <px4_log.h>
#include <fstream>
#include <iomanip>

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

    if (success)
    {
        //PX4_INFO("LQR computation successful.");
        // Write K matrix to a CSV file
        std::ofstream file("/home/pawelj/Git_repos/PX4-Autopilot/src/modules/lqr_module/K_matrix.csv");
        if (file.is_open())
        {
            file << std::scientific << std::setprecision(6); // Set scientific notation and precision for all values
            for (int i = 0; i < K.rows(); ++i)
            {
                for (int j = 0; j < K.cols(); ++j)
                {
                    file << std::setw(15) << K(i, j); // Set width for each column
                    if (j != K.cols() - 1)
                        file << ", "; // CSV column separator
                }
                file << "\n"; // Newline for next row
            }
            file.close();
        }
        else
        {
            PX4_INFO("Failed to open file for writing K matrix.");
        }
    }

    return success;
}


