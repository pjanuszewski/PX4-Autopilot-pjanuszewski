/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef LQR_HPP
#define LQR_HPP

#pragma once

#include "CARE.hpp"


/*!
 * \ingroup LQR
 *
 * \brief continuous-time infinite-horizon LQR
 *
 * Implements continous-time infinite-horizon LQR.
 * Resulting feedback law will take the form
 * \f[
 * u_{fb} = -K \cdot (x - x_{ref})
 * \f]
 *
 * @tparam STATE_DIM
 * @tparam CONTROL_DIM
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class LQR
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> control_gain_matrix_t;
    typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_feedback_t;

    //! design the infinite-horizon LQR controller.
    /*!
	 * @param Q state-weighting matrix
	 * @param R control input weighting matrix
	 * @param A linear system dynamics matrix A
	 * @param B linear system dynamics matrix B
	 * @param K control feedback matrix K (to be designed)
	 * @param RisDiagonal set to true if R is a diagonal matrix (efficiency boost)
	 * @param solveRiccatiIteratively
	 * 	use closed-form solution of the infinite-horizon Riccati Equation
	 * @return success
	 */
    bool compute(const state_matrix_t& Q,
        const control_matrix_t& R,
        const state_matrix_t& A,
        const control_gain_matrix_t& B,
        control_feedback_t& K,
        bool RisDiagonal = false,
        bool solveRiccatiIteratively = false);

	void lqrtest();
private:
    CARE<STATE_DIM, CONTROL_DIM> care_;  // continuous-time algebraic riccati equation
};

#include "LQR-impl.hpp"

#endif // LQR_HPP
