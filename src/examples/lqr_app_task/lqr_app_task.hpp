// /****************************************************************************
//  *
//  *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in
//  *    the documentation and/or other materials provided with the
//  *    distribution.
//  * 3. Neither the name PX4 nor the names of its contributors may be
//  *    used to endorse or promote products derived from this software
//  *    without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  *
//  ****************************************************************************/

// #pragma once

// #include <px4_platform_common/module.h>
// #include <px4_platform_common/module_params.h>
// #include <uORB/SubscriptionInterval.hpp>
// #include <uORB/topics/parameter_update.h>
// #include <memory>
// #include <Eigen/Dense>
// #include <control-toolbox/ct_optcon/include/ct/optcon/optcon.h>

// using namespace time_literals;

// class lqr_app_task : public ModuleBase<lqr_app_task>, public ModuleParams
// {
// public:
//     lqr_app_task();
//     virtual ~lqr_app_task() = default;

//     /** @see ModuleBase */
//     static int _task_id;
//     static int task_spawn(int argc, char *argv[]);
//     static int task_stop();
//     static int print_lqr_status();
//     static constexpr int stateDim = 8;
//     static constexpr int controlDim = 4;
//     /** Helper function to instantiate LQR */
//     static std::unique_ptr<ct::optcon::LQR<stateDim, controlDim>> instantiate_lqr();

// private:

//     DEFINE_PARAMETERS(
//         (ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
//         (ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
//     )

//     // Subscriptions
//     uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
// };
