/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "lqr_module.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>

int LQRModule::print_status()
{
    PX4_INFO("Running");
    // TODO: print additional runtime information about the state of the module

    return 0;
}

int LQRModule::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int LQRModule::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("lqr_module",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  2048,
                                  (px4_main_t)&LQRModule::run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

LQRModule *LQRModule::instantiate(int argc, char *argv[])
{
    return new LQRModule();
}

LQRModule::LQRModule()
    : ModuleParams(nullptr)
{
}

void LQRModule::run()
{
    // LQR computation logic
    LQR<2, 1> lqr_instance;
    LQR<2, 1>::state_matrix_t A_;
    LQR<2, 1>::control_gain_matrix_t B_;
    LQR<2, 1>::state_matrix_t Q_;
    LQR<2, 1>::control_matrix_t R_;
    LQR<2, 1>::control_feedback_t K_;
    A_ << 0, 1,
        0.01, 0;
    B_ << 0,
        1;
    Q_ << 1, 0,
        0, 2;
    R_ << 2;
    lqr_instance.compute(Q_, R_, A_, B_, K_);

    PX4_INFO("K: %f, %f", K_(0, 0), K_(1, 0));

    // Exit after computation
    PX4_INFO("LQR computation completed, exiting.");
}

void LQRModule::parameters_update(bool force)
{
    // check for parameter updates
    if (_parameter_update_sub.updated() || force) {
        // clear update
        parameter_update_s update;
        _parameter_update_sub.copy(&update);

        // update parameters from storage
        updateParams();
    }
}

void LQRModule::run_tramp(int argc, char *argv[])
{
    LQRModule *instance = instantiate(argc, argv);
    if (instance) {
        instance->run();
        delete instance;
    }
}

int LQRModule::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
This module implements the LQR controller.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("lqr_module", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int lqr_module_main(int argc, char *argv[])
{
    return LQRModule::main(argc, argv);
}
