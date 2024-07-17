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
#include "lqr_quaternion.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <fstream>
#include <iomanip>
#include <commander/px4_custom_mode.h>
#include <iostream>

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
                                  4096,  // Adjusted stack size
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
    //Initialize uORB subscriptions
    uORB::Subscription state_sub{ORB_ID(vehicle_status)};
    uORB::Subscription local_pos_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Publication<vehicle_rates_setpoint_s> cmd_raw_pub{ORB_ID(vehicle_rates_setpoint)};
    uORB::Subscription command_sub{ORB_ID(vehicle_command)};

    vehicle_status_s current_state{}; // Declare the variable

    // Wait for mission start command with a timeout
    hrt_abstime start_time = hrt_absolute_time();
    const hrt_abstime timeout = 500_s;

    // vehicle_command_s command;
    bool mission_started = false;
    vehicle_status_s previous_state = {}; // Initialize previous state
    bool not_mission_mode_logged = false; // Flag to track if the message has been logged


    while (!mission_started) {
        if (state_sub.update(&current_state)) {
            if (current_state.nav_state != previous_state.nav_state) {
                PX4_INFO("Current nav_state: %d", current_state.nav_state);
                previous_state = current_state; // Update previous state
                not_mission_mode_logged = false; // Reset the flag when state changes
            }

            if (current_state.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
                mission_started = true;
                PX4_INFO("Mission started");
            } else {
                if (!not_mission_mode_logged) {
                    PX4_INFO("Current state is not mission mode");
                    not_mission_mode_logged = true; // Set the flag to indicate the message has been logged
                }
            }
        }

        px4_usleep(100000); // Sleep for 100ms

        if (hrt_absolute_time() - start_time > timeout) {
            PX4_ERR("Timeout waiting for mission to start");
            return;
        }
    }

    // Open a log file
    std::ofstream log_file("/home/pawelj/Git_repos/PX4-Autopilot/src/modules/lqr_module/lqr_data.log", std::ios::out | std::ios::trunc);
    if (!log_file.is_open()) {
        PX4_ERR("Failed to open log file");
        return;
    }
    PX4_INFO("Log file opened");

    vehicle_local_position_s local_pos;

    vehicle_attitude_s attitude;

    vehicle_status_s status;

    LQR_Q::LQR_Quaternion lqr;
    lqr.initiated = true;
    try {
        lqr.readTrajectoryFromFile("/home/pawelj/Git_repos/PX4-Autopilot/src/modules/lqr_module/log_data_interpolated.csv");
    } catch (const std::exception &e) {
        PX4_ERR("Exception in readTrajectoryFromFile: %s", e.what());
        return;
    }

    // std::cout << "Trajectory read from file" << std::endl;
    // if (lqr.getStates().size() < 10) {
    //     PX4_ERR("Not enough states in the trajectory file");
    //     return;
    // }

    // Print the first 10 states
    // for (int i = 0; i < 10; i++) {
    //     const auto& state = lqr.getStates()[i];
    //     std::cout << "State " << i << ": "
    //               << "timestamp=" << state.timestamp << ", "
    //               << "position=[" << state.position_W.transpose() << "], "
    //               << "velocity=[" << state.velocity_W.transpose() << "], "
    //               << "acceleration=[" << state.acceleration_W.transpose() << "], "
    //               << "orientation=[" << state.orientation_W_B.w() << ", "
    //                                 << state.orientation_W_B.x() << ", "
    //                                 << state.orientation_W_B.y() << ", "
    //                                 << state.orientation_W_B.z() << "], "
    //               << "angular_velocity=[" << state.angular_velocity_B.transpose() << "]"
    //               << std::endl;
    // }

    // Initialize timing variables
    hrt_abstime last_calculation_time = hrt_absolute_time();
    const hrt_abstime calculation_interval = 1000000; // Perform calculations every 100ms


    // Main loop
    while (!should_exit()) {
        // Update the vehicle status
        if (state_sub.update(&status)) {
            current_state = status;
        }

        // Update local position and attitude
        if (local_pos_sub.update(&local_pos) && attitude_sub.update(&attitude)) {
            // Check if enough time has passed since the last calculation
            if (hrt_absolute_time() - last_calculation_time >= calculation_interval) {
                try {
                    lqr.topicCallback(local_pos, attitude);
                } catch (const std::exception &e) {
                    PX4_ERR("Exception in topicCallback: %s", e.what());
                    return;
                }

                control_vector_t output = lqr.getTrajectoryControl() - lqr.getGain() * lqr.getError();

                // // Clamping the output values
                // for (int i = 0; i < 3; i++) {
                //     if (output(i) > 1.0) {
                //         output(i) = 1.0;
                //     } else if (output(i) < -1.0) {
                //         output(i) = -1.0;
                //     }
                // }
                // if (output(3) > 0) {
                //     output(3) = 0;
                // } else if (output(3) < -1.0) {
                //     output(3) = -1;
                // }

                lqr.setOutput(output);

                Eigen::Vector3d cmd_body_rate_baselink;
                cmd_body_rate_baselink << lqr.getOutput()(0),
                                        lqr.getOutput()(1),
                                        lqr.getOutput()(2);

                // Transform from ENU to NED
                Eigen::Vector3d cmd_body_rate_ned;
                cmd_body_rate_ned << cmd_body_rate_baselink(1),  // North (y in ENU)
                                    cmd_body_rate_baselink(0),  // East (x in ENU)
                                    -cmd_body_rate_baselink(2); // Down (-z in ENU)

                vehicle_rates_setpoint_s bodyrate_msg{};
                bodyrate_msg.roll = cmd_body_rate_ned(0);
                bodyrate_msg.pitch = cmd_body_rate_ned(1);
                bodyrate_msg.yaw = cmd_body_rate_ned(2);
                bodyrate_msg.thrust_body[2] = -lqr.getOutput()(3)/20; // Thrust should also be negated for NED

                try {
                    // Ensure the values are valid before logging
                    if (!std::isnan(bodyrate_msg.roll) && !std::isnan(bodyrate_msg.pitch) &&
                        !std::isnan(bodyrate_msg.yaw) && !std::isnan(bodyrate_msg.thrust_body[0])) {

                        // Log the bodyrate_msg in a formatted manner
                        const int space = 20; // Adjust the width as needed
                        log_file << std::fixed << std::setprecision(8)
                                << "Roll: " << std::right << std::setw(space) << bodyrate_msg.roll << "  |  "
                                << "Pitch: " << std::right << std::setw(space) << bodyrate_msg.pitch << "  |  "
                                << "Yaw: " << std::right << std::setw(space) << bodyrate_msg.yaw << "  |  "
                                << "Thrust[0]: " << std::right << std::setw(space) << bodyrate_msg.thrust_body[0] << "  |  "
                                << "Thrust[1]: " << std::right << std::setw(space) << bodyrate_msg.thrust_body[1] << "  |  "
                                << "Thrust[2]: " << std::right << std::setw(space) << bodyrate_msg.thrust_body[2] << std::endl;
                    }
                } catch (const std::exception &e) {
                    PX4_ERR("Exception while writing to log file: %s", e.what());
                }

                // Update the last calculation time
                last_calculation_time = hrt_absolute_time();
            }
        }

        px4_usleep(1000000); // Sleep for 10ms to reduce CPU usage
    }

    log_file.close();
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
