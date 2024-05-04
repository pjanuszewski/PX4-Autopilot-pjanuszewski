// #include <iostream>
// #include <Eigen/Dense>
// #include <math.h>
// #include <control-toolbox/ct_optcon/include/ct/optcon/optcon.h>
// #include <px4_platform_common/log.h>
// #include <px4_platform_common/app.h>
// #include "lqr_app_task.hpp"

// int lqr_app_task::_task_id = -1;

// int lqr_app_task::print_lqr_status()
// {
//     PX4_INFO("Running lqr_app_task...");
//     // Example of additional runtime information
//     PX4_INFO("Task ID: %d", _task_id);
//     return 0;
// }

// static int lqr_task_entry(int argc, char *argv[]) {
//     auto lqr = lqr_app_task::instantiate_lqr();
//     if (!lqr) {
//         PX4_ERR("Failed to instantiate LQR");
//         return -1;
//     }

//     // Placeholder for LQR computations or operations
//     // Example: lqr->compute(Q, R, A, B, K);
//     PX4_INFO("LQR Task Running");

//     return 0;
// }

// std::unique_ptr<ct::optcon::LQR<lqr_app_task::stateDim, lqr_app_task::controlDim>> lqr_app_task::instantiate_lqr()
// {
//     //auto lqr = std::make_unique<ct::optcon::LQR<lqr_app_task::stateDim, lqr_app_task::controlDim>>();
//     //PX4_INFO("LQR instantiated with stateDim=%d, controlDim=%d", lqr_app_task::stateDim, lqr_app_task::controlDim);
//     //return lqr;
//     return nullptr;
// }

// int lqr_app_task::task_spawn(int argc, char *argv[])
// {
//     _task_id = px4_task_spawn_cmd("lqr_app_task",
//                                   SCHED_DEFAULT,
//                                   SCHED_PRIORITY_MIN,
//                                   8192,  // Stack size
//                                   (px4_main_t)&lqr_task_entry,
//                                   (char *const *)argv);

//     if (_task_id < 0) {
//         PX4_ERR("Failed to spawn task: %d", _task_id);
//         return -errno;
//     }

//     PX4_INFO("LQR task spawned with ID: %d", _task_id);
//     return 0;
// }

// int lqr_app_task::task_stop()
// {
//     if (_task_id != -1) {
//         px4_task_delete(_task_id);
//         _task_id = -1;
//         PX4_INFO("LQR task stopped");
//         return 0;
//     }

//     PX4_INFO("No LQR task running");
//     return -1;
// }

// extern "C" __EXPORT int lqr_app_task_main(int argc, char *argv[])
// {
//     if (argc > 1) {
//         const char *cmd = argv[1];
//         if (!strcmp(cmd, "start")) {
//             int result = lqr_app_task::task_spawn(argc - 1, &argv[1]);
//             lqr_app_task::print_lqr_status(); // Assuming print_lqr_status is a static method
//             return result;
//         } else if (!strcmp(cmd, "stop")) {
//             return lqr_app_task::task_stop();
//         } else {
//             PX4_INFO("Unknown command: %s", cmd);
//             return -1;
//         }
//     }

//     PX4_INFO("Usage: lqr_app_task {start|stop}");
//     PX4_INFO("exiting");
//     return 0;
// }
// >
