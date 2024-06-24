#pragma once
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <matrix/matrix/math.hpp>
#include "LQR.hpp"
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/slew_rate/SlewRate.hpp>

using namespace time_literals;

extern "C" __EXPORT int lqr_module_main(int argc, char *argv[]);

class LQRModule : public ModuleBase<LQRModule>, public ModuleParams
{
public:
    LQRModule();

    virtual ~LQRModule() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static LQRModule *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

    static void run_tramp(int argc, char *argv[]);


private:
    /**
     * Check for parameter changes and update them if needed.
     * @param parameter_update_sub uorb subscription to parameter_update
     * @param force for a parameter update
     */
    void parameters_update(bool force = false);

    // Subscriptions
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

};
