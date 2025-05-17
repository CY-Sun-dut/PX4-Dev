#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/mavlink_uorb.h>

class MavlinkUorbTest : public ModuleBase<MavlinkUorbTest>, public px4::ScheduledWorkItem
{
public:
	MavlinkUorbTest();
	~MavlinkUorbTest() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};

	// Publications
	uORB::Publication<mavlink_uorb_s> _mavlink_uorb_pub{ORB_ID(mavlink_uorb)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

};
