#pragma once

// px4 基本引用的头文件 module workitem moduleParas 等
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

// uORB 相关头文件 包括 订阅 发布 Topics 等
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/mavlink_uorb.h>
#include <uORB/topics/mavlink_uorb_rx.h>

// 模块类对象
// 模块作为任务出现 继承自 ModuleBase 类，其中模板类型写<模块类本身>
// 模块要加入 work queue 继承自 ScheduledWorkItem 类
// 若模块中包含参数配置  继承自 ModuleParas 类
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
    // 相当于输入变量 - PX4 模块间通信通过 uORB 完成，因此订阅消息等同于输入，发布消息等同于输出
    // Subscription 分类
    // Subscription - 普通的订阅消息
    // SubscriptionInterval- 周期性的订阅消息
    // SubscriptionCallbackWorkItem - 回调式的订阅消息
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};		// 订阅传感器消息
	uORB::SubscriptionCallbackWorkItem _mavlink_uorb_rx_sub{this, ORB_ID(mavlink_uorb_rx)};	// 订阅 mavlink_uorb_rx 接收到的消息

	// Publications
    // 相当于模型输出
	uORB::Publication<mavlink_uorb_s> _mavlink_uorb_pub{ORB_ID(mavlink_uorb)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

};
