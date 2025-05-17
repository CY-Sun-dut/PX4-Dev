
#include "mavlink_uorb_test.hpp"

// 自定义时间函数（应该在其他模块中已有定义 临时方法）
#define _s(x) (x * 1000000)		// s --> us
#define _ms(x) (x * 1000)		// ms --> us

// 构造函数
MavlinkUorbTest::MavlinkUorbTest() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{

}

// 析构函数
MavlinkUorbTest::~MavlinkUorbTest()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

// 初始化函数 注册回调
bool MavlinkUorbTest::init()
{
	// execute Run() on every sensor_accel publication
	// if (!_sensor_accel_sub.registerCallback()) {		// 在每次sensor_accel发布时执行 Run()
	// PX4_ERR("callback registration failed");
	//	return false;
	// }

	// 或设定固定频率执行
	// alternatively, Run on fixed interval
	ScheduleOnInterval(_s(1));	 // 1s interval, 1 Hz rate

	return true;
}

void MavlinkUorbTest::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// get the sensor accel
	if (_sensor_accel_sub.updated()) {
		sensor_accel_s accel;

		if (_sensor_accel_sub.copy(&accel)) {
			// publish some data
			mavlink_uorb_s mavlink_uorb_data{};
			mavlink_uorb_data.accel[0] = accel.x;
			mavlink_uorb_data.accel[1] = accel.y;
			mavlink_uorb_data.accel[2] = accel.z;
			mavlink_uorb_data.timestamp = hrt_absolute_time();
			_mavlink_uorb_pub.publish(mavlink_uorb_data);

			PX4_INFO("accel: %8.4f %8.4f %8.4f", (double)accel.x, (double)accel.y, (double)accel.z);
			PX4_INFO("mavlink_uorb: %8.4f %8.4f %8.4f", (double)mavlink_uorb_data.accel[0], (double)mavlink_uorb_data.accel[1], (double)mavlink_uorb_data.accel[2]);
		}
	}

	perf_end(_loop_perf);
}

int MavlinkUorbTest::task_spawn(int argc, char *argv[])
{
	MavlinkUorbTest *instance = new MavlinkUorbTest();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MavlinkUorbTest::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int MavlinkUorbTest::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MavlinkUorbTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a uORB message link with MAVLink.
This module subscribes to the sensor_accel topic and publishes the data to the mavlink_uorb_test topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mavlink_uorb_test", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mavlink_uorb_test_main(int argc, char *argv[])
{
	return MavlinkUorbTest::main(argc, argv);
}



