
#include "mavlink_uorb_test.hpp"

// 自定义时间函数（应该在其他模块中已有定义 临时方法）
#define _s(x) (x * 1000000)		// s --> us
#define _ms(x) (x * 1000)		// ms --> us

// 构造函数
MavlinkUorbTest::MavlinkUorbTest() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	// 继承了 ScheduledWorkItem 类，需要初始化 ScheduledWorkItem
    // 将任务加入 wq_configurations::test1 队列，不同的队列具有不同的优先级
    // 任务队列的定义见:
    // /platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp
}

// 析构函数
MavlinkUorbTest::~MavlinkUorbTest()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

// 初始化函数 注册回调  或确定模块的执行方式
bool MavlinkUorbTest::init()
{
	/// 若采用注册回调的方式，则使用以下代码
    // 以下设置为在每次 sensoe_accel 更新时就执行此模块
	// if (!_sensor_accel_sub.registerCallback()) {		// 在每次sensor_accel发布时执行 Run()
	// PX4_ERR("callback registration failed");
	//	return false;
	// }

	// 也可以设置为固定频率运行，以下设置运行的时间间隔，单位为us
	// alternatively, Run on fixed interval
	ScheduleOnInterval(_s(1));	 // 1s interval, 1 Hz rate

	return true;
}

// 模块功能的核心函数，系统会根据设置的任务调度方法执行 Run 函数
void MavlinkUorbTest::Run()
{
	// 异常处理
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// 性能监控
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// 模块功能 TODO
    // Example：读取 sensor_accel 消息，并将加速度发布到 mavlink_uorb 消息中
	if (_sensor_accel_sub.updated()) {		// 检测更新
		sensor_accel_s accel;

		if (_sensor_accel_sub.copy(&accel)) {		// 获取数据
			// publish some data
			mavlink_uorb_s mavlink_uorb_data{};		// 组装消息结构体
			mavlink_uorb_data.accel[0] = accel.x;
			mavlink_uorb_data.accel[1] = accel.y;
			mavlink_uorb_data.accel[2] = accel.z;
			mavlink_uorb_data.timestamp = hrt_absolute_time();	// 添加时间戳
			_mavlink_uorb_pub.publish(mavlink_uorb_data);		// 发布 uORB 消息

			// 打印输出，可以利用类似的方法进行调试
			PX4_INFO("accel: %8.4f %8.4f %8.4f", (double)accel.x, (double)accel.y, (double)accel.z);
			PX4_INFO("mavlink_uorb: %8.4f %8.4f %8.4f", (double)mavlink_uorb_data.accel[0], (double)mavlink_uorb_data.accel[1], (double)mavlink_uorb_data.accel[2]);
		}
	}

	perf_end(_loop_perf);
}

// 任务生成函数，和 init 函数类似，在任务启动时设置任务相关的参数
// 例如任务中类对象的实例化并保存
int MavlinkUorbTest::task_spawn(int argc, char *argv[])
{
	MavlinkUorbTest *instance = new MavlinkUorbTest();	// 实例化任务类

	if (instance) {
		_object.store(instance);			// 若实例化成功，则存储
		_task_id = task_id_is_work_queue;	// 记录任务id

		if (instance->init()) {				// 进行任务初始化
			return PX4_OK;					// 初始化成功
		}

	} else {						// 处理失败等
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

// 打印模块状态
int MavlinkUorbTest::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

// 用户指令 默认会包含 start stop status
// 指令调用格式 <module_name> <command>
// 例 pxh> mavlink_uorb_test start
// 这些指令可以在 pxh命令行、nsh命令行、系统启动文件 中执行
int MavlinkUorbTest::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

// 输出模块用法
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

// 模块主函数，提供模块的调用入口
// 主函数的命名应为 <module_name>_main  并保留输入 argc argv
extern "C" __EXPORT int mavlink_uorb_test_main(int argc, char *argv[])
{
	return MavlinkUorbTest::main(argc, argv);
}



