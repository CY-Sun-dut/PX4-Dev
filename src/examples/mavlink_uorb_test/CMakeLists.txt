px4_add_module(
	MODULE examples__mavlink_uorb_test		# 模块名从src文件夹开始，文件路径的 / 替换为双下划线 __
	MAIN mavlink_uorb_test					# 模块入口与 main 函数对应  <module_name>(_main)
	COMPILE_FLAGS
	SRCS
		mavlink_uorb_test.cpp
		mavlink_uorb_test.hpp
	DEPENDS
		px4_work_queue						# 模块中使用 WorkQueue 添加依赖
	)
