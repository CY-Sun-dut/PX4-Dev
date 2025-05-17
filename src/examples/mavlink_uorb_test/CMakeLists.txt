px4_add_module(
	MODULE examples__mavlink_uorb_test
	MAIN mavlink_uorb_test
	COMPILE_FLAGS
	SRCS
		mavlink_uorb_test.cpp
		mavlink_uorb_test.hpp
	DEPENDS
		px4_work_queue
	)
