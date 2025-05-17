#ifndef MAVLINK_UORB_HPP
#define MAVLINK_UORB_HPP

#include <uORB/topics/mavlink_uorb.h>

class MavlinkStreamMavlinkUorb : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMavlinkUorb(mavlink); }

	static constexpr const char *get_name_static() { return "MAVLINK_UORB"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MAVLINK_UORB; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _mavlink_uorb_sub.advertised() ? MAVLINK_MSG_ID_MAVLINK_UORB_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMavlinkUorb(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _mavlink_uorb_sub{ORB_ID(mavlink_uorb)};

	bool send() override
	{
		mavlink_uorb_s mavlink_uorb_data;

		if (_mavlink_uorb_sub.update(&mavlink_uorb_data)) {
			mavlink_mavlink_uorb_t msg{};
			msg.time_usec = mavlink_uorb_data.timestamp;
			// memcpy(msg.name, debug.name, sizeof(msg.name));
			// msg.name[sizeof(msg.name) - 1] = '\0'; // enforce null termination
			msg.accel0 = mavlink_uorb_data.accel[0];
			msg.accel1 = mavlink_uorb_data.accel[1];
			msg.accel2 = mavlink_uorb_data.accel[2];

			mavlink_msg_mavlink_uorb_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // MAVLINK_UORB_HPP
