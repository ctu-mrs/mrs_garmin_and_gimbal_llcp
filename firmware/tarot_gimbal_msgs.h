#define GIMBAL_SET_CHANNELS_MSG_ID 60
#define GIMBAL_STATUS_MSG_ID 61
#define GIMBAL_HB_MSG_ID 62


struct __attribute__((__packed__)) gimbal_set_channels_msg
{
  uint8_t   id;
  uint16_t  channel_1;
  uint16_t  channel_2;
  bool 	    gimbal_mode;
  bool      gimbal_is_on;
};

struct __attribute__((__packed__)) gimbal_status_msg
{
  uint8_t   id;
  uint16_t  channel_1;
  uint16_t  channel_2;
  bool      gimbal_mode;
  bool      gimbal_is_on;
};

struct __attribute__((__packed__)) gimbal_hb_msg
{
  uint8_t   id;
  bool      is_running;
};
