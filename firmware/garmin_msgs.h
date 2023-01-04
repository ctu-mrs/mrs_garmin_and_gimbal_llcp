#define GARMIN_1_ID 0x00
#define GARMIN_2_ID 0x01
#define HEARTBEAT_MSG_ID 51

struct __attribute__((__packed__)) distance_msg
{
  uint8_t  id;
  uint16_t  distance;
};

struct __attribute__((__packed__)) heartbeat_msg
{
  uint8_t  id;
  bool     is_running;
};
