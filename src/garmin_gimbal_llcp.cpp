#include <ros/package.h>
#include <ros/ros.h>
#include <mrs_modules_msgs/Llcp.h>
#include <mrs_msgs/GimbalState.h>
#include <sensor_msgs/Range.h>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "../firmware/garmin_msgs.h"
#include "../firmware/tarot_gimbal_msgs.h"

// garmin specs
#define MAX_RANGE 4000  // cm
#define MIN_RANGE 10    // cm


namespace garmin_gimbal_llcp
{

/** class GarminGimbalLlcp **/

class GarminGimbalLlcp : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::Timer send_timer_;
  // Declaring our callbacks
  void callbackSendTimer(const ros::TimerEvent &event);
  void callbackReceiveMessage(const mrs_modules_msgs::LlcpConstPtr &msg);
  void callbackReceiveTarotGimbalCommand(const mrs_msgs::GimbalState &msg);

  ros::NodeHandle nh_;
  
  // Preparing llcp subscriber and publisher
  ros::Subscriber llcp_subscriber_;
  ros::Publisher  llcp_publisher_;  
  
  // Publishers for rangefinder topics
  ros::Publisher range_publisher_A_;
  ros::Publisher range_publisher_B_;
  
  // Subscriber and publisher for tarot gimbal control
  ros::Subscriber Tarot_Gimbal_State_subscriber_;
  ros::Publisher Tarot_Gimbal_Status_publisher_;
  

  bool     swap_garmins;
  
  std::string uav_name_;
  std::string garmin_A_frame_;
  std::string garmin_B_frame_;
  std::string tarot_gimbal_frame_;
  
  ros::Time interval_      = ros::Time::now();
  ros::Time last_received_ = ros::Time::now();

  bool is_initialized_ = false;
};



/** onInit()  **/

void GarminGimbalLlcp::onInit() {

  // Get node paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

   nh_.param("uav_name", uav_name_, std::string("uav")); 
  /* nh_.param("portname", portname_, std::string("/dev/ttyUSB0")); */
  /* nh_.param("baudrate", baudrate_, 115200); */
  /* nh_.param("publish_bad_checksum", publish_bad_checksum, false); */
  /* nh_.param("use_timeout", use_timeout, true); */
  /* nh_.param("serial_rate", serial_rate_, 5000); */
  /* nh_.param("serial_buffer_size", serial_buffer_size_, 1024); */
  
  nh_.param("swap_garmins", swap_garmins, false);

  // Publishers
  std::string postfix_A = swap_garmins ? "_up" : "";
  std::string postfix_B = swap_garmins ? "" : "_up";
  // Initialize rangefinder publishers with the correct name
  range_publisher_A_    = nh_.advertise<sensor_msgs::Range>("range" + postfix_A, 1);
  range_publisher_B_    = nh_.advertise<sensor_msgs::Range>("range" + postfix_B, 1);

  garmin_A_frame_       = uav_name_ + "/garmin" + postfix_A;
  garmin_B_frame_       = uav_name_ + "/garmin" + postfix_B;
  tarot_gimbal_frame_   = uav_name_ + "/gimbal";
  
  // Initialize LLCP publisher and subscriber
  llcp_publisher_ = nh_.advertise<mrs_modules_msgs::Llcp>("llcp_out", 1);
  llcp_subscriber_ = nh_.subscribe("llcp_in", 10, &GarminGimbalLlcp::callbackReceiveMessage, this, ros::TransportHints().tcpNoDelay());
  
  // Initialize publisher and subscriber for the gimbal control
  Tarot_Gimbal_Status_publisher_ = nh_.advertise<sensor_msgs::Range>("gimbal_status", 1);
  Tarot_Gimbal_State_subscriber_ = nh_.subscribe("gimbal_command", 10, &GarminGimbalLlcp::callbackReceiveTarotGimbalCommand, this, ros::TransportHints().tcpNoDelay());

  is_initialized_ = true;

}


// | ------------------------ callbacks ------------------------ |

/** callbackReceiveTarotGimbalCommand() **/
/* waiting for message from MRS Status and then sending it via LLCP to the tarot gimbal */

void GarminGimbalLlcp::callbackReceiveTarotGimbalCommand(const mrs_msgs::GimbalState &msg) {

  if (!is_initialized_) {
    return;
  }

  gimbal_set_channels_msg msg_out;
  mrs_modules_msgs::Llcp llcp_msg;
  
  /*  
   * Uncomment for debug
   */
  /*
  ROS_INFO_STREAM("[Garmin Gimbal LLCP]: Received msg from status, CH1: " << msg.gimbal_tilt <<
      		      ", CH2: " << msg.gimbal_pan <<
      		      ", gimbal mode: " << msg.fpv_mode <<
      		      ", gimbal on: " << msg.is_on
      		      );
  */
  
  /* message structure*/
  msg_out.id = GIMBAL_SET_CHANNELS_MSG_ID;
  msg_out.channel_1 = msg.gimbal_tilt;
  msg_out.channel_2 = msg.gimbal_pan;
  msg_out.gimbal_is_on = msg.is_on;
  msg_out.gimbal_mode = msg.fpv_mode;
    
  /* LLCP transmission */
  uint8_t *msg_ptr = (uint8_t *)&msg_out;

  for (int i = 0; i < sizeof(msg_out); i++) {
    llcp_msg.payload.push_back(msg_ptr[i]);
  }

  llcp_publisher_.publish(llcp_msg);  

}

/** callbackReceiveMessage() **/
/* callback for incoming LLCP messages  */
void GarminGimbalLlcp::callbackReceiveMessage(const mrs_modules_msgs::LlcpConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  // llcp is working with arrays, so we need to convert the payload from the ROS message into an array
  uint8_t payload_size = msg->payload.size();
  uint8_t payload_array[payload_size];
  std::copy(msg->payload.begin(), msg->payload.end(), payload_array);

  /* sorting the messages based on the message ID */
  switch (payload_array[0]) {

    case GARMIN_1_ID: {

      distance_msg *received_msg = (distance_msg *)payload_array;
      int16_t range = received_msg->distance;
      
      /* message structure */
      sensor_msgs::Range range_msg;
      range_msg.field_of_view  = 0.0523599;  // +-3 degree
      range_msg.max_range      = MAX_RANGE * 0.01;
      range_msg.min_range      = MIN_RANGE * 0.01;
      range_msg.radiation_type = sensor_msgs::Range::INFRARED;
      range_msg.header.stamp   = ros::Time::now();
      range_msg.header.frame_id = garmin_A_frame_;
      
      range_msg.range = range * 0.01; // convert to m
      
      if (range > MAX_RANGE) {
      	range_msg.range = std::numeric_limits<double>::infinity();
      } else if (range < MIN_RANGE) {
      	range_msg.range = -std::numeric_limits<double>::infinity();
      }
	    /* publishing to the rangefinder topic */
      try {
        range_publisher_A_.publish(range_msg);
      }
      catch (...) {
        ROS_ERROR("[Garmin Gimbal LLCP]: exception caught during publishing topic %s", range_publisher_A_.getTopic().c_str());
      }
      /*
       * Uncomment for debug
       */
      /*
      ROS_INFO_STREAM("[Garmin Gimbal LLCP]: Garmin 1 measured distance: " << unsigned(received_msg->distance));
      */
      break;
    }

    case GARMIN_2_ID: {
    
      distance_msg *received_msg = (distance_msg *)payload_array;      
      int16_t range = received_msg->distance;
      

      /* message structure */
      sensor_msgs::Range range_msg;
      range_msg.field_of_view  = 0.0523599;  // +-3 degree
      range_msg.max_range      = MAX_RANGE * 0.01;
      range_msg.min_range      = MIN_RANGE * 0.01;
      range_msg.radiation_type = sensor_msgs::Range::INFRARED;
      range_msg.header.stamp   = ros::Time::now();
      range_msg.header.frame_id = garmin_B_frame_;
      
      range_msg.range = range * 0.01; // convert to m
      
      if (range > MAX_RANGE) {
      	range_msg.range = std::numeric_limits<double>::infinity();
      } else if (range < MIN_RANGE) {
      	range_msg.range = -std::numeric_limits<double>::infinity();
      }
	
	    /* publishing to the rangefinder topic */
      try {
        range_publisher_B_.publish(range_msg);
      }
      catch (...) {
        ROS_ERROR("[Garmin gimbal LLCP]: exception caught during publishing topic %s", range_publisher_B_.getTopic().c_str());
      }
      break;
    }

    case GIMBAL_STATUS_MSG_ID: {

      gimbal_status_msg *received_msg = (gimbal_status_msg *)payload_array;
      mrs_msgs::GimbalState gimbal_state_msg;
    
      /*
       * Uncomment for debug
       */ 
			/*
      ROS_INFO_STREAM("[Garmin Gimbal LLCP]: Received msg from llcp, ID: " << received_msg->id <<
      		      ", CH1: " << received_msg->channel_1 <<
      		      ", CH2: " << received_msg->channel_2 <<
      		      ", gimbal mode: " << received_msg->gimbal_mode <<
      		      ", gimbal on: " << received_msg->gimbal_is_on
      		      );
      */
      
      gimbal_state_msg.gimbal_pan = received_msg->channel_2;
      gimbal_state_msg.gimbal_tilt = received_msg->channel_1;
      gimbal_state_msg.fpv_mode = received_msg->gimbal_mode;
      gimbal_state_msg.is_on = received_msg->gimbal_is_on;
      gimbal_state_msg.header.stamp = ros::Time::now();
      gimbal_state_msg.header.frame_id = tarot_gimbal_frame_;

      
      
          /* publishing to the rangefinder topic */
      try {
        Tarot_Gimbal_Status_publisher_.publish(gimbal_state_msg);
      }
      catch (...) {
        ROS_ERROR("[Garmin Gimbal LLCP]: exception caught during publishing topic %s", Tarot_Gimbal_Status_publisher_.getTopic().c_str());
      }
      break;
    }
    
    case HEARTBEAT_MSG_ID: {
      heartbeat_msg *received_msg = (heartbeat_msg *)payload_array;
      ROS_INFO_STREAM("[Garmin Gimbal LLCP]: Received heartbeat ");
      break;
    }
    default: {
      ROS_ERROR_STREAM("[Garmin Gimbal LLCP]: Received unknown message with id " << int(payload_array[0]));
      break;
    }
  }
}




/** | ------------------------ routines ------------------------ | **/


}  // namespace garmin_gimbal_llcp

PLUGINLIB_EXPORT_CLASS(garmin_gimbal_llcp::GarminGimbalLlcp, nodelet::Nodelet);
