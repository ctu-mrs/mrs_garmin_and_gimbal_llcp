extern "C" {
#include <llcp.h>
}

#include "garmin_msgs.h"
#include "tarot_gimbal_msgs.h"

LLCP_Receiver_t llcp_receiver;

#include <Wire.h>
#include <stdio.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>

#define Lidar_1_en 2
#define Lidar_2_en 3

#define TX_BUFFER_LEN 255
uint8_t tx_buffer[TX_BUFFER_LEN];

#define RC_CHANNEL_MIN 1000 //These values will be mapped onto the SBus Channels, as they are easier to process for humans
#define RC_CHANNEL_MAX 2000

#define SBUS_MIN_OFFSET 192 //Sbus uses this range of values as min/max of the channels
#define SBUS_MAX_OFFSET 1792

#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 16 //ms

SoftwareSerial swSerial(10, 4, true); // RX, TX

uint16_t channel1_set = 1500;
uint16_t channel2_set = 1500;

bool mode = true;
bool is_on = true;

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
uint16_t rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;

int cal_cnt = 0;
int reset_cnt = 0;

int loops = 0;
uint16_t dist1 = 0;
uint16_t dist2 = 0;
long last_heartbeat = millis();

bool got_heart = false;

uint16_t num_msg_received = 0;

void setup()
{
  pinMode(Lidar_1_en, OUTPUT);
  pinMode(Lidar_2_en, OUTPUT);

  digitalWrite(Lidar_1_en, 0);
  digitalWrite(Lidar_2_en, 0);
  Wire.begin();
  Wire.setClock(40000UL);

  for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
    rcChannels[i] = 1500;
  }

  rcChannels[2] = 2000;
  swSerial.begin(100000);
  
  Serial.begin(115200); // Initialize serial connection to display distance readings
  // inicialize the llcp receiver struct
  llcp_initialize(&llcp_receiver);
  
  delay(50); // give both lidars some time to turn off
  digitalWrite(Lidar_1_en, 1);
  delay(50);
  byte serial_number[2];
  bool succesful_config = true;
  //read and write back serial to begin address change
  read_i2c(0x96, 2, serial_number, false, 0x62);
  succesful_config &= write_i2c(0x18, serial_number[0], 0x62);
  succesful_config &= write_i2c(0x19, serial_number[1], 0x62);
  //set the new address
  succesful_config &= write_i2c(0x1a, 0x64, 0x62);
  //disable the old address, now the new address should be used
  succesful_config &= write_i2c(0x1e, 0x08, 0x62);
  succesful_config &= configure(0, 0x64); // configure lidar on new address

  digitalWrite(Lidar_2_en, 1); // turn on the other lidar
  delay(50);
  //read and write back serial to begin address change
  read_i2c(0x96, 2, serial_number, false, 0x62);
  succesful_config &= write_i2c(0x18, serial_number[0], 0x62);
  succesful_config &= write_i2c(0x19, serial_number[1], 0x62);
  //set the new address
  succesful_config &= write_i2c(0x1a, 0x66, 0x62);
  //disable the old address, now the new address should be used
  succesful_config &= write_i2c(0x1e, 0x08, 0x62);
  succesful_config &= configure(0, 0x66); // configure lidar on new address
  wdt_enable(WDTO_250MS);
  //Serial.print("All is set.");

  /*
    if(!succesful_config){
    //config failed, restart program
    delay(1);
    asm volatile ("  jmp 0");
    }
  */
}

void loop()
{
  wdt_reset();
  
  uint32_t currentMillis = millis();

  /** Receiving LLCP message and sending sbus packet **/
  if (currentMillis > sbusTime) {
    receive_message();
    rcChannels[0] = channel1_set;
    rcChannels[1] = channel2_set;
    if (mode) {
      rcChannels[2] = 2000;
    } else {
      rcChannels[2] = 1000;
    }    
    sbusPreparePacket(sbusPacket, rcChannels, !is_on, false);
    swSerial.write(sbusPacket, SBUS_PACKET_LENGTH);
    sbusTime = currentMillis + SBUS_UPDATE_RATE;    
  }
  
  /**
   * Getttting a measurement from the garmin rangefinders
   * At the beginning of every 100 readings,
   * take a measurement with receiver bias correction  
   **/
  if ( cal_cnt == 0 ) {
    dist1 = get_distance(true, 0x64);      // With bias correction
    dist2 = get_distance(true, 0x66);      // With bias correction
  } else {
    dist1 = get_distance(false, 0x64); // Without bias correction
    dist2 = get_distance(false, 0x66); // Without bias correction
  }
  // Increment reading counter
  cal_cnt++;
  if (cal_cnt == 100) {
    cal_cnt = 0;
  }
  if (dist1 == -1 || dist2 == -1) {
    reset_cnt++;
  } else {
    reset_cnt = 0;
  }
  if (reset_cnt == 25) {
    asm volatile ("  jmp 0");
  }


  /** 
   * Sending distance measurements 
   * and gimbal status via LLCP  
   **/
  /* Uncomment for debug */ 
  /* 
  Serial.print("Garmin 1: ");
  Serial.print(dist1);
  Serial.print("   Garmin 2: ");
  Serial.println(dist2);
  */
  send_distance(dist1, 0x00);
  send_distance(dist2, 0x01);
  send_status();

  /* Sending device heartbeat */
  if (millis() - last_heartbeat >= 1000) {
    last_heartbeat = millis();
    send_heartbeat();
  }

  delay(1);
}

bool write_i2c(char myAddress, char myValue, char lidarliteAddress)
{
  Wire.beginTransmission((int)lidarliteAddress);
  Wire.write((int)myAddress); // Set register for write
  Wire.write((int)myValue); // Write myValue to register

  // A nack means the device is not responding, report the error over serial
  int nackCatcher = Wire.endTransmission();
  if (nackCatcher != 0)
  {
    return false;
    //asm volatile ("  jmp 0");
  }

  delay(1); // 1 ms delay for robustness with successive reads and writes
  return true;
}

bool configure(int configuration, char lidarliteAddress)
{
  bool ret_val = true;
  ret_val &= write_i2c(0x02, 0x80, lidarliteAddress);
  ret_val &= write_i2c(0x04, 0x08, lidarliteAddress);
  ret_val &= write_i2c(0x1c, 0x00, lidarliteAddress);
  return ret_val;
}

int get_distance(bool biasCorrection, char lidarliteAddress)
{
  if (biasCorrection)
  {
    // Take acquisition & correlation processing with receiver bias correction
    write_i2c(0x00, 0x04, lidarliteAddress);
  }
  else
  {
    // Take acquisition & correlation processing without receiver bias correction
    write_i2c(0x00, 0x03, lidarliteAddress);
  }
  // Array to store high and low bytes of distance
  byte distanceArray[2];
  // Read two bytes from register 0x8f (autoincrement for reading 0x0f and 0x10)
  read_i2c(0x8f, 2, distanceArray, true, lidarliteAddress);
  // Shift high byte and add to low byte
  int distance = (distanceArray[0] << 8) + distanceArray[1];
  return (distance);
}


void read_i2c(char myAddress, int numOfBytes, byte arrayToSave[2], bool monitorBusyFlag, char lidarliteAddress)
{
  int busyFlag = 0; // busyFlag monitors when the device is done with a measurement
  if (monitorBusyFlag)
  {
    busyFlag = 1; // Begin read immediately if not monitoring busy flag
  }
  int busyCounter = 0; // busyCounter counts number of times busy flag is checked, for timeout

  while (busyFlag != 0) // Loop until device is not busy
  {
    // Read status register to check busy flag
    Wire.beginTransmission((int)lidarliteAddress);
    Wire.write(0x01); // Set the status register to be read

    // A nack means the device is not responding, report the error over serial
    int nackCatcher = Wire.endTransmission();
    if (nackCatcher != 0)
    {
      if (numOfBytes == 2) {
        arrayToSave[0] = 255;
        arrayToSave[1] = 255;
      }
      return;
    }

    Wire.requestFrom((int)lidarliteAddress, 1); // Read register 0x01
    busyFlag = bitRead(Wire.read(), 0); // Assign the LSB of the status register to busyFlag

    busyCounter++; // Increment busyCounter for timeout

    // Handle timeout condition, exit while loop and goto bailout
    if (busyCounter > 999)
    {
      busyCounter = 0;
      if (numOfBytes == 2) {
        arrayToSave[0] = 255;
        arrayToSave[1] = 255;
      }
      return;
    }
  }

  // Device is not busy, begin read
  if (busyFlag == 0)
  {
    Wire.beginTransmission((int)lidarliteAddress);
    Wire.write((int)myAddress); // Set the register to be read

    // A nack means the device is not responding, report the error over serial
    int nackCatcher = Wire.endTransmission();
    if (nackCatcher != 0)
    {
      if (numOfBytes == 2) {
        arrayToSave[0] = 255;
        arrayToSave[1] = 255;
      }
      return;
    }

    // Perform read of 1 or 2 bytes, save in arrayToSave
    Wire.requestFrom((int)lidarliteAddress, numOfBytes);
    int i = 0;
    if (numOfBytes <= Wire.available())
    {
      while (i < numOfBytes)
      {
        arrayToSave[i] = Wire.read();
        i++;
      }
    }
  }

  // bailout reports error over serial
  if (busyCounter > 999)
  {
    busyCounter = 0;
    if (numOfBytes == 2) {
      arrayToSave[0] = 255;
      arrayToSave[1] = 255;
    }
    return;
  }
}

bool receive_message() {
  uint16_t msg_len;
  bool got_valid_msg = false;
  LLCP_Message_t* llcp_message_ptr;

  while (Serial.available() > 0) {
    bool checksum_matched;
    uint8_t char_in = Serial.read();

    //individual chars are processed one by one by llcp, if a complete message is received, llcp_processChar() returns true
    if (llcp_processChar(char_in, &llcp_receiver, &llcp_message_ptr, &checksum_matched)) {
      if (checksum_matched) {
        num_msg_received++;
        switch (llcp_message_ptr->payload[0]) {
          case GIMBAL_SET_CHANNELS_MSG_ID : {

              gimbal_set_channels_msg *received_msg = (gimbal_set_channels_msg *)llcp_message_ptr;

              channel1_set = received_msg->channel_1;
              channel2_set = received_msg->channel_2;
              mode = received_msg->gimbal_mode;
              is_on = received_msg->gimbal_is_on;

              got_valid_msg = true;
              break;
            }
        }

        return true;
      }
    }
  }
  return got_valid_msg;
}


void send_heartbeat() {

  heartbeat_msg my_msg;
  uint16_t msg_len;

  // fill the message with data

  my_msg.id = HEARTBEAT_MSG_ID;
  my_msg.is_running = true;

  //llcp_prepareMessage will fill your TX buffer while returning the number of bytes written
  msg_len = llcp_prepareMessage((uint8_t*)&my_msg, sizeof(my_msg), tx_buffer);

  //send the message out over the serial line
  for (int i = 0; i < msg_len; i++) {
    Serial.write(tx_buffer[i]);
  }
}
void send_status() {

  gimbal_status_msg my_msg;
  uint16_t msg_len;

  // fill the message with data

  my_msg.id = GIMBAL_STATUS_MSG_ID ;
  my_msg.channel_1 = rcChannels[0];
  my_msg.channel_2 = rcChannels[1];
  my_msg.gimbal_mode = mode;
  my_msg.gimbal_is_on = is_on;

  //llcp_prepareMessage will fill your TX buffer while returning the number of bytes written
  msg_len = llcp_prepareMessage((uint8_t*)&my_msg, sizeof(my_msg), tx_buffer);

  //send the message out over the serial line
  for (int i = 0; i < msg_len; i++) {
    Serial.write(tx_buffer[i]);
  }
}

void send_distance(uint16_t dist, uint8_t msg_id) {

  distance_msg my_msg;
  uint16_t msg_len;

  // fill the message with data
  my_msg.id = msg_id;
  my_msg.distance = dist;

  //llcp_prepareMessage will fill your TX buffer while returning the number of bytes written
  msg_len = llcp_prepareMessage((uint8_t*)&my_msg, sizeof(my_msg), tx_buffer);

  //send the message out over the serial line
  for (int i = 0; i < msg_len; i++) {
    Serial.write(tx_buffer[i]);
  }
}

void sbusPreparePacket(uint8_t packet[], uint16_t channels[], bool isSignalLoss, bool isFailsafe) {

  static int output[SBUS_CHANNEL_NUMBER] = {0};

  for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
    output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
  }

  uint8_t stateByte = 0x00;
  if (isSignalLoss) {
    stateByte |= SBUS_STATE_SIGNALLOSS;
  }
  if (isFailsafe) {
    stateByte |= SBUS_STATE_FAILSAFE;
  }
  packet[0] = SBUS_FRAME_HEADER; //Header

  packet[1] = (uint8_t) (output[0] & 0x07FF);
  packet[2] = (uint8_t) ((output[0] & 0x07FF) >> 8 | (output[1] & 0x07FF) << 3);
  packet[3] = (uint8_t) ((output[1] & 0x07FF) >> 5 | (output[2] & 0x07FF) << 6);
  packet[4] = (uint8_t) ((output[2] & 0x07FF) >> 2);
  packet[5] = (uint8_t) ((output[2] & 0x07FF) >> 10 | (output[3] & 0x07FF) << 1);
  packet[6] = (uint8_t) ((output[3] & 0x07FF) >> 7 | (output[4] & 0x07FF) << 4);
  packet[7] = (uint8_t) ((output[4] & 0x07FF) >> 4 | (output[5] & 0x07FF) << 7);
  packet[8] = (uint8_t) ((output[5] & 0x07FF) >> 1);
  packet[9] = (uint8_t) ((output[5] & 0x07FF) >> 9 | (output[6] & 0x07FF) << 2);
  packet[10] = (uint8_t) ((output[6] & 0x07FF) >> 6 | (output[7] & 0x07FF) << 5);
  packet[11] = (uint8_t) ((output[7] & 0x07FF) >> 3);
  packet[12] = (uint8_t) ((output[8] & 0x07FF));
  packet[13] = (uint8_t) ((output[8] & 0x07FF) >> 8 | (output[9] & 0x07FF) << 3);
  packet[14] = (uint8_t) ((output[9] & 0x07FF) >> 5 | (output[10] & 0x07FF) << 6);
  packet[15] = (uint8_t) ((output[10] & 0x07FF) >> 2);
  packet[16] = (uint8_t) ((output[10] & 0x07FF) >> 10 | (output[11] & 0x07FF) << 1);
  packet[17] = (uint8_t) ((output[11] & 0x07FF) >> 7 | (output[12] & 0x07FF) << 4);
  packet[18] = (uint8_t) ((output[12] & 0x07FF) >> 4 | (output[13] & 0x07FF) << 7);
  packet[19] = (uint8_t) ((output[13] & 0x07FF) >> 1);
  packet[20] = (uint8_t) ((output[13] & 0x07FF) >> 9 | (output[14] & 0x07FF) << 2);
  packet[21] = (uint8_t) ((output[14] & 0x07FF) >> 6 | (output[15] & 0x07FF) << 5);
  packet[22] = (uint8_t) ((output[15] & 0x07FF) >> 3);

  packet[23] = stateByte; //Flags byte
  packet[24] = SBUS_FRAME_FOOTER; //Footer
}
