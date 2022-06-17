// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/BulkGetItem.h"
#include "dynamixel_sdk_examples/BulkSetItem.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_PWM 124
#define ADDR_GOAL_POSITION    116
#define ADDR_GOAL_PWM    100

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler * packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupBulkRead groupBulkRead(portHandler, packetHandler);
GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

bool get_data_callback(
  dynamixel_sdk_examples::BulkGetItem::Request & req,
  dynamixel_sdk_examples::BulkGetItem::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = 0;
  int address_data; 

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  if (req.data_required == "position") {
    address_data = ADDR_PRESENT_POSITION;
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor1_id, address_data, 4);
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor2_id, address_data, 4);
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor3_id, address_data, 4);
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor4_id, address_data, 4);
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor5_id, address_data, 4);
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor6_id, address_data, 4);

    if (dxl_addparam_result != 6) {
      ROS_ERROR("Failed to addparam to groupBulkRead for Dynamixel");
      return 0;
    }

    uint32_t value1 = 0;
    uint32_t value2 = 0;
    dxl_comm_result = groupBulkRead.txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS) {
      res.motor1_data = groupBulkRead.getData((uint8_t)req.motor1_id, address_data, 4);
      res.motor2_data = groupBulkRead.getData((uint8_t)req.motor2_id, address_data, 4);
      res.motor3_data = groupBulkRead.getData((uint8_t)req.motor3_id, address_data, 4);
      res.motor4_data = groupBulkRead.getData((uint8_t)req.motor4_id, address_data, 4);
      res.motor5_data = groupBulkRead.getData((uint8_t)req.motor5_id, address_data, 4);
      res.motor6_data = groupBulkRead.getData((uint8_t)req.motor6_id, address_data, 4);

    groupBulkRead.clearParam();
    return true;
  } else {
    ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
    groupBulkRead.clearParam();
    return false;
  }
  }
  else if (req.data_required == "PWM") {
    address_data = ADDR_PRESENT_PWM;
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor1_id, address_data, 2);
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor2_id, address_data, 2);
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor3_id, address_data, 2);
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor4_id, address_data, 2);
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor5_id, address_data, 2);
    dxl_addparam_result += groupBulkRead.addParam((uint8_t)req.motor6_id, address_data, 2);

    if (dxl_addparam_result != 6) {
      ROS_ERROR("Failed to addparam to groupBulkRead for Dynamixel");
      return 0;
    }

    uint32_t value1 = 0;
    uint32_t value2 = 0;
    dxl_comm_result = groupBulkRead.txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS) {
      res.motor1_data = groupBulkRead.getData((uint8_t)req.motor1_id, address_data, 2);
      res.motor2_data = groupBulkRead.getData((uint8_t)req.motor2_id, address_data, 2);
      res.motor3_data = groupBulkRead.getData((uint8_t)req.motor3_id, address_data, 2);
      res.motor4_data = groupBulkRead.getData((uint8_t)req.motor4_id, address_data, 2);
      res.motor5_data = groupBulkRead.getData((uint8_t)req.motor5_id, address_data, 2);
      res.motor6_data = groupBulkRead.getData((uint8_t)req.motor6_id, address_data, 2);

      groupBulkRead.clearParam();
      return true;
    } else {
      ROS_ERROR("Failed to get pwm! Result: %d", dxl_comm_result);
      groupBulkRead.clearParam();
      return false;
    }
  }

}

void set_data_callback(const dynamixel_sdk_examples::BulkSetItem::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = 0;
  uint8_t param_goal_position[6][4];
  uint8_t param_goal_pwm[6][2];
  uint8_t addr_goal_item[6];
  uint8_t len_goal_item[6];
  uint32_t position;
  uint32_t pwm_goal;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  if (msg->data_required == "position") {
    position = (unsigned int)msg->motor1_data; // Convert int32 -> uint32
    param_goal_position[0][0] = DXL_LOBYTE(DXL_LOWORD(position));
    param_goal_position[0][1] = DXL_HIBYTE(DXL_LOWORD(position));
    param_goal_position[0][2] = DXL_LOBYTE(DXL_HIWORD(position));
    param_goal_position[0][3] = DXL_HIBYTE(DXL_HIWORD(position));
    addr_goal_item[0] = ADDR_GOAL_POSITION;
    len_goal_item[0] = 4;

    position = (unsigned int)msg->motor2_data; // Convert int32 -> uint32
    param_goal_position[1][0] = DXL_LOBYTE(DXL_LOWORD(position));
    param_goal_position[1][1] = DXL_HIBYTE(DXL_LOWORD(position));
    param_goal_position[1][2] = DXL_LOBYTE(DXL_HIWORD(position));
    param_goal_position[1][3] = DXL_HIBYTE(DXL_HIWORD(position));
    addr_goal_item[1] = ADDR_GOAL_POSITION;
    len_goal_item[1] = 4;

    position = (unsigned int)msg->motor3_data; // Convert int32 -> uint32
    param_goal_position[2][0] = DXL_LOBYTE(DXL_LOWORD(position));
    param_goal_position[2][1] = DXL_HIBYTE(DXL_LOWORD(position));
    param_goal_position[2][2] = DXL_LOBYTE(DXL_HIWORD(position));
    param_goal_position[2][3] = DXL_HIBYTE(DXL_HIWORD(position));
    addr_goal_item[2] = ADDR_GOAL_POSITION;
    len_goal_item[2] = 4;

    position = (unsigned int)msg->motor4_data; // Convert int32 -> uint32
    param_goal_position[3][0] = DXL_LOBYTE(DXL_LOWORD(position));
    param_goal_position[3][1] = DXL_HIBYTE(DXL_LOWORD(position));
    param_goal_position[3][2] = DXL_LOBYTE(DXL_HIWORD(position));
    param_goal_position[3][3] = DXL_HIBYTE(DXL_HIWORD(position));
    addr_goal_item[3] = ADDR_GOAL_POSITION;
    len_goal_item[3] = 4;

    position = (unsigned int)msg->motor5_data; // Convert int32 -> uint32
    param_goal_position[4][0] = DXL_LOBYTE(DXL_LOWORD(position));
    param_goal_position[4][1] = DXL_HIBYTE(DXL_LOWORD(position));
    param_goal_position[4][2] = DXL_LOBYTE(DXL_HIWORD(position));
    param_goal_position[4][3] = DXL_HIBYTE(DXL_HIWORD(position));
    addr_goal_item[4] = ADDR_GOAL_POSITION;
    len_goal_item[4] = 4;
  
    position = (unsigned int)msg->motor5_data; // Convert int32 -> uint32
    param_goal_position[5][0] = DXL_LOBYTE(DXL_LOWORD(position));
    param_goal_position[5][1] = DXL_HIBYTE(DXL_LOWORD(position));
    param_goal_position[5][2] = DXL_LOBYTE(DXL_HIWORD(position));
    param_goal_position[5][3] = DXL_HIBYTE(DXL_HIWORD(position));
    addr_goal_item[5] = ADDR_GOAL_POSITION;
    len_goal_item[5] = 4;

  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor1_id, addr_goal_item[0], len_goal_item[0], param_goal_position[0]);
  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor2_id, addr_goal_item[1], len_goal_item[1], param_goal_position[1]);
  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor3_id, addr_goal_item[2], len_goal_item[2], param_goal_position[2]);
  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor4_id, addr_goal_item[3], len_goal_item[3], param_goal_position[3]);
  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor5_id, addr_goal_item[4], len_goal_item[4], param_goal_position[4]);
  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor6_id, addr_goal_item[5], len_goal_item[5], param_goal_position[5]);

  }
  else if (msg->data_required == "PWM") {
    pwm_goal = (unsigned int)msg->motor1_data; // Convert int32 -> uint32
    param_goal_pwm[0][0] = DXL_LOBYTE(DXL_LOWORD(pwm_goal));
    param_goal_pwm[0][1] = DXL_HIBYTE(DXL_LOWORD(pwm_goal));
    addr_goal_item[0] = ADDR_GOAL_PWM;
    len_goal_item[0] = 2;

    position = (unsigned int)msg->motor2_data; // Convert int32 -> uint32
    param_goal_pwm[1][0] = DXL_LOBYTE(DXL_LOWORD(pwm_goal));
    param_goal_pwm[1][1] = DXL_HIBYTE(DXL_LOWORD(pwm_goal));
    addr_goal_item[1] = ADDR_GOAL_PWM;
    len_goal_item[1] = 2;

    position = (unsigned int)msg->motor3_data; // Convert int32 -> uint32
    param_goal_pwm[2][0] = DXL_LOBYTE(DXL_LOWORD(pwm_goal));
    param_goal_pwm[2][1] = DXL_HIBYTE(DXL_LOWORD(pwm_goal));
    addr_goal_item[2] = ADDR_GOAL_PWM;
    len_goal_item[2] = 2;

    position = (unsigned int)msg->motor4_data; // Convert int32 -> uint32
    param_goal_pwm[3][0] = DXL_LOBYTE(DXL_LOWORD(pwm_goal));
    param_goal_pwm[3][1] = DXL_HIBYTE(DXL_LOWORD(pwm_goal));
    addr_goal_item[3] = ADDR_GOAL_PWM;
    len_goal_item[3] = 2;

    position = (unsigned int)msg->motor5_data; // Convert int32 -> uint32
    param_goal_pwm[4][0] = DXL_LOBYTE(DXL_LOWORD(pwm_goal));
    param_goal_pwm[4][1] = DXL_HIBYTE(DXL_LOWORD(pwm_goal));
    addr_goal_item[4] = ADDR_GOAL_PWM;
    len_goal_item[4] = 2;
  
    position = (unsigned int)msg->motor5_data; // Convert int32 -> uint32
    param_goal_pwm[5][0] = DXL_LOBYTE(DXL_LOWORD(pwm_goal));
    param_goal_pwm[5][1] = DXL_HIBYTE(DXL_LOWORD(pwm_goal));
    addr_goal_item[5] = ADDR_GOAL_PWM;
    len_goal_item[5] = 2;

  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor1_id, addr_goal_item[0], len_goal_item[0], param_goal_pwm[0]);
  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor2_id, addr_goal_item[1], len_goal_item[1], param_goal_pwm[1]);
  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor3_id, addr_goal_item[2], len_goal_item[2], param_goal_pwm[2]);
  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor4_id, addr_goal_item[3], len_goal_item[3], param_goal_pwm[3]);
  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor5_id, addr_goal_item[4], len_goal_item[4], param_goal_pwm[4]);
  dxl_addparam_result += groupBulkWrite.addParam((uint8_t)msg->motor6_id, addr_goal_item[5], len_goal_item[5], param_goal_pwm[5]);

  }

  if (dxl_addparam_result != 6) {
    ROS_ERROR("Failed to addparam to groupBulkWrite for Dynamixel");
  }

  dxl_comm_result = groupBulkWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("set data SUCCESS");
  } else {
    ROS_INFO("Failed to set data! Result: %d", dxl_comm_result);
  }

  groupBulkWrite.clearParam();
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }
  

  ros::init(argc, argv, "read_write_dinamixel_motors");
  ros::NodeHandle nh;
  ros::ServiceServer bulk_get_item_srv = nh.advertiseService("/get_dinamixel_motor_group_data", get_data_callback);
  ros::Subscriber bulk_set_item_sub = nh.subscribe("/set_dinamixel_motor_group_data", 10, set_data_callback);
  ROS_INFO("Initialized correctly the service and topics --groupmotors-- " );
  ros::spin();

  portHandler->closePort();
  return 0;
}
