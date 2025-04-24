// Copyright 2020 ROBOTIS CO., LTD.
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

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
 * $ rosservice call /get_position "id: 1"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 1000}"
 * $ rosservice call /get_position "id: 2"
 *
 * Author: Zerom
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_BODY_L_R         1               // 몸통 좌우 모터 ID
#define DXL2_HEAD_L_R         2               // 머리 좌우 모터 ID
#define DXL3_HEAD_UPDOWN      3               // 머리 상하 모터 ID
#define BAUDRATE              1000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/robotis"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler;
PacketHandler * packetHandler;

// 전역 변수로 dxl3_home_position과 dxl3_limit_position 선언
int32_t dxl1_L_limit = 2320;
int32_t dxl1_R_limit = 1776;
int32_t dxl2_L_limit = 4095;
int32_t dxl2_R_limit = 0;
int32_t dxl3_home_position = 0;
int32_t dxl3_limit_position = 0;

bool getPresentPositionCallback(
  dynamixel_sdk_examples::GetPosition::Request & req,
  dynamixel_sdk_examples::GetPosition::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req.id, position);
    res.position = position;
    return true;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return false;
  }
}

void setPositionCallback(const dynamixel_sdk_examples::SetPosition::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position = (unsigned int)msg->position; // Convert int32 -> uint32

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)msg->id, ADDR_GOAL_POSITION, position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id, msg->position);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }
}

bool checkDeviceExistence(uint8_t id)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  uint32_t position = 0;

  // 현재 위치를 읽어보면서 장치 존재 여부 확인
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, id, ADDR_PRESENT_POSITION, &position, &dxl_error);

  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("Device ID:%d exists", id);
    return true;
  } else {
    ROS_ERROR("Device ID:%d does not exist. Error code: %d", id, dxl_comm_result);
    return false;
  }
}

bool initializeMotor3()
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  uint32_t prev_position = 0;
  uint32_t current_position = 0;
  bool position_stable = false;
  int stable_count = 0;
  
  // 모터를 비활성화 한다
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_HEAD_UPDOWN, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", DXL3_HEAD_UPDOWN);
    return false;
  }
  
  // 전류제어 모드로 변경한다 (운영 모드 주소: 11, 전류제어 모드: 0)
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_HEAD_UPDOWN, 11, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to change operating mode for Dynamixel ID %d", DXL3_HEAD_UPDOWN);
    return false;
  }
  
  // 모터를 활성화 한다
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_HEAD_UPDOWN, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL3_HEAD_UPDOWN);
    return false;
  }
  
  // 전류제어 모드로 + 방향으로 10 만큼 부여한다 (전류 목표값 주소: 102)
  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, DXL3_HEAD_UPDOWN, 102, 222, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to set current for Dynamixel ID %d", DXL3_HEAD_UPDOWN);
    return false;
  }
  
  ros::Time start_time = ros::Time::now();
  ros::Rate rate(10); // 10Hz로 위치 확인
  
  // 위치를 주기적으로 읽는다
  while (ros::ok() && !position_stable) {
    // 이전 위치 저장
    prev_position = current_position;
    
    // 현재 위치 읽기
    dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, DXL3_HEAD_UPDOWN, ADDR_PRESENT_POSITION, &current_position, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR("Failed to read position for Dynamixel ID %d", DXL3_HEAD_UPDOWN);
      return false;
    }
    
    // 위치 변화 확인
    if (abs((int32_t)current_position - (int32_t)prev_position) < 5) {
      stable_count++;
    } else {
      stable_count = 0;
    }
    
    // 2초간 위치가 변하지 않으면 (10Hz에서 20회)
    if (stable_count >= 20) {
      position_stable = true;
      dxl3_home_position = current_position - 20; // 20 마진
      ROS_INFO("Home position found at: %d", dxl3_home_position);
    }
    
    rate.sleep();
  }
  
  // 모터를 비활성화 한다
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_HEAD_UPDOWN, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", DXL3_HEAD_UPDOWN);
    return false;
  }
  
  // 동작 모드를 확장위치제어모드로 변경한다 (확장위치제어모드: 4)
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_HEAD_UPDOWN, 11, 4, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to change operating mode for Dynamixel ID %d", DXL3_HEAD_UPDOWN);
    return false;
  }
  
  // 최소 위치 한계값 설정 (dxl3_home_position - 7300)
  dxl3_limit_position = (int32_t)dxl3_home_position - 7300;
  
  
  
  // 모터를 활성화 한다
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_HEAD_UPDOWN, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL3_HEAD_UPDOWN);
    return false;
  }
  
  ROS_INFO("Motor 3 initialized successfully. Home position: %d, Min position: %d", 
           dxl3_home_position, dxl3_limit_position);
  return true;
}

int dxl_v3_up(){
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, DXL3_HEAD_UPDOWN, ADDR_GOAL_POSITION, dxl3_limit_position, &dxl_error);
    
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to set position for Dynamixel ID %d", DXL3_HEAD_UPDOWN);
    return false;
  }

  return true;
}

int dxl_v3_down(){
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, DXL3_HEAD_UPDOWN, ADDR_GOAL_POSITION, dxl3_home_position, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to set position for Dynamixel ID %d", DXL3_HEAD_UPDOWN);
    return false;
  }

  return true;
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  bool devices_exist = true;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  // 3개 장치 존재 여부 확인
  ROS_INFO("Checking Dynamixel motor devices...");
  if (!checkDeviceExistence(DXL1_BODY_L_R)) {
    devices_exist = false;
  }
  if (!checkDeviceExistence(DXL2_HEAD_L_R)) {
    devices_exist = false;
  }
  if (!checkDeviceExistence(DXL3_HEAD_UPDOWN)) {
    devices_exist = false;
  }

  if (!devices_exist) {
    ROS_ERROR("Some Dynamixel devices are not connected. Please check the connection.");
    portHandler->closePort();
    return -1;
  }

  ros::init(argc, argv, "read_write_node");
  ros::NodeHandle nh;
  
  // ros::NodeHandle 생성 이후로 initializeMotor3() 호출 위치 이동
  if (!initializeMotor3()) {
    ROS_ERROR("Failed to initialize motor 3");
    return -1;
  }


  dxl_v3_up();

  // 토크 활성화
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_BODY_L_R, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_BODY_L_R);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_HEAD_L_R, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_HEAD_L_R);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_HEAD_UPDOWN, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL3_HEAD_UPDOWN);
    return -1;
  }

  ros::ServiceServer get_position_srv = nh.advertiseService("/get_position", getPresentPositionCallback);
  ros::Subscriber set_position_sub = nh.subscribe("/set_position", 10, setPositionCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}

