/*
 * read_write.cpp
 *
 *  Created on: 2016. 2. 21.
 *      Author: leon
 */

//
// *********     Read and Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
// To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below "#define"d variables yourself.
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000 [1M])
//

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <dynamixel_sdk/dynamixel_sdk.h>
//#include "dynamixel_sdk.h"                                  // Uses DYNAMIXEL SDK library

// Control table address
#define ADDR_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define ADDR_HOMING_OFFSET          20

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        3000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      2024              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     40                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main(int argc, char *argv[])
{
  if(argc < 2) {
    printf("Did not specify ID of Dynamixel to be homed. Please run the command like this: ./home_nozzle 101 \n");
    return 0;
  }

  int DXL_ID_ = std::atoi(argv[1]);
  if (DXL_ID_ < 1) {
    printf("Did not specify ID of Dynamixel to be homed. Please run the command like this: ./home_nozzle 101 \n");
    return 0;
  }
  
  #define DXL_ID                          DXL_ID_                 // Dynamixel ID: 1
  
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position
  int32_t dxl_homing_offset = 0;
  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s \n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s \n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("[ID:%03d] Dynamixel has been successfully connected! \n", DXL_ID);
  }

  printf("\nPlease turn the DXL U2D2 board off and back on, and then \n");

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!) \n\n");
    if (getch() == ESC_ASCII_VALUE)
      break;
    
    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s", packetHandler->getRxPacketError(dxl_error));
    }

    // Read Homing offset
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_HOMING_OFFSET, (uint32_t*)&dxl_homing_offset, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s  \n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s \n", packetHandler->getRxPacketError(dxl_error));
    }
    printf("[ID:%03d] Homing Offset was at: %03d \n", DXL_ID, dxl_homing_offset);

    // Set Homing Offset to 0
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_HOMING_OFFSET, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s \n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s \n", packetHandler->getRxPacketError(dxl_error));
    }

    // Read Homing offset again
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_HOMING_OFFSET, (uint32_t*)&dxl_homing_offset, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s  \n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s \n", packetHandler->getRxPacketError(dxl_error));
    }
    printf("[ID:%03d] Homing Offset set to: %03d \n", DXL_ID, dxl_homing_offset);


    // Read present actual postition
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s \n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s \n", packetHandler->getRxPacketError(dxl_error));
    }

    printf("[ID:%03d] Present Absolute Position: %03d \n", DXL_ID, dxl_present_position);
    int dxl_actual_position = dxl_present_position;

    // Set Homing Offset to -dxl_present_position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_HOMING_OFFSET, - dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s \n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s \n", packetHandler->getRxPacketError(dxl_error));
    }

    // Read Homing offset
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_HOMING_OFFSET, (uint32_t*)&dxl_homing_offset, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s  \n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s \n", packetHandler->getRxPacketError(dxl_error));
    }

    printf("[ID:%03d] Set Homing Offset to %03d \n", DXL_ID, dxl_homing_offset);


    // Read present postition
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s \n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        printf("%s \n", packetHandler->getRxPacketError(dxl_error));
      }

      printf("[ID:%03d] Present Position:%03d \n", DXL_ID, dxl_present_position);



    //SHOW HOMING QUADRANT!
    printf("Press any key to see the homing Quadrant! (or press ESC to quit!) \n\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s \n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s \n", packetHandler->getRxPacketError(dxl_error));
    }

    printf("Showing homing quadrant \n");

    // Read present postition
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s \n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s \n", packetHandler->getRxPacketError(dxl_error));
    }

    printf("[ID:%03d] Present Position:%03d \n", DXL_ID, dxl_present_position);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, -dxl_actual_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s", packetHandler->getRxPacketError(dxl_error));
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

        // Read present postition
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s \n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s \n", packetHandler->getRxPacketError(dxl_error));
    }

    printf("[ID:%03d] Present Position:%03d \n", DXL_ID, dxl_present_position);
    std::this_thread::sleep_for(std::chrono::seconds(1));

      // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 4048-dxl_actual_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s", packetHandler->getRxPacketError(dxl_error));
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

        // Read present postition
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s \n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s \n", packetHandler->getRxPacketError(dxl_error));
    }

    printf("[ID:%03d] Present Position:%03d \n", DXL_ID, dxl_present_position);
    std::this_thread::sleep_for(std::chrono::seconds(1));

 // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s", packetHandler->getRxPacketError(dxl_error));
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

        // Read present postition
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s \n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s \n", packetHandler->getRxPacketError(dxl_error));
    }

    printf("[ID:%03d] Present Position:%03d \n", DXL_ID, dxl_present_position);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    printf("\nHOMING COMPLETED! \n");
    
    break;
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  printf("Bye Bye :) \n");

  return 0;
}
