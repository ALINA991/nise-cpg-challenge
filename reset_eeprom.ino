#include <DynamixelSDK.h>

#define ADDR_AX_ID           3                 

#define ADDR_AX_TORQUE_ENABLE           24                 // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_PRESENT_POSITION        36
#define MOVING                          46

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1


#define BAUDRATE                        1000000
#define DEVICENAME                      "1"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
                                                            //DEVICENAME "2" -> Serial2
                                                            //DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      1000                 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

// Create PortHandler instance
dynamixel::PortHandler *portHandler;

// Create PacketHandler instance
dynamixel::PacketHandler *packetHandler;

//***********Set Global Variables****************
int goalPosition = 0;
int isMoving = 0;
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
uint8_t dxl_error = 0;                          // Dynamixel error
int16_t dxl_present_position = 0;               // Present position

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  
  // Initialize portHandler. Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize packetHandler. Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler= dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Change ID
  for(int id=0;id<=253;id++)
  {
  packetHandler->write1ByteTxRx(portHandler, id, 3, DXL_ID, &dxl_error);delay(1);
  packetHandler->write1ByteTxRx(portHandler, id, 4, 1, &dxl_error);delay(1);
  packetHandler->write1ByteTxRx(portHandler, id, 5, 250, &dxl_error);delay(1);
  packetHandler->write2ByteTxRx(portHandler, id, 6, 0, &dxl_error);delay(1);
  packetHandler->write2ByteTxRx(portHandler, id, 8, 1023, &dxl_error);delay(1);
  packetHandler->write1ByteTxRx(portHandler, id, 11, 70, &dxl_error);delay(1);
  packetHandler->write1ByteTxRx(portHandler, id, 12, 60, &dxl_error);delay(1);
  packetHandler->write1ByteTxRx(portHandler, id, 13, 140, &dxl_error);delay(1);
  packetHandler->write2ByteTxRx(portHandler, id, 14, 1023, &dxl_error);delay(1);
  packetHandler->write1ByteTxRx(portHandler, id, 16, 2, &dxl_error);delay(1);
  packetHandler->write1ByteTxRx(portHandler, id, 17, 36, &dxl_error);delay(1);
  packetHandler->write1ByteTxRx(portHandler, id, 18, 36, &dxl_error);delay(1);
  Serial.print("ID : ");
  Serial.println(id);
  delay(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  packetHandler->read1ByteTxRx(portHandler, DXL_ID, MOVING, (uint8_t*)&isMoving, &dxl_error);

  if( 1 ){ //if Dynamixel is stopped
    //Send instruction packet to move for goalPosition
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_GOAL_POSITION, goalPosition, &dxl_error);
    //toggle the position if goalPosition is 1000, set to 0, if 0, set to 1000
    Serial.println("changing ps");
    if(goalPosition == 700)
      goalPosition = 0;
    else
      goalPosition = 700;
  }
  
  packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);

  
  Serial.print("ID : ");
  Serial.print(DXL_ID);
  Serial.print("\t Present Position : ");
  Serial.print(dxl_present_position);
  Serial.print("\n");
}
