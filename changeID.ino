#include <DynamixelSDK.h>

#define ADDR_AX_ID           3                 

#define ADDR_AX_TORQUE_ENABLE           24                 // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_PRESENT_POSITION        36
#define MOVING                          46
//#define CHANGE_SPEED                    

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel


// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define DXL_NEW_ID                      1                   // Dynamixel ID: 2


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

uint8_t dxl_new_id = DXL_NEW_ID; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial.available());
String a;
  if(Serial.available())
  {
    a= Serial.readString();// read the incoming data as string
    Serial.println(a); 
    dxl_new_id = a.toInt();
    delay(2000);
  }

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
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_AX_ID, dxl_new_id, &dxl_error);
  delay(10);
  }
  packetHandler->write2ByteTxRx(portHandler, dxl_new_id, ADDR_AX_GOAL_POSITION, goalPosition, &dxl_error);
}

void loop() {
  // put your main code here, to run repeatedly:
  packetHandler->read1ByteTxRx(portHandler, dxl_new_id, MOVING, (uint8_t*)&isMoving, &dxl_error);

  if( isMoving == 0 ){ //if Dynamixel is stopped
    //Send instruction packet to move for goalPosition
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_new_id, ADDR_AX_GOAL_POSITION, goalPosition, &dxl_error);
    //toggle the position if goalPosition is 1000, set to 0, if 0, set to 1000
    if(goalPosition == 700)
      goalPosition = 0;
    else
      goalPosition = 700;
  }
  
  packetHandler->read2ByteTxRx(portHandler, dxl_new_id, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);

  
  Serial.print("ID : ");
  Serial.print(dxl_new_id);
  Serial.print("\t Present Position : ");
  Serial.print(dxl_present_position);
  Serial.print("\n");
}
