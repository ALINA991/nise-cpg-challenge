#include <DynamixelSDK.h>


#define analogInPin0  A0
#define analogInPin2  A2
#define analogInPin4  A4
#define analogInPin6  A6


#define ADDR_AX_ID           3                 

#define ADDR_AX_TORQUE_ENABLE           24                 // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_PRESENT_POSITION        36
#define MOVING                          46
#define CHANGE_SPEED                    32
#define CHANGE_TORQUE                   34
#define READ_SPEED                      38

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel


// Default setting
#define MOTOR_ID1                       1                 // Dynamixel ID: 1
#define MOTOR_ID2                       2                 // Dynamixel ID: 2
#define MOTOR_ID3                       3                 // Dynamixel ID: 3
#define MOTOR_ID4                       4                 // Dynamixel ID: 4
#define MOTOR_ID5                       5                 // Dynamixel ID: 5
#define MOTOR_ID6                       6                 // Dynamixel ID: 6
#define MOTOR_ID7                       7                 // Dynamixel ID: 7


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
int goalPosition1 = 0;
int goalPosition2 = 0;
int isMoving1 = 0;
int isMoving2 = 0;
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
uint8_t dxl_error = 0;                          // Dynamixel error
int16_t dxl_present_position = 0;
int16_t dx2_present_position = 0; // Present position

int16_t dx1_change_speed = 1023;
int16_t dxl_change_torque = 1023;
int16_t dx1_read_change_torque = 0;
int16_t dxl_read_change_speed =0;

//Declare sensors

double sensorValue0 = 0 ;
double sensorValue2 = 0 ;
double sensorValue4 = 0 ;
double sensorValue6 = 0 ;
/*=================RS neuron======================*/
#define DISCRIPTION_LENGTH     15
#define NOMBER_RS_NEURONS     7
unsigned long int myTime;
unsigned int mydelay = 10; // ms

struct RSneuron { 
   char discription[DISCRIPTION_LENGTH]; // name 
   double tao_m   = 0.1;  
   double tao_s   = 20*tao_m; 
   double Af      = 5; 
   double Es      = 0; 
   double sigma_s = 0; 
   double sigma_f = 0; 
   double V_0     = 0;
   double q_0     = 0;
   double V       = V_0;
   double q       = q_0; 
   double inj_cur = 0; 
   double inj_cur_MultiplicationFactor = 1; 
   
} rs_neuron[NOMBER_RS_NEURONS]; 
/******************************************************/ 
//struct Pattern 
/******************************************************/ 
struct Pattern{
  double sigma_s;
  double sigma_f;
  double tao_m;
  double InjCurrentMultiplicationFactor;
  };
/******************************************************/
/*
  OSCILLATORY
   ^
   |     
   |   /\    /\    /\
   |  /  \  /  \  /  \
   | /    \/    \/    \
   ------------------------> 
*/
Pattern OSCILLATORY1={4.6,1.5,0.05,1};
Pattern OSCILLATORY2={6.6,2.5,0.1,5};

/******************************************************/ 
inline double fun_v ( double V , double q , double Sf , double inj , double tm , double Af )
{ return  (double)(-1/tm)*( V-Af*tanh(Sf/Af*V)+q-inj) ; } 
/******************************************************/ 
inline double  fun_q ( double V , double q , double ts , double Es , double Ss )
{ return (double)(1/ts)*(-q+Ss*(V-Es)); } 
/******************************************************/ 
void update_RS_neuron(struct RSneuron* rs_n)
{
int n = 20; 
//double x0 = t-ts;
//double xf = t;
double h; 
//h = (xf-x0)/(double)n;
h= ((double)mydelay/1000)/n;
double xa[20],ya[20],yb[20]; 
double k1,k2,k3,k4,k, l1,l2,l3,l4,l;  
//xa[0] = x0;
ya[0] = rs_n->V;
yb[0] = rs_n->q;

for (int i=0; i<n-1 ; i++)
{
  k1= h*fun_v( ya[i] , yb[i] , rs_n->sigma_f , rs_n->inj_cur*rs_n->inj_cur_MultiplicationFactor ,  rs_n->tao_m , rs_n->Af ); 
  l1 = h*fun_q( ya[i] , yb[i] , rs_n->tao_s , rs_n->Es , rs_n->sigma_s ); 

  k2= h*fun_v( ya[i]+k1/2 , yb[i]+l1/2 , rs_n->sigma_f , rs_n->inj_cur*rs_n->inj_cur_MultiplicationFactor , rs_n->tao_m , rs_n->Af ); 
  l2 = h*fun_q( ya[i]+k1/2 , yb[i]+l1/2 , rs_n->tao_s , rs_n->Es , rs_n->sigma_s);   
  
  k3= h*fun_v( ya[i]+k2/2 , yb[i]+l2/2 , rs_n->sigma_f , rs_n->inj_cur*rs_n->inj_cur_MultiplicationFactor , rs_n->tao_m , rs_n->Af ); 
  l3 = h*fun_q( ya[i]+k2/2 , yb[i]+l2/2 , rs_n->tao_s , rs_n->Es , rs_n->sigma_s );   

  k4= h*fun_v( ya[i]+k3 , yb[i]+l3 , rs_n->sigma_f , rs_n->inj_cur*rs_n->inj_cur_MultiplicationFactor , rs_n->tao_m , rs_n->Af ); 
  l4 = h*fun_q( ya[i]+k3 , yb[i]+l3 , rs_n->tao_s , rs_n->Es , rs_n->sigma_s );   
  
  k = 1/6.0 * (k1 + 2*k2 + 2*k3 + k4);
  l  = 1/6.0 * ( l1 + 2*l2  + 2*l3  +  l4);
 
  ya[i+1]= ya[i]+k;
  yb[i+1]= yb[i]+l ;
  //xa[i+1]= xa[i]+h;
} 
rs_n->V = ya[n-1]; 
rs_n->q = yb[n-1]; 

return; 
}
/******************************************************/ 
void setup_RS_neurons(struct RSneuron* rs_n,struct Pattern myP)
{
rs_n->tao_m = myP.tao_m; 
rs_n->tao_s = 20 * myP.tao_m; 
rs_n->sigma_s = myP.sigma_s;
rs_n->sigma_f = myP.sigma_f;
rs_n->inj_cur_MultiplicationFactor = myP.InjCurrentMultiplicationFactor;

}
/******************************************************/ 
void update_locomotion_network(void)
{
for (int i = 0; i< NOMBER_RS_NEURONS ; i++)
  update_RS_neuron(&rs_neuron[i]);
}
/******************************************************/ 
/* put your setup code in setup(), to run once */

unsigned long int mystartTime;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(analogInPin0,INPUT);
  pinMode(analogInPin2,INPUT);
  pinMode(analogInPin4,INPUT);
  pinMode(analogInPin6,INPUT);

  setup_RS_neurons(&rs_neuron[0],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[1],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[2],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[3],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[4],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[5],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[6],OSCILLATORY1);


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
  mystartTime = millis();
  
}

void loop() {

  //sensorValue1 = analogRead(analogInPin1);
  sensorValue0 = analogRead(analogInPin0);
  sensorValue2 = analogRead(analogInPin2);
  sensorValue4 = analogRead(analogInPin4);
  sensorValue6 = analogRead(analogInPin6);
  // sensorValue10 = analogRead(analogInPin10);
  // sensorValue11 = analogRead(analogInPin11);
  // sensorValue12 = analogRead(analogInPin12);
  // sensorValue13 = analogRead(analogInPin13);
  // sensorValue14 = analogRead(analogInPin14);
  // //sensorValue15 = analogRead(analogInPin15);

  Serial.print(sensorValue0);
  Serial.print("\t");
  Serial.print(sensorValue2);
  Serial.print("\t");
  Serial.print(sensorValue4);
  Serial.print("\t");
  Serial.print(sensorValue6);
  Serial.print("\n");

  myTime = millis()- mystartTime;
  double offset_time;
  offset_time = 5000;
  rs_neuron[0].inj_cur = 0;
  rs_neuron[1].inj_cur = 0;
  rs_neuron[2].inj_cur = 0;
  rs_neuron[3].inj_cur = 0;
  rs_neuron[4].inj_cur = 0;
  rs_neuron[5].inj_cur = 0;
  rs_neuron[6].inj_cur = 0;

  for (int i = 0; i< NOMBER_RS_NEURONS ; i++)
  if((myTime>(5000 + 1050*i))&&(myTime<(5100 + 1050*i))){
  rs_neuron[i].inj_cur = 1;
  }

  update_locomotion_network();

  double output1 = rs_neuron[0].V;
  double output2 = rs_neuron[1].V;
  double output3 = rs_neuron[2].V;
  double output4 = rs_neuron[3].V;
  double output5 = rs_neuron[4].V;
  double output6 = rs_neuron[5].V;
  double output7 = rs_neuron[6].V;
  
  int input1 = int(map(output1,-4.6,4.6,252,772));
  int input2 = int(map(output2,-4.6,4.6,252,772));
  int input3 = int(map(output3,-4.6,4.6,252,772));
  int input4 = int(map(output4,-4.6,4.6,252,772));
  int input5 = int(map(-output5,-4.6,4.6,252,772));
  int input6 = int(map(output6,-4.6,4.6,252,772));
  int input7 = int(map(-output7,-4.6,4.6,252,772));
  // put your main code here, to run repeatedly:
//  packetHandler->read1ByteTxRx(portHandler, MOTOR_ID1, MOVING, (uint8_t*)&isMoving1, &dxl_error);
//  packetHandler->read1ByteTxRx(portHandler, MOTOR_ID2, MOVING, (uint8_t*)&isMoving2, &dxl_error);

  
//  if( isMoving1 == 0 ){ //if Dynamixel is stopped
    //Send instruction packet to move for goalPosition
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID1, ADDR_AX_GOAL_POSITION, input1, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID3, ADDR_AX_GOAL_POSITION, input3, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID5, ADDR_AX_GOAL_POSITION, input5, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID7, ADDR_AX_GOAL_POSITION, input7, &dxl_error);

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID2, ADDR_AX_GOAL_POSITION, input2, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID4, ADDR_AX_GOAL_POSITION, input4, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID6, ADDR_AX_GOAL_POSITION, input6, &dxl_error);
    //toggle the position if goalPosition is 1000, set to 0, if 0, set to 1000
//    if(goalPosition2 == 1023)
//      goalPosition2 = 0;
//    else
//      goalPosition2 = 1023;
//  }
  
  packetHandler->read2ByteTxRx(portHandler, MOTOR_ID1, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);
  packetHandler->read2ByteTxRx(portHandler, MOTOR_ID2, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dx2_present_position, &dxl_error);
  
//  packetHandler->write2ByteTxRx(portHandler, MOTOR_ID1, CHANGE_SPEED, dx1_change_speed, &dxl_error);
//  packetHandler->read2ByteTxRx(portHandler, MOTOR_ID1, READ_SPEED, (uint16_t*)&dxl_read_change_speed, &dxl_error);

//   Serial.print(input1);
//   Serial.print("\t");
//   Serial.print(input2);
//   Serial.print("\t");
//   Serial.print(input3);
//   Serial.print("\t");
//   Serial.print(input4);
//   Serial.print("\t");
//   Serial.print(input5);
//   Serial.print("\t");
//   Serial.print(input6);  
//   Serial.print("\t");
//   Serial.print(input7);
//   Serial.print("\n");
// //  delay(100);
}
