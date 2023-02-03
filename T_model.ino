#include <DynamixelSDK.h>


#define ADDR_AX_ID 3

#define ADDR_AX_TORQUE_ENABLE 24  // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION 30
#define ADDR_AX_PRESENT_POSITION 36
#define MOVING 46
#define CHANGE_SPEED 32
#define CHANGE_TORQUE 34
#define READ_SPEED 38

// Protocol version
#define PROTOCOL_VERSION 1.0  // See which protocol version is used in the Dynamixel


// Default setting
#define MOTOR_ID1 1  // Dynamixel ID: 1
#define MOTOR_ID2 2  // Dynamixel ID: 2
#define MOTOR_ID3 3  // Dynamixel ID: 3
#define MOTOR_ID4 4  // Dynamixel ID: 4
#define MOTOR_ID5 5  // Dynamixel ID: 5
#define MOTOR_ID6 6  // Dynamixel ID: 6
#define MOTOR_ID7 7  // Dynamixel ID: 7

#define analogInPin0 A0
#define analogInPin2 A2
#define analogInPin4 A4
#define analogInPin6 A6


#define BAUDRATE 1000000
#define DEVICENAME "1"                   //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports) \
  //DEVICENAME "2" -> Serial2 \
  //DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE 1                  // Value for enabling the torque
#define TORQUE_DISABLE 0                 // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE 0     // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE 1000  // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD 20   // Dynamixel moving status threshold

#define ESC_ASCII_VALUE 0x1b

// Create PortHandler instance
dynamixel::PortHandler* portHandler;

// Create PacketHandler instance
dynamixel::PacketHandler* packetHandler;

//***********Set Global Variables****************
int goalPosition1 = 0;
int goalPosition2 = 0;
int isMoving1 = 0;
int isMoving2 = 0;
int dxl_comm_result = COMM_TX_FAIL;  // Communication result
uint8_t dxl_error = 0;               // Dynamixel error
int16_t dxl_present_position = 0;
int16_t dx2_present_position = 0;  // Present position

int16_t dx1_change_speed = 1023;
int16_t dxl_change_torque = 1023;
int16_t dx1_read_change_torque = 0;
int16_t dxl_read_change_speed = 0;


int CW_position_1 = 442;
int CCW_position_1 = 597;

int CW_position_2 = 462;
int CCW_position_2 = 622;

//Declare sensors

double sensor_FR = 0;
double sensor_FL = 0;
double sensor_BR = 0;
double sensor_BL = 0;

/*=================Matsuoka neuron======================*/
#define DISCRIPTION_LENGTH 15
#define NOMBER_M_NEURONS 7
unsigned long int myTime;
unsigned long int count_time;
unsigned long int start_count_time;
unsigned int mydelay = 10;  // ms -> this is the interval

/******************************************************/
//struct Matsuoka neuron
struct Mneuron {
  char discription[DISCRIPTION_LENGTH];  // name
  double tao = 0;
  double T = 0;
  double b = 0;
  double x_0 = 0;
  double xh_0 = 0;
  double x = x_0;
  double xh = xh_0;
  double y = 0;
  double Y[NOMBER_M_NEURONS] = { 0, 0, 0, 0, 0, 0, 0 };
  double a[NOMBER_M_NEURONS] = { 0, 0, 0, 0, 0, 0, 0 };
  //  double a_slow[NOMBER_M_NEURONS]= {0.7,1.2,1.2,1.2,1.2,0,0};
  double inj_cur = 0;
}; 

Mneuron m_neuron_fast[NOMBER_M_NEURONS];
Mneuron m_neuron_slow[NOMBER_M_NEURONS];


/******************************************************/
//struct Pattern
struct Pattern {
  double tao;
  double T;
  double b;
  double a[NOMBER_M_NEURONS];
};

Pattern CASE_2_fast = {0.05, 12, 2.5, { 1, 1.2, 1.2, 1.2, 1.2, 0, 0 } };  
// fast a = { 1, 1.2, 1.2, 1.2, 1.2, 0, 0 }  
Pattern CASE_2_slow = {0.5, 12.0, 2.5,{0.7, 1.2, 1.2, 1.2, 1.2, 0.0, 0.0}};////
// slow a = { 0.7, 1.2, 1.2, 1.2, 1.2, 0, 0 }


/******************************************************/
inline double diff_x(double x, double xh, double tao, double b, double a[], double Y[], double inj_cur) {
  double totalinput = 0;
  for (int i = 0; i < NOMBER_M_NEURONS; i++) {
    totalinput = totalinput + a[i] * Y[i];
  }
  return (double)(-1 / tao) * (x + b * xh + totalinput - inj_cur);
}

/******************************************************/
inline double diff_xh(double x, double xh, double T, double y) {
  return (double)(-1 / T) * (xh - y);
}
//  default theta =0; y = max(x, theta) = x

/******************************************************/
inline double y_max(double x) {
  return (x > 0) ? x : 0;
}

/******************************************************/
// C++ program for right rotation of an array (Reversal Algorithm)
// Ref URL: https://www.geeksforgeeks.org/reversal-algorithm-right-rotation-array/
// Example: Input: arr[] = {1, 2, 3, 4}; k = 2 → Output: 3,4,1,2
void reverseArray(double arr[], int start, int end) {
  while (start < end) {
    std::swap(arr[start], arr[end]);
    start++;
    end--;
  }
}
/* Function to right rotate arr[] of size n by d */
void rightRotate(double arr[], int d, int n) {
  // if in case d>n,this will give segmentation fault.
  d = d % n;
  reverseArray(arr, 0, n - 1);
  reverseArray(arr, 0, d - 1);
  reverseArray(arr, d, n - 1);
}

/******************************************************/
// Euler's Method
void update_M_neuron(struct Mneuron* m_n, double Y[]) {
  int n = 20;
  double h;
  //mydelay = Δt[ms] = f(x+Δt)-f(x)
  h = ((double)mydelay / 1000) / n;
  double ya[n], yb[n], y[n];
  double k1, k2, k3, k4, k, l1, l2, l3, l4, l;

  ya[0] = m_n->x;
  yb[0] = m_n->xh;
  y[0] = m_n->y;

  for (int i = 0; i < n - 1; i++) {
    k = h * diff_x(ya[i], yb[i], m_n->tao, m_n->b, m_n->a, Y, m_n->inj_cur);
    l = h * diff_xh(ya[i], yb[i], m_n->T, y[0]);
    ya[i + 1] = ya[i] + k;
    yb[i + 1] = yb[i] + l;
    y[i + 1] = y_max(ya[i]);
  }
  m_n->x = ya[n - 1];
  m_n->xh = yb[n - 1];
  m_n->y = y[n - 1];
  return;
}


/******************************************************/
void setup_M_neurons(struct Mneuron* m_n, struct Pattern myP) {
  m_n->tao = myP.tao;
  m_n->T = myP.T;
  m_n->b = myP.b;
  for (int i = 0; i < NOMBER_M_NEURONS; i++){
  m_n->a[i] = myP.a[i]; 
}}



/******************************************************/
void update_locomotion_network(struct Mneuron* m_n) {
  double Y_tag[NOMBER_M_NEURONS];

  for (int i = 0; i < NOMBER_M_NEURONS; i++) {
    update_M_neuron(&m_n[i], m_n[i].Y);
    Y_tag[i] = m_n[i].y;
  }

  for (int j = 0; j < NOMBER_M_NEURONS; j++) {
    for (int k = 0; k < NOMBER_M_NEURONS; k++) {
      // copy the vector output to each neuron Y
      m_n[j].Y[k] = Y_tag[k];
    }
  }
}



/******************************************************/
/* put your setup code in setup(), to run once */

unsigned long int mystartTime;
unsigned long int count4pattern;
int mode = 1;
double output[NOMBER_M_NEURONS];
int input[NOMBER_M_NEURONS];

void injectCurrent(struct Mneuron* m_n, int injectMag, int currentTime, int startTime, int stopTime) {
  // Function to inject the current
  for (int i = 0; i < NOMBER_M_NEURONS; i++)
  {
    // m_n[i].inj_cur = 0;
    if ((currentTime > (startTime + 10 * i)) && (currentTime < stopTime)  )
    {
      // m_neuron_slow[i].inj_cur = 10;
      m_n[i].inj_cur = injectMag;
    }
  }
}

bool isSensorTouched() {
  sensor_FR = analogRead(analogInPin0);
  sensor_FL = analogRead(analogInPin4);
  sensor_BR = analogRead(analogInPin6);
  sensor_BL = analogRead(analogInPin2);

  if (sensor_FR < 950 || sensor_FL < 950 || sensor_BR < 950 || sensor_BL < 950) {
    return true;
  } else {
    return false;
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(analogInPin0, INPUT);
  pinMode(analogInPin2, INPUT);
  pinMode(analogInPin4, INPUT);
  pinMode(analogInPin6, INPUT);

  for (int i = 0; i < NOMBER_M_NEURONS; i++) {
    setup_M_neurons(&m_neuron_slow[i], CASE_2_slow);
    rightRotate(m_neuron_slow[i].a, i, NOMBER_M_NEURONS);
  }
  for (int i = 0; i < NOMBER_M_NEURONS; i++) {
    setup_M_neurons(&m_neuron_fast[i], CASE_2_fast);
    rightRotate(m_neuron_fast[i].a, i, NOMBER_M_NEURONS);
  }
  //  bool flag = true;
  // Initialize portHandler. Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize packetHandler. Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);


  if (portHandler->openPort()) {
    Serial.print("Succeeded to open the port!\n");
  } else {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)) {
    Serial.print("Succeeded to change the baudrate!\n");
  } else {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  mystartTime = millis();
  start_count_time = millis();
}

int count = 0;
int start_count = 0;
int timee = 0;
int thr_time = 100;



void loop() {


  myTime = millis() - mystartTime;

  // Inject current for a specific amount of start and stop time 
  injectCurrent(m_neuron_fast, 1, myTime, 3000, 30000);
  injectCurrent(m_neuron_slow, 10, myTime, 3000, 10000);
  /* Update the neurons output*/
  update_locomotion_network(m_neuron_fast);
  update_locomotion_network(m_neuron_slow);


  // Logic Mode switcher 
  if (isSensorTouched() && (mode == 1)) {
    // if the sensor is touch and it's in mode 1 set motor input to fast 
    for (int i = 0; i < NOMBER_M_NEURONS; i++)
    // Switch to mode 2 
    mode = 2;

  }
  else if ( (!isSensorTouched()) && (mode == 2) && (timee < thr_time)) {
    // if the sensor is not touch and in mode 2 and time is less than theshold time, increase
    timee = timee + 1;

  } else if ( (!isSensorTouched()) && (mode == 2) && (timee >= thr_time)) {
    // else if the sensor has not been touched over the threshold time, go slow 
    mode = 1;
    timee = 0;
  }


  // Actually give output 
  if (mode == 1) {
    for (int i = 0; i < NOMBER_M_NEURONS; i++)
    {
      output[i] = m_neuron_slow[0].y;
      input[i] = int(map(output[i] * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    }
  } else if (mode == 2) {
    for (int i = 0; i < NOMBER_M_NEURONS; i++)
    {
      output[i] = m_neuron_fast[i].y;
      input[i] = int(map(output[i] * 100,  0.0, 30.0, CW_position_2, CCW_position_2));
    }
  }

  // put your main code here, to run repeatedly:
  //  packetHandler->read1ByteTxRx(portHandler, MOTOR_ID1, MOVING, (uint8_t*)&isMoving1, &dxl_error);
  //  packetHandler->read1ByteTxRx(portHandler, MOTOR_ID2, MOVING, (uint8_t*)&isMoving2, &dxl_error);


  //  if( isMoving1 == 0 ){ //if Dynamixel is stopped
  //Send instruction packet to move for goalPosition
  if (myTime > (5000)) {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID1, ADDR_AX_GOAL_POSITION, input[0], &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID2, ADDR_AX_GOAL_POSITION, input[1], &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID3, ADDR_AX_GOAL_POSITION, input[2], &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID4, ADDR_AX_GOAL_POSITION, input[3], &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID5, ADDR_AX_GOAL_POSITION, input[3], &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID6, ADDR_AX_GOAL_POSITION, input[5], &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID7, ADDR_AX_GOAL_POSITION, input[6], &dxl_error);
  }

  //toggle the position if goalPosition is 1000, set to 0, if 0, set to 1000

  //  }

  packetHandler->read2ByteTxRx(portHandler, MOTOR_ID1, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);
  packetHandler->read2ByteTxRx(portHandler, MOTOR_ID2, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dx2_present_position, &dxl_error);

    for (int i = 0; i < NOMBER_M_NEURONS; i++)
    {
     Serial.print(output[i]);
      Serial.print("\t");
    }
  Serial.print("\n");
}
