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
struct Mneuron_f {
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
  double a[NOMBER_M_NEURONS] = { 1, 1.2, 1.2, 1.2, 1.2, 0, 0 };
  //  double a_slow[NOMBER_M_NEURONS]= {0.7,1.2,1.2,1.2,1.2,0,0};
  double inj_cur = 0;
} m_neuron_fast[NOMBER_M_NEURONS];

struct Mneuron_s {
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
  //  double a_fast[NOMBER_M_NEURONS]= {1,1.2,1.2,1.2,1.2,0,0};
  double a[NOMBER_M_NEURONS] = { 0.7, 1.2, 1.2, 1.2, 1.2, 0, 0 };
  double inj_cur = 0;
} m_neuron_slow[NOMBER_M_NEURONS];


/******************************************************/
//struct Pattern
struct Pattern {
  double tao;
  double T;
  double b;
};
//Pattern CASE_1 = {1,12,0};


Pattern CASE_2_fast = { 0.05, 12, 2.5 };
Pattern CASE_2_slow = { 0.5, 12, 2.5 };
//Pattern CASE_3 = {1,120000000000000000,25000000000000000};

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
void update_M_neuron_s(struct Mneuron_s* m_n, double Y[]) {
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

void update_M_neuron_f(struct Mneuron_f* m_n, double Y[]) {
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
void setup_M_neurons_f(struct Mneuron_f* m_n, struct Pattern myP) {
  m_n->tao = myP.tao;
  m_n->T = myP.T;
  m_n->b = myP.b;
}

void setup_M_neurons_s(struct Mneuron_s* m_n, struct Pattern myP) {
  m_n->tao = myP.tao;
  m_n->T = myP.T;
  m_n->b = myP.b;
}

/******************************************************/
void update_locomotion_network_slow(void) {
  double Y_tag[NOMBER_M_NEURONS];

  for (int i = 0; i < NOMBER_M_NEURONS; i++) {
    update_M_neuron_s(&m_neuron_slow[i], m_neuron_slow[i].Y);
    Y_tag[i] = m_neuron_slow[i].y;
  }

  for (int j = 0; j < NOMBER_M_NEURONS; j++) {
    for (int k = 0; k < NOMBER_M_NEURONS; k++) {
      // copy the vector output to each neuron Y
      m_neuron_slow[j].Y[k] = Y_tag[k];
    }
  }
}

void update_locomotion_network_fast(void) {
  double Y_tag[NOMBER_M_NEURONS];

  for (int i = 0; i < NOMBER_M_NEURONS; i++) {
    update_M_neuron_f(&m_neuron_fast[i], m_neuron_fast[i].Y);
    Y_tag[i] = m_neuron_fast[i].y;
  }

  for (int j = 0; j < NOMBER_M_NEURONS; j++) {
    for (int k = 0; k < NOMBER_M_NEURONS; k++) {
      // copy the vector output to each neuron Y
      m_neuron_fast[j].Y[k] = Y_tag[k];
    }
  }
}

/******************************************************/
/* put your setup code in setup(), to run once */

unsigned long int mystartTime;
unsigned long int count4pattern;
int mode = 1;
double output1;
double output2;
double output3;
double output4;
double output5;
double output6;
double output7;
double output1s;
double output2s;
double output3s;
double output4s;
double output5s;
double output6s;
double output7s;
double output1_1;
double output2_1;
double output3_1;
double output4_1;
double output5_1;
double output6_1;
double output7_1;
double output1s_1;
double output2s_1;
double output3s_1;
double output4s_1;
double output5s_1;
double output6s_1;
double output7s_1;

int input1;
int input2;
int input3;
int input4;
int input5;
int input6;
int input7;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(analogInPin0, INPUT);
  pinMode(analogInPin2, INPUT);
  pinMode(analogInPin4, INPUT);
  pinMode(analogInPin6, INPUT);

  for (int i = 0; i < NOMBER_M_NEURONS; i++) {
    setup_M_neurons_s(&m_neuron_slow[i], CASE_2_slow);
    rightRotate(m_neuron_slow[i].a, i, NOMBER_M_NEURONS);
  }
  for (int i = 0; i < NOMBER_M_NEURONS; i++) {
    setup_M_neurons_f(&m_neuron_fast[i], CASE_2_fast);
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
int thr_time = 80;
void loop() {


  myTime = millis() - mystartTime;

  sensor_FR = analogRead(analogInPin0);
  sensor_FL = analogRead(analogInPin4);
  sensor_BL = analogRead(analogInPin6);
  sensor_BR = analogRead(analogInPin2);

   Serial.print(sensor_FR);
   Serial.print("\t");
//   Serial.print(sensor_FL);
//   Serial.print("\t");
//   Serial.print(sensor_BR);
//   Serial.print("\t");
//   Serial.print(sensor_BL);
//   Serial.print("\n");

  for (int i = 0; i < NOMBER_M_NEURONS; i++) {
    m_neuron_fast[i].inj_cur = 0;
    m_neuron_slow[i].inj_cur = 0;
    if ((myTime > (3000 + 10 * i))) {
      m_neuron_slow[i].inj_cur = 10;
      m_neuron_fast[i].inj_cur = 1;
    }
  }
  /* Update the neurons output*/
  update_locomotion_network_slow();
  update_locomotion_network_fast();

  if ((sensor_FR < 775 || sensor_FL < 1000 || sensor_BR < 1000 || sensor_BL < 775) && (mode == 1)) {
    // output1 = m_neuron_fast[0].y;
    // output2 = m_neuron_fast[1].y;
    // output3 = m_neuron_fast[2].y;
    // output4 = m_neuron_fast[3].y;
    // output5 = m_neuron_fast[4].y;
    // output6 = m_neuron_fast[5].y;
    // output7 = m_neuron_fast[6].y;
    // input1 = int(map(output1 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input2 = int(map(output2 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input3 = int(map(output3 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input4 = int(map(output4 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input5 = int(map(output5 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input6 = int(map(output6 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input7 = int(map(output7 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));

//    output1 = m_neuron_slow[0].y;
//    output2 = m_neuron_slow[1].y;
//    output3 = m_neuron_slow[2].y;
//    output4 = m_neuron_slow[3].y;
//    output5 = m_neuron_slow[4].y;
//    output6 = m_neuron_slow[5].y;
//    output7 = m_neuron_slow[6].y;
//    input1 = int(map(output1 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
//    input2 = int(map(output2 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
//    input3 = int(map(output3 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
//    input4 = int(map(output4 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
//    input5 = int(map(output5 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
//    input6 = int(map(output6 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
//    input7 = int(map(output7 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    mode = 2;
  } 
  else if (sensor_FR >= 775 && sensor_FL >= 1000 && sensor_BR >= 1000 && sensor_BL >= 775 && (mode == 2) && (timee < thr_time)) {
    timee = timee+1;

   }else if (sensor_FR >= 775 && sensor_FL >= 1000 && sensor_BR >= 1000 && sensor_BL >= 775 && (mode == 2) && (timee >= thr_time)) {
    // output1 = m_neuron_slow[0].y;
    // output2 = m_neuron_slow[1].y;
    // output3 = m_neuron_slow[2].y;
    // output4 = m_neuron_slow[3].y;
    // output5 = m_neuron_slow[4].y;
    // output6 = m_neuron_slow[5].y;
    // output7 = m_neuron_slow[6].y;
    // input1 = int(map(output1 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input2 = int(map(output2 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input3 = int(map(output3 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input4 = int(map(output4 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input5 = int(map(output5 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input6 = int(map(output6 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input7 = int(map(output7 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));

//    output1 = m_neuron_fast[0].y;
//    output2 = m_neuron_fast[1].y;
//    output3 = m_neuron_fast[2].y;
//    output4 = m_neuron_fast[3].y;
//    output5 = m_neuron_fast[4].y;
//    output6 = m_neuron_fast[5].y;
//    output7 = m_neuron_fast[6].y;
//    input1 = int(map(output1 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
//    input2 = int(map(output2 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
//    input3 = int(map(output3 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
//    input4 = int(map(output4 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
//    input5 = int(map(output5 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
//    input6 = int(map(output6 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
//    input7 = int(map(output7 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    mode = 1;
    timee = 0;
  }

  if (mode == 1) {
    // output1 = m_neuron_slow[0].y;
    // output2 = m_neuron_slow[1].y;
    // output3 = m_neuron_slow[2].y;
    // output4 = m_neuron_slow[3].y;
    // output5 = m_neuron_slow[4].y;
    // output6 = m_neuron_slow[5].y;
    // output7 = m_neuron_slow[6].y;
    // input1 = int(map(output1 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input2 = int(map(output2 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input3 = int(map(output3 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input4 = int(map(output4 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input5 = int(map(output5 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input6 = int(map(output6 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    // input7 = int(map(output7 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));

    output1 = m_neuron_fast[0].y;
    output2 = m_neuron_fast[1].y;
    output3 = m_neuron_fast[2].y;
    output4 = m_neuron_fast[3].y;
    output5 = m_neuron_fast[4].y;
    output6 = m_neuron_fast[5].y;
    output7 = m_neuron_fast[6].y;
    input1 = int(map(output1 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    input2 = int(map(output2 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    input3 = int(map(output3 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    input4 = int(map(output4 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    input5 = int(map(output5 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    input6 = int(map(output6 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    input7 = int(map(output7 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
  } else if (mode == 2) {
    // output1 = m_neuron_fast[0].y;
    // output2 = m_neuron_fast[1].y;
    // output3 = m_neuron_fast[2].y;
    // output4 = m_neuron_fast[3].y;
    // output5 = m_neuron_fast[4].y;
    // output6 = m_neuron_fast[5].y;
    // output7 = m_neuron_fast[6].y;
    // input1 = int(map(output1 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input2 = int(map(output2 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input3 = int(map(output3 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input4 = int(map(output4 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input5 = int(map(output5 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input6 = int(map(output6 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));
    // input7 = int(map(output7 * 100, 0.0, 30.0, CW_position_2, CCW_position_2));

    output1 = m_neuron_slow[0].y;
    output2 = m_neuron_slow[1].y;
    output3 = m_neuron_slow[2].y;
    output4 = m_neuron_slow[3].y;
    output5 = m_neuron_slow[4].y;
    output6 = m_neuron_slow[5].y;
    output7 = m_neuron_slow[6].y;
    input1 = int(map(output1 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    input2 = int(map(output2 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    input3 = int(map(output3 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    input4 = int(map(output4 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    input5 = int(map(output5 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    input6 = int(map(output6 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
    input7 = int(map(output7 * 100, 100.0, 180.0, CW_position_1, CCW_position_1));
  }

  // put your main code here, to run repeatedly:
  //  packetHandler->read1ByteTxRx(portHandler, MOTOR_ID1, MOVING, (uint8_t*)&isMoving1, &dxl_error);
  //  packetHandler->read1ByteTxRx(portHandler, MOTOR_ID2, MOVING, (uint8_t*)&isMoving2, &dxl_error);


  //  if( isMoving1 == 0 ){ //if Dynamixel is stopped
  //Send instruction packet to move for goalPosition
  if (myTime > (5000)) {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID1, ADDR_AX_GOAL_POSITION, input1, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID2, ADDR_AX_GOAL_POSITION, input2, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID3, ADDR_AX_GOAL_POSITION, input3, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID4, ADDR_AX_GOAL_POSITION, input4, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID5, ADDR_AX_GOAL_POSITION, input5, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID6, ADDR_AX_GOAL_POSITION, input6, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID7, ADDR_AX_GOAL_POSITION, input7, &dxl_error);
  }

  //toggle the position if goalPosition is 1000, set to 0, if 0, set to 1000

  //  }

  packetHandler->read2ByteTxRx(portHandler, MOTOR_ID1, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);
  packetHandler->read2ByteTxRx(portHandler, MOTOR_ID2, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dx2_present_position, &dxl_error);

  //  packetHandler->write2ByteTxRx(portHandler, MOTOR_ID1, CHANGE_SPEED, dx1_change_speed, &dxl_error);
  //  packetHandler->read2ByteTxRx(portHandler, MOTOR_ID1, READ_SPEED, (uint16_t*)&dxl_read_change_speed, &dxl_error);

  //  Serial.print("ID : ");
  //  Serial.print(MOTOR_ID1);
  //  Serial.print(" Goal Position : ");
  //  Serial.print(goalPosition1);
  //  Serial.print(" current position: ");
//  if (input1 == 452) {
//    count_time = millis() - start_count_time;
//    start_count_time = millis();
//  }

  // Serial.print(input1);
  // Serial.print("\t");
  // Serial.print(input2);
  // Serial.print("\t");
  // Serial.print(input3);
  // Serial.print("\t");
  // Serial.print(input4);
  // Serial.print("\t");
  // Serial.print(input5);
  // Serial.print("\t");
  // Serial.print(input6);
  // Serial.print("\t");
  // Serial.print(input7);

  Serial.print(input1);
  Serial.print("\t");
  Serial.print(input2);
//  Serial.print(2);
  Serial.print("\t");
  Serial.print(input3);
  Serial.print("\t");
  Serial.print(input4);
  Serial.print("\t");
  Serial.print(input5);
  Serial.print("\t");
  Serial.print(input6);
  Serial.print("\t");
  Serial.print(input7);
  Serial.print("\t");
  Serial.print(timee);
  // Serial.print("\t");
  //Serial.print(count_time);
  // Serial.print("\t");
  // Serial.print(mode);
  //  Serial.print(" is moving: ");
  //  Serial.print(isMoving2);
  //  Serial.print("\t Speed : ");
  //  Serial.print(dxl_read_change_speed);
  //  Serial.print("\n");
  //  Serial.print("\t Torque : ");
  //  Serial.print(dx1_read_change_torque);
  Serial.print("\n");
  //  delay(100);
}
