
// adapted from file jkcode 

/*=================Matsuoka neuron======================*/
#define DISCRIPTION_LENGTH     15
#define NOMBER_M_NEURONS     7
unsigned long int myTime;
unsigned long int count_time;
unsigned long int start_count_time;
unsigned int mydelay = 10; // ms -> this is the interval

/******************************************************/ 
//struct Matsuoka neuron 
struct Mneuron { 
  char discription[DISCRIPTION_LENGTH]; // name 
  double tao   = 0;  
  double T     = 0; 
  double b     = 0; 
  double x_0    = 0;
  double xh_0   = 0;
  double x      = x_0; 
  double xh     = xh_0;
  double y = 0;
//  double Y[NOMBER_M_NEURONS] = {0,0,0,0};
//  double a[NOMBER_M_NEURONS]= {0,1.5,1.5,0};
  double Y[NOMBER_M_NEURONS] = {0,0,0,0,0,0,0};
  double a[NOMBER_M_NEURONS]= {1,1.2,1.2,1.2,1.2,0,0};
  double inj_cur = 0; 
} m_neuron[NOMBER_M_NEURONS]; 


/******************************************************/ 
//struct Pattern 
struct Pattern{
  double tao;
  double T;
  double b;
  };
Pattern CASE_1 = {1,12,0};
//Pattern CASE_2 = {0.085,60,5.5};
//Pattern CASE_2 = {0.04,60,5.5};
Pattern CASE_2 = {0.05,12,2.5};
Pattern CASE_3 = {1,120000000000000000,25000000000000000};

/******************************************************/ 
inline double diff_x (double x , double xh , double tao, double b , double a[],  double Y[], double inj_cur){
    double totalinput = 0;
    for (int i=0; i<NOMBER_M_NEURONS; i++) {
        totalinput = totalinput + a[i]*Y[i];
    }
    return (double)(-1/tao)*(x + b*xh + totalinput - inj_cur) ; } 

/******************************************************/ 
inline double  diff_xh (double x , double xh,  double T, double y)
 { return (double)(-1/T)*(xh - y); } 
//  default theta =0; y = max(x, theta) = x

/******************************************************/ 
inline double y_max (double x) {
    return (x > 0 ) ? x : 0;
}

/******************************************************/ 
// C++ program for right rotation of an array (Reversal Algorithm)
// Ref URL: https://www.geeksforgeeks.org/reversal-algorithm-right-rotation-array/
// Example: Input: arr[] = {1, 2, 3, 4}; k = 2 → Output: 3,4,1,2
void reverseArray(double arr[], int start, int end){
    while (start < end)    {
        std::swap(arr[start], arr[end]);
        start++;
        end--;}
}
/* Function to right rotate arr[] of size n by d */
void rightRotate(double arr[], int d, int n){
    // if in case d>n,this will give segmentation fault.
    d=d%n;
    reverseArray(arr, 0, n-1);
    reverseArray(arr, 0, d-1);
    reverseArray(arr, d, n-1);
}

/******************************************************/ 
// RK Method
void update_M_neuron(struct Mneuron* m_n, double Y[]){
int n = 20; 
double h; 
//mydelay = Δt[ms] = f(x+Δt)-f(x)
h= ((double)mydelay/1000)/n;
double ya[n],yb[n],y[n]; 
double k1,k2,k3,k4,k, l1,l2,l3,l4,l;  

ya[0] = m_n->x;
yb[0] = m_n->xh;
y[0]= m_n->y;

for (int i=0; i<n-1 ; i++){
  k1 = h*diff_x(ya[i], yb[i], m_n->tao, m_n->b, m_n->a, Y, m_n->inj_cur);
  l1 = h*diff_xh(ya[i], yb[i], m_n->T, y[0]);

  k1 = h*diff_x(ya[i] + k1/2 , yb[i]+ l1/2, m_n->tao, m_n->b, m_n->a, Y, m_n->inj_cur);
  l1 = h*diff_xh(ya[i]+ k1/2  , yb[i]+ l1/2 , m_n->T, y[0]);
  
  k1 = h*diff_x(ya[i]+k2/2, yb[i]+l2/2, m_n->tao, m_n->b, m_n->a, Y, m_n->inj_cur);
  l1 = h*diff_xh(ya[i]+k2/2, yb[i]+l2/2, m_n->T, y[0]);
  
  k1 = h*diff_x(ya[i] + k3, yb[i] + l3, m_n->tao, m_n->b, m_n->a, Y, m_n->inj_cur);
  l1 = h*diff_xh(ya[i] + k3 , yb[i] + l3, m_n->T, y[0]);


  k = 1/6.0 * (k1 + 2*k2 + 2*k3 + k4);
  l  = 1/6.0 * ( l1 + 2*l2  + 2*l3  +  l4);

  ya[i+1]= ya[i]+k;
  yb[i+1]= yb[i]+l;
  y[i+1]= y_max(ya[i]);
} 
m_n->x = ya[n-1]; 
m_n->xh = yb[n-1]; 
m_n->y = y[n-1]; 
return; }

/******************************************************/ 
void setup_M_neurons(struct Mneuron* m_n,struct Pattern myP){
m_n->tao = myP.tao; 
m_n->T = myP.T;
m_n->b = myP.b;
}

/******************************************************/ 
void update_locomotion_network(void){
double Y_tag[NOMBER_M_NEURONS];

for (int i = 0; i< NOMBER_M_NEURONS ; i++){
    update_M_neuron(&m_neuron[i], m_neuron[i].Y);
    Y_tag[i] = m_neuron[i].y;
    }

for (int j = 0; j< NOMBER_M_NEURONS ; j++){
  for (int k = 0; k< NOMBER_M_NEURONS ; k++){
  // copy the vector output to each neuron Y
      m_neuron[j].Y[k] = Y_tag[k];
      }
  }
}

/******************************************************/ 
/* put your setup code in setup(), to run once */

unsigned long int mystartTime;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  for (int i = 0; i< NOMBER_M_NEURONS ; i++){
    setup_M_neurons(&m_neuron[i],CASE_2);
    rightRotate(m_neuron[i].a, i, NOMBER_M_NEURONS);
  }


  mystartTime = millis();
  start_count_time = millis();
  
}

void loop() {

  myTime = millis()- mystartTime;
  
  for (int i = 0; i< NOMBER_M_NEURONS ; i++){
    m_neuron[i].inj_cur = 0;
    if((myTime>(3000 + 1000*i))){
    m_neuron[i].inj_cur = 1;
    }
  }
  /* Update the neurons output*/
  update_locomotion_network();
  double output1 = m_neuron[0].y;
  double output2 = m_neuron[1].y;
  double output3 = m_neuron[2].y;
  double output4 = m_neuron[3].y;
  double output5 = m_neuron[4].y;
  double output6 = m_neuron[5].y;
  double output7 = m_neuron[6].y;
  
  int input1 = int(map(output1*100,0.0,30.0,462,622));
  int input2 = int(map(output2*100,0.0,30.0,462,622));
  int input3 = int(map(output3*100,0.0,30.0,462,622));
  int input4 = int(map(output4*100,0.0,30.0,462,622));
  int input5 = int(map(output5*100,0.0,30.0,462,632));
  int input6 = int(map(output6*100,0.0,30.0,462,632));
  int input7 = int(map(output7*100,0.0,30.0,462,632));
  // put your main code here, to run repeatedly:
//  packetHandler->read1ByteTxRx(portHandler, MOTOR_ID1, MOVING, (uint8_t*)&isMoving1, &dxl_error);
//  packetHandler->read1ByteTxRx(portHandler, MOTOR_ID2, MOVING, (uint8_t*)&isMoving2, &dxl_error);

  
//  if( isMoving1 == 0 ){ //if Dynamixel is stopped
    //Send instruction packet to move for goalPosition
    //toggle the position if goalPosition is 1000, set to 0, if 0, set to 1000

//  }
  
  
//  packetHandler->write2ByteTxRx(portHandler, MOTOR_ID1, CHANGE_SPEED, dx1_change_speed, &dxl_error);
//  packetHandler->read2ByteTxRx(portHandler, MOTOR_ID1, READ_SPEED, (uint16_t*)&dxl_read_change_speed, &dxl_error);

//  Serial.print("ID : ");
//  Serial.print(MOTOR_ID1);
//  Serial.print(" Goal Position : ");
//  Serial.print(goalPosition1);
//  Serial.print(" current position: ");
  if (input1 == 452){
    count_time = millis()-start_count_time;
    start_count_time = millis();
    }

  Serial.print(output1);
  Serial.print("\t");
    Serial.print(output2);
  Serial.print("\t");
    Serial.print(output3);
  Serial.print("\t");
    Serial.print(output4);
  Serial.print("\t");
    Serial.print(output5);
  Serial.print("\t");
    Serial.print(output6);
  Serial.print("\t");
    Serial.print(output7);
  Serial.print("\t");
  
  Serial.print("Loop Completion time");
  Serial.print(millis()- myTime);
//   Serial.print(input1);
// //  Serial.print("\n");
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
//   Serial.print("\t");
//   Serial.print(count_time);

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