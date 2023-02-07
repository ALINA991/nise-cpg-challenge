// from file jk code 

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
  k = h*fun_v( ya[i] , yb[i] , rs_n->sigma_f , rs_n->inj_cur*rs_n->inj_cur_MultiplicationFactor ,  rs_n->tao_m , rs_n->Af ); 
  l = h*fun_q( ya[i] , yb[i] , rs_n->tao_s , rs_n->Es , rs_n->sigma_s ); 

 
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
  setup_RS_neurons(&rs_neuron[0],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[1],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[2],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[3],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[4],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[5],OSCILLATORY1);
  setup_RS_neurons(&rs_neuron[6],OSCILLATORY1);


  mystartTime = millis();
  
}

void loop() {
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
  
//  if((myTime>offset_time)&&(myTime<(offset_time+30))){
//  rs_neuron[1].inj_cur = 1;
//  }
//  if((myTime>(offset_time+300))&&(myTime<(offset_time+330))){
//  rs_neuron[2].inj_cur = 1;
//  }
//  if((myTime>(offset_time+600))&&(myTime<(offset_time+630))){
//  rs_neuron[3].inj_cur = 1;
//  }
//  if((myTime>(offset_time+900))&&(myTime<(offset_time+930))){
//  rs_neuron[4].inj_cur = 1;
//  }
//  if((myTime>(offset_time+1200))&&(myTime<(offset_time+1230))){
//  rs_neuron[5].inj_cur = 1;
//  }
//  if((myTime>(offset_time+1500))&&(myTime<(offset_time+1530))){
//  rs_neuron[6].inj_cur = 1;
//  }

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

  

//  packetHandler->write2ByteTxRx(portHandler, MOTOR_ID1, CHANGE_SPEED, dx1_change_speed, &dxl_error);
//  packetHandler->read2ByteTxRx(portHandler, MOTOR_ID1, READ_SPEED, (uint16_t*)&dxl_read_change_speed, &dxl_error);

//  Serial.print("ID : ");
//  Serial.print(MOTOR_ID1);
//  Serial.print(" Goal Position : ");
//  Serial.print(goalPosition1);
//  Serial.print(" current position: ");
//   Serial.print(input1);
// //  Serial.print("\n");
//   Serial.print("\t");
// //  Serial.print(" is moving: ");
// //  Serial.print(isMoving1);
// //  Serial.print("\t ID : ");
// //  Serial.print(MOTOR_ID2);
// //  Serial.print(" Goal Position : ");
// //  Serial.print(goalPosition2);
// //  Serial.print(" current position: ");
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
