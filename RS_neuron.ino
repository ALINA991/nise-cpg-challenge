
#define DISCRIPTION_LENGTH     15
#define NOMBER_RS_NEURONS     4
unsigned long int myTime;
unsigned int mydelay = 5; // ms
/******************************************************/ 
/* Rowat P.F., Selverston, A.I., 
 *  Oscillatory Mechanisms in Pairs of Neurons Connected 
 *  with Fast Inhibitory Synapses, Journal of Computationa 
 *  Neuroscience, 1997. 
 */
/******************************************************/ 
//struct RSneuron 
/******************************************************/ 
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
Pattern OSCILLATORY={4.6,1.5,0.1,1};


/*
  QUIESCENT
   ^
   |     
   |   /\
   |  /  \
   | /    \____________
   ------------------------> 
*/
Pattern QUIESCENT={0.5,0.5,0.1,5};

/*
  PL
   ^
   |     ------------
   |   /
   |  / 
   | /
   ------------------------> 
*/
Pattern PLATEAU = {0.1,5,0.1,1};

/*  Almost-Osc
   ^
   |     
   |   /\
   |  /  \    /\  
   | /    \  /  \/\-----
   |/      \/
   ------------------------> 
*/
Pattern ALMOSTOSC = {5,0.9,0.1,1};


Pattern Almolok = {4.6,1.5,0.5,1};
// Increase tao 

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
void setup_RS_neurons(struct RSneuron* rs_n,struct Pattern myP, String str)
{
  for(int i=0 ; i<(sizeof(str) / sizeof(str[0])) ; i++) 
      rs_n->discription[i] = str[i]; 

  rs_n->tao_m = myP.tao_m; 
  rs_n->tao_s = 20 * myP.tao_m; 
  rs_n->sigma_s = myP.sigma_s;
  rs_n->sigma_f = myP.sigma_f;
  rs_n->inj_cur_MultiplicationFactor = myP.InjCurrentMultiplicationFactor;

}

/****************************************************/
void inject_current(unsigned long int currenttime, struct RSneuron* neuron, int start_time, int end_time)
{
  if ((currenttime>start_time) && (currenttime<end_time))
  neuron->inj_cur = 1; 
  else
  neuron->inj_cur = 0; 
}

/******************************************************/ 
void update_locomotion_network(void)
{
  for (int i = 0; i< NOMBER_RS_NEURONS ; i++)
    update_RS_neuron(&rs_neuron[i]);
}


/******************************************************/ 
/* put your setup code in setup(), to run once */
void setup() {

Serial.begin(115200);

/* set the configuration of the RS neuron to match a desired output: OSCILLATORY, QUIESCENT, PLATEAU, ALMOSTOSC */
setup_RS_neurons(&rs_neuron[0],OSCILLATORY,"Fist neuron");
// setup_RS_neurons(&rs_neuron[1],QUIESCENT,"Second neuron"); //PLATEAU
// setup_RS_neurons(&rs_neuron[2],OSCILLATORY,"Third neuron");
// setup_RS_neurons(&rs_neuron[3],QUIESCENT,"Fourth neuron");
setup_RS_neurons(&rs_neuron[1],Almolok,"NewPattern");
}

/******************************************************/ 
/* put your main code here in loop(), to run repeatedly */
void loop() {
/* Read my program running time in milliseconds */
myTime = millis();

/*Check the current time of this loop 
inject current look at the current  time 
the 3rd and 4th arguments are start and end time respectively */
inject_current(myTime, &rs_neuron[0], 5000, 5050);
// inject_current(myTime, &rs_neuron[1], 6000, 6020);
// inject_current(myTime, &rs_neuron[2], 7000, 7020);
// inject_current(myTime, &rs_neuron[3], 7000, 8020);
inject_current(myTime, &rs_neuron[1], 5000, 5050);

/* Update the neurons output*/
update_locomotion_network();

/* Printing the output of the neurons on serial port*/
for (int i = 0; i< NOMBER_RS_NEURONS ; i++)
{Serial.print(rs_neuron[i].V);Serial.print(" ");}
// Serial.print("\t");
// Serial.print(5.0);
Serial.print("\t");
// Serial.print(-1.5);
Serial.print("Loop Completion time")
Serial.print(millis()- myTime);
Serial.print("\n");


/* delay at the end */
delay(mydelay);

}
