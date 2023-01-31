/******************************************************/ 
// one oscillator 
// theta : state variable phase of oscillator 
 // r : state variable amplitude of oscillator

 // v : intrinsic frequency
 // R :  intrinsic amplitude 

 // coupling definition 

 // w_ij : weighths
 // phi_ij : phase biases 

 // signal 
 // x_i = r_i * ( 1 + cos( theta_i))
 // dtheta_i/dt  = 2 * pi * v_i + sum(r_j * w_ij * sin( theta_i - thetai - phi_ij )  )
 // d2theta/ dt2 = a_i * ( a_i /4 * (R_i - r_i ) - dr_r/dt)

// implement and solve with euler method

/******************************************************/ 

#include <bits/stdc++.h>
#define DESCRIPTION_LENGTH     15
#define NUMBER_S_NEURONS     1 // SALAMANDER NEURONS
#define PI (3.141592653589793)
unsigned long int myTime;
unsigned int mydelay = 10; // ms -> this is the interval

double Y_all[NUMBER_S_NEURONS] = {0}; // output from other neurons    : TO INITIALIZE
double w[NUMBER_S_NEURONS] = {0} ; // weight matrix                  : TO INITIALIZE                     
double phi[NUMBER_S_NEURONS] = {0}; // phase biases 
double r_all[NUMBER_S_NEURONS] = {0};  // intrinsic phase               
double theta_all[NUMBER_S_NEURONS] = {0}; 

/******************************************************/ 
//struct Salamander neuron 
struct Sneuron {   // VALUES TO CHECK 

  char description[DESCRIPTION_LENGTH];     // check for spelling here is reuse the script
  double v            = 0;  // intrinsic phase 
  double R            = 0;   // instrisic amplitude : point attractor 
  double theta        = 0;   // state variable phase of oscillator
  double r            = 0;   // state variable amplitude of oscillator -> will converge to R
  double y            = 0;    // output (oscillatory signal) : instantaneous spike firing frequency
  double a            = 0;   // def speed of convergence of r to R 
  double dtheta_dt    = 0;        
  double ddtheta_dt2  = 0;
  double dr_dt        = 0;
  double ddr_dt2      = 0;
  double inj_cur      = 0; 

  // check if have to add time constants

} s_neuron[NUMBER_S_NEURONS];  // initialize list of neurons 

//    CHECK IF ADD SATURATION FUNCTION 
/******************************************************/ 
//struct Pattern 
struct Pattern{           // CHECK IF THESE ARE THE RIGHT PARAMS TO CHANGE PATTERN 
  double d; // drive = injected current 
  double v; // intrinsic phase 
  double R; // intrinsic amplitude 
  double a;  // const 
  };


Pattern TEST = {1,12,0,0};


/******************************************************/ 
inline double diff_theta (double v, double w[], double Y_all[], double phi[]){ 
    int sum = 0;
    for (int i = 0; i < NUMBER_S_NEURONS; i++) { // CHECK WHAT HAPPENS IF I = J
        sum = sum + ( w[i] * Y_all[i] - phi[i] ); 
    }
    return (double)   2 * v * PI + sum; 
  } 

inline double y (struct Sneuron* s_n){
  return s_n -> r * (1 + cos( s_n -> theta));
}

// DEFINE AMPLITUTE VARIBALE CONVERGENCE EQUATION 
// critically damped second order linear differential equation 

void setup_Sneuron(struct Sneuron* s_n, Pattern myP, String str){
  for (int i = 0; i < sizeof(str) / sizeof(str[0]) ; i++)
  s_n -> description[i] = str[i] ; // set name of neuron
  s_n ->  inj_cur = myP.d;
  s_n ->  v = myP.v;
  s_n ->  R = myP.R;
  s_n ->  a = myP.a;
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
// Runge-Kutta 4th Order Method
void update_S_neuron(struct Sneuron* s_n, double Y_all[], double w[], double phi[]){

int n = 20; 
double h; 
//mydelay = Δt[ms] = f(x+Δt)-f(x)
h = ((double)mydelay/1000)/n;

double k1,k2,k3,k4,k, l1,l2,l3,l4,l;  

double v_curr = s_n -> v; // acces intrinstic freq value 
double y_curr = s_n -> y; // access output value
                                            
for (int i=0; i<n-1 ; i++){  

}