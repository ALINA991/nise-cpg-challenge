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
#define NUMBER_S_NEURONS     2 // SALAMANDER NEURONS
#define PI (3.141592653589793)
unsigned long int myTime;
unsigned int mydelay = 10; // ms -> this is the interval

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
  double inj_cur       = 0; 

  double Y_all[NUMBER_S_NEURONS] = {0,0}; // output from other neurons    : TO INITIALIZE
  double w[NUMBER_S_NEURONS] = {0,0} ; // weight matrix               : TO INITIALIZE                     
  double phi[NUMBER_S_NEURONS] = {0,0}; // phase biases 
  double r_all[NUMBER_S_NEURONS] = {0,0};                 // CHECK HOW TO DEFINE THIS MORE EFFICIENTLY
  double theta_all[NUMBER_S_NEURONS] = {0,0};


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

inline double find_r (struct Sneuron* s_n, int i) { // get amplitude value of ith neuron
    return s_n -> r_all[i];
}

inline double find_theta (struct Sneuron* s_n, int i) { // get phase value of ith neuron 
    return s_n -> theta_all[i];
}


/******************************************************/ 
inline double diff_theta (struct Sneuron* s_n){ 
    int sum = 0;
    for (int i = 0; i < NUMBER_S_NEURONS; i++) { // CHECK WHAT HAPPENS IF I = J
        sum = sum + (  + s_n -> w[i] * s_n -> Y_all[i] - s_n -> phi[i] ); 
    }
    // Serial.print(Y[0]);Serial.print(" ");
    // Serial.print("\n");
    return (double)   2 * s_n -> v * PI + sum; 
  } 