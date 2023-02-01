#define PI 3.1415926535897932384626433832795
#define N_S_NEURONS 14

double c_v1 = 0.2; //*0.1;
double c_v0 = 0.3; //*0.1;
double c_R1 = 0.065; //*0.1;
double c_R0 = 0.196; //*0.1;

double v_sat = 0; // 0;
double R_sat = 0; //0;

int a_b = 20;

double d_low  = 1; //boundaries for input 
double d_high = 5;
double d = 4; // input current 
double dt = 0.1;

double t1; double t2; double t_tot;

double theta[N_S_NEURONS]; //phase 
double r[N_S_NEURONS]; // amplitude 
double x_s[N_S_NEURONS]; // output 
double v[N_S_NEURONS] = {1,1};  //freq
double u[N_S_NEURONS];


//
//double ddr1 = a[0] * ( a[0] / 4 * ( ))
                                            // 1    2    3    4    5    6    7    8    9   10   11   12   13   14   
int w_matrix[N_S_NEURONS][N_S_NEURONS] = {{   0,  10,   0,   0,   0,   0,   0,   10,  0,    0,   0,   0,   0,   0},      //  1 
                                          {  10,   0,  10,   0,   0,   0,   0,   0,   10,   0,   0,   0,   0,   0},    //  2 
                                          {   0,  10,   0,  10,   0,   0,   0,   0,   0,    10,  0,   0,   0,   0},     //  3 
                                          {   0,   0,  10,   0,  10,   0,   0,   0,   0,    0,   10,  0,   0,   0},     //  4 
                                          {   0,   0,   0,  10,   0,  10,   0,   0,   0,    0,   0,   10,  0,   0},      //  5 
                                          {   0,   0,   0,   0,  10,   0,  10,   0,   0,    0,   0,   0,   10,  0},     //  6 
                                          {   0,   0,   0,   0,   0,  10,   0,  10,   0,    0,   0,   0,   0,   10},     //  7 
                                          {  10,   0,   0,   0,   0,   0,  10,   0,  10,    0,   0,   0,   0,   0},      //  8 
                                          {   0,  10,   0,   0,   0,   0,   0,  10,   0,   10,   0,   0,   0,   0},     //  9 
                                          {   0,   0,  10,   0,   0,   0,   0,   0,  10,    0,  10,   0,   0,   0},     // 10 
                                          {   0,   0,  0,   10,   0,   0,   0,   0,   0,   10,   0,  10,   0,   0},      // 11 
                                          {   0,   0,   0,  0,   10,   0,   0,   0,   0,    0,  10,   0,  10,   0},    // 12 
                                          {   0,   0,   0,   0,   0,   10,   0,   0,   0,   0,   0,  10,   0,  10},     // 13 
                                          {   0,   0,   0,   0,   0,   0,   10,   0,   0,   0,   0,   0,  10,   0}     // 14             
                        };
  
double a = (2* PI) / 8; 
                                             // 1    2    3    4    5    6    7    8    9   10   11   12   13   14     
int phi_matrix[N_S_NEURONS][N_S_NEURONS] = {{    0,  -a,   0,   0,   0,   0,   0,   a,   0,   0,   0,   0,   0,   0},   //  1
                                            {    a,   0,  -a,   0,   0,   0,   0,   0,   a,   0,   0,   0,   0,   0},   //  2
                                            {    0,   a,   0,  -a,   0,   0,   0,   0,   0,   a,   0,   0,   0,   0},   //  3
                                            {    0,   0,   a,   0,  -a,   0,   0,   0,   0,   0,   a,   0,   0,   0},   //  4
                                            {    0,   0,   0,   a,   0,  -a,   0,   0,   0,   0,   0,   a,   0,   0},   //  5
                                            {    0,   0,   0,   0,   a,   0,  -a,   0,   0,   0,   0,   0,   a,   0},   //  6
                                            {    0,   0,   0,   0,   0,   a,   0,  -a,   0,   0,   0,   0,   0,   a},   //  7
                                            {   -a,   0,   0,   0,   0,   0,   a,   0,   0,   0,   0,   0,   0,   0},   //  8
                                            {    0,  -a,   0,   0,   0,   0,   0,   a,   0,  -a,   0,   0,   0,   0},   //  9
                                            {    0,   0,  -a,   0,   0,   0,   0,   0,   a,   0,  -a,   0,   0,   0},   // 10
                                            {    0,   0,   0,  -a,   0,   0,   0,   0,   0,   a,   0,  -a,   0,   0},   // 11
                                            {    0,   0,   0,   0,  -a,   0,   0,   0,   0,   0,   a,   0,  -a,   0},   // 12
                                            {    0,   0,   0,   0,   0,  -a,   0,   0,   0,   0,   0,   a,   0,  -a},   // 13
                                            {    0,   0,   0,   0,   0,   0,  -a,   0,   0,   0,   0,   0,   a,   0}    // 14 
                          };


// staturation function for R and v : intrinsic vals for freq and amplitude 
inline double g(double c_v1, double c_v0, double sat, double d_low, double d_high, double d)
{ 
    double res = sat;
    if(d_low<=d && d<=d_high){
        res = c_v1*d + c_v0; 
    }
    return  res; 
}

// build 2 neurons with same values exept for phase theta
// R and v : intrinsic vals for freq and amplitude 


void update_neurons(double d, double theta[N_S_NEURONS], double u[N_S_NEURONS], double r[N_S_NEURONS], double x_s[N_S_NEURONS]){

    for(int i=0; i< N_S_NEURONS ; i++){
        double v;
        double R;

        R = g(c_R1, c_R0, R_sat, d_low, d_high, d);
        v = g(c_v1, c_v0, v_sat, d_low, d_high, d);

        double sum = 2 * PI * v;

        for(int j = 0; j < N_S_NEURONS ; j++)
            sum += r[j] * w_matrix[i][j] * sin(theta[j] - theta[i] - phi_matrix[i][j]); 

        theta[i] = sum*dt + theta[i]; // solve for equation 2 (2nd order diff)
        u[i] = a_b*((a_b/4)*(R-r[i])-u[i])*dt + u[i];
        r[i] = u[i]*dt + r[i];

        x_s[i] = r[i]*(1+cos(theta[i]));
    }
}

void setup(){
  Serial.begin(115200);

};

void loop(){

    d += 0.1;
    t1 = millis();
    update_neurons(d, theta, u, r, x_s);
    t2 = millis();
    t_tot = t2-t1;

    for (int i = 0; i< N_S_NEURONS; i++)
         {Serial.print(x_s[i]);Serial.print(" ");}
    //Serial.print(t_tot);Serial.print(" ");
    Serial.print("\n");
};