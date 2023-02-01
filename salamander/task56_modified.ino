#define PI 3.1415926535897932384626433832795

double c_v1_b = 0.2*0.1;
double c_v0_b = 0.3*0.1;
double c_v1_l = 0.2*0.5;
double c_v0_l = 0*0.5;
double c_R1_b = 0.065*0.1;
double c_R0_b = 0.196*0.1;
double c_R1_l = 0.131*0.5;
double c_R0_l = 0.131*0.5;
double v_sat = 0.1;// 0;
double R_sat = 0.1; //0;
double d_low_b = 1;
double d_high_b = 5;
double d_low_l = 1;
double d_high_l = 3;

double N_S_NEURONS = 20;

int a_b = 20; // body 
double dt = 0.1;

double t1; double t2; double t_tot;

double theta[20]; //phase 
double u[20];
double r[20]; // amplitude 
double x_s[20]; // output 
double d = 0; // input current 
double v[20];


// staturation function for R 
inline double g(double c_v1, double c_v0, double sat, double d_low, double d_high, double d)
{ 
    double res = sat;
    if(d_low<=d && d<=d_high){
        res = c_v1*d + c_v0; 
    }
    return  res; 
}
inline double g_vb(double d)
{ return g(c_v1_b, c_v0_b, v_sat, d_low_b, d_high_b, d); } // V body 

inline double g_vl(double d)
{ return g(c_v1_l, c_v0_l, v_sat, d_low_l, d_high_l, d); }  // V limb

inline double g_Rb(double d)
{ return g(c_R1_b, c_R0_b, R_sat, d_low_b, d_high_b, d); } // R body

inline double g_Rl(double d)
{ return g(c_R1_l, c_R0_l, R_sat, d_low_l, d_high_l, d); }  // R limb 

// fila (lo que recibe)
                            // 1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19    20
int w_matrix[20][20] =  {  {   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,   0,   0,   0,  30,   0,   0,   0},   //  1 
                           {  10,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,   0,   0,  30,   0,   0,   0},   //  2 
                           {   0,  10,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,   0,  30,   0,   0,   0},   //  3 
                           {   0,   0,  10,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,  30,   0,   0,   0},   //  4 
                           {   0,   0,   0,  10,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,   0,  30,   0},   //  5 
                           {   0,   0,   0,   0,  10,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,  30,   0},   //  6 
                           {   0,   0,   0,   0,   0,  10,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,  30,   0},   //  7 
                           {   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,   0,   0,   0,   0,  10,   0,   0,  30,   0},   //  8 
                           {  10,   0,   0,   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,   0,   0,   0,  30,   0,   0},   //  9 
                           {   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,  10,   0,   0,   0,   0,   0,   0,  30,   0,   0},   // 10 
                           {   0,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,  10,   0,   0,   0,   0,   0,  30,   0,   0},   // 11 
                           {   0,   0,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,  10,   0,   0,   0,   0,  30,   0,   0},   // 12 
                           {   0,   0,   0,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,  10,   0,   0,   0,   0,   0,  30},   // 13 
                           {   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,  10,   0,   0,   0,   0,  30},   // 14 
                           {   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,  10,   0,   0,   0,  30},   // 15 
                           {   0,   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,   0,   0,  10,   0,   0,   0,   0,  30},   // 16 
                           {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  10,  10,   0},   // 17 
                           {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  10,   0,   0,  10},   // 18 
                           {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  10,   0,   0,  10},   // 19 
                           {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  10,  10,   0},   // 20 
                        };

double a = 2*PI/8;
double pi = PI;
                                //   1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20
double phi_matrix[20][20] = {   {    0,  -a,   0,   0,   0,   0,   0,   0,  pi,   0,   0,   0,   0,   0,   0,   0,  pi,   0,   0,   0},   //  1
                                {    a,   0,  -a,   0,   0,   0,   0,   0,   0,  pi,   0,   0,   0,   0,   0,   0,  pi,   0,   0,   0},   //  2
                                {    0,   a,   0,  -a,   0,   0,   0,   0,   0,   0,  pi,   0,   0,   0,   0,   0,  pi,   0,   0,   0},   //  3
                                {    0,   0,   a,   0,  -a,   0,   0,   0,   0,   0,   0,  pi,   0,   0,   0,   0,  pi,   0,   0,   0},   //  4
                                {    0,   0,   0,   a,   0,  -a,   0,   0,   0,   0,   0,   0,  pi,   0,   0,   0,   0,   0,  pi,   0},   //  5
                                {    0,   0,   0,   0,   a,   0,  -a,   0,   0,   0,   0,   0,   0,  pi,   0,   0,   0,   0,  pi,   0},   //  6
                                {    0,   0,   0,   0,   0,   a,   0,  -a,   0,   0,   0,   0,   0,   0,  pi,   0,   0,   0,  pi,   0},   //  7
                                {    0,   0,   0,   0,   0,   0,   a,   0,   0,   0,   0,   0,   0,   0,   0,  pi,   0,   0,  pi,   0},   //  8
                                {   pi,   0,   0,   0,   0,   0,   0,   0,   0,  -a,   0,   0,   0,   0,   0,   0,   0,  pi,   0,   0},   //  9
                                {    0,  pi,   0,   0,   0,   0,   0,   0,   a,   0,  -a,   0,   0,   0,   0,   0,   0,  pi,   0,   0},   // 10
                                {    0,   0,  pi,   0,   0,   0,   0,   0,   0,   a,   0,  -a,   0,   0,   0,   0,   0,  pi,   0,   0},   // 11
                                {    0,   0,   0,  pi,   0,   0,   0,   0,   0,   0,   a,   0,  -a,   0,   0,   0,   0,  pi,   0,   0},   // 12
                                {    0,   0,   0,   0,  pi,   0,   0,   0,   0,   0,   0,   a,   0,  -a,   0,   0,   0,   0,   0,  pi},   // 13
                                {    0,   0,   0,   0,   0,  pi,   0,   0,   0,   0,   0,   0,   a,   0,  -a,   0,   0,   0,   0,  pi},   // 14
                                {    0,   0,   0,   0,   0,   0,  pi,   0,   0,   0,   0,   0,   0,   a,   0,  -a,   0,   0,   0,  pi},   // 15
                                {    0,   0,   0,   0,   0,   0,   0,  pi,   0,   0,   0,   0,   0,   0,   a,   0,   0,   0,   0,  pi},   // 16
                                {    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  pi,  pi,   0},   // 17
                                {    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  pi,   0,   0,  pi},   // 18
                                {    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  pi,   0,   0,  pi},   // 19
                                {    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  pi,  pi,   0},   // 20
                            };
// added freq here as array 
void update_neurons(double d, double theta[20], double u[20], double r[20], double x_s[20], double v[20]){

    for(int i=0; i<20 ; i++){
        //double v;
        double R;

        if(16 <= i <= 19){ // last neurons == limbs ? 
            v[i] = g_vl(d);
            R = g_Rl(d);
        }else{
            v[i] = g_vb(d);    // first 16 neurons == body ? 
            R = g_Rb(d);
        }
        double sum = 2*PI*v[i];

        for(int j=0; j<20 ; j++)
            sum += r[j] * w_matrix[i][j] * sin(theta[j] - theta[i] - phi_matrix[i][j]); 

        theta[i] = sum*dt + theta[i]; // solve for equation 2 (2nd order diff)
        r[i] = u[i]*dt + r[i];
        u[i] = a_b*((a_b/4)*(R-r[i])-u[i])*dt + u[i];
        x_s[i] = r[i]*(1+cos(theta[i]));
    }
}

void setup() {
    Serial.begin(115200);
}


// y(t) = A * sin(2 * PI * f * t + shift)

void loop() {
    d += 0.1;
    t1 = millis();
    update_neurons(d, theta, u, r, x_s, v); 
    t2 = millis();
    t_tot = t2-t1;

    // for (int i = 0; i< 20; i++)
    //     {Serial.print(x_s[i]);Serial.print(" ");}
    //Serial.print(theta[0]);Serial.print(" ");
    //Serial.print(r[0]);Serial.print(" ");


for (int i = 0; i< 2 ; i++)
  {Serial.print(x_s[i]);Serial.print(" ");}
  Serial.print("\n");
    // Serial.println(x_s[0]);Serial.print(" ");
    // Serial.print("\t");
    // Serial.println(x_s[1]);Serial.print(" ");
    //Serial.println(r[1] * sin( 2* PI * v[1] + theta[1] ));

    //Serial.println( (2 * PI * ( v[0] - v[1])) / c_R0_b* w[0][1] );
    // Serial.print(r[2]);Serial.print(" ");
    // Serial.print(r[3]);Serial.print(" ");
    // Serial.print(r[4]);Serial.print(" ");

    
}