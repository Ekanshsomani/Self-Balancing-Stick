#include<Wire.h>

////////////////////////////////////////////
//              Motor Driver              //
////////////////////////////////////////////

#define PWMA A0
#define Ain2 5
#define Ain1 6
#define PWMB A1
#define Bin2 7
#define Bin1 8

String package;

////////////////////////////////////////////
//          Controller Variables          //
////////////////////////////////////////////

float   Kp1_A = 500;            // K_p for motor A    (for theta_A_error)
float   Kp2_A = 32;             // K_d for motor A    (for dTheta_A)
float   Kp3_A = 2;              // K_p,w for motor A  (for A_speed)

float   A_set_speed = 0;        // Controller output for motor A
float   A_set_PWM = 0;          // Setpoint PWM for motor A (from A_set_speed)
float   A_set_PWM_prev = 0;     // Previous PWM setpoint

float   Kp1_B = 500;            // K_p for motor B    (for theta_B_error)
float   Kp2_B = 32;             // K_d for motor B    (for dTheta_B)
float   Kp3_B = 2;              // K_p,w for motor B  (for B_speed)

float   B_set_speed = 0;        // Controller output for motor A
float   B_set_PWM = 0;          // Setpoint PWM for motor A (from A_set_speed)
float   B_set_PWM_prev = 0;     // Previous PWM setpoint

////////////////////////////////////////////
//           Theta Calculations           //
////////////////////////////////////////////
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double angle_x, angle_y;

int minVal=265;
int maxVal=402;

double theta_A, theta_A_prev, dTheta_A=0, dTheta_A_prev, theta_A_zero=0,
theta_A_error, theta_A_zero_prev;

double theta_B, theta_B_prev, dTheta_B=0, dTheta_B_prev, theta_B_zero=0,
theta_B_error, theta_B_zero_prev;

float cycle_time, theta_time=0;

int t=0;


////////////////////////////////////////////
//              Motor Speeds              //
////////////////////////////////////////////
float A_speed=0, A_speed_prev, A_time_now=0, A_time_prev, A_tach_rev_tick=0, 
A_time_diff=1, A_dir=1, A_dir_set=1;
float B_speed=0, B_speed_prev, B_time_now=0, B_time_prev, B_tach_rev_tick=0, 
B_time_diff=1, B_dir=1, B_dir_set=1;

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() {
  if(t < 2500){
    //Delay the start of the motors to let the IMU calibrate
    getTheta(); 
    t++;
  }else{
    // Read Motor Speeds
    getMotorSpeeds();
    // Calculate Theta and dTheta
    getTheta();
    // Let controller calculate set values for motors
    getPWM();
    // Set desired motor values
    setMotorSpeeds();
    // Print to Serial output for plotting
    printSerial();
  }
}
/*
////////////////////////////////////////////
//         Motor Encoder Interrupts       //
////////////////////////////////////////////
void rpm_A_tach() {
  A_tach_rev_tick++;

  if (A_tach_rev_tick >= tach_rev_res) {
    A_time_prev = A_time_now;
    A_time_now = micros();
    A_time_diff = A_time_now - A_time_prev;

    A_tach_rev_tick = 0;
  }
}

void rpm_B_tach() {
  B_tach_rev_tick++;

  if (B_tach_rev_tick >= tach_rev_res) {
    B_time_prev = B_time_now;
    B_time_now = micros();
    B_time_diff = B_time_now - B_time_prev;

    B_tach_rev_tick = 0;
  }
}
*/

////////////////////////////////////////////
//    Calculate Motor Speeds in rad/s     //
////////////////////////////////////////////
void getMotorSpeeds(){
  /// MOTOR A
  A_speed_prev = A_speed;
  
  //if time since last revolution is too big
  /*if ( (micros() - A_time_now) > (long)(500000.) ) {
    A_speed = 0;
  } else {*/
    A_speed = (1000000./A_time_diff)*2*M_PI;
    if(A_dir != A_dir_set && abs(A_speed_prev) < 17 && abs(A_speed) > abs(A_speed_prev)){
      A_dir = A_dir_set;
    }
    // Set speed with direction
    A_speed = A_speed * A_dir;
  //}

  /// MOTOR B
  B_speed_prev = B_speed;

  // Check if time since last revolution is too big
  /*if ( (micros() - B_time_now) > (long)(500000.) ) {
    B_speed = 0;
  } else {*/
    // Calculate speed in rad/s
    B_speed = 1000000./B_time_diff * 2*M_PI ;// * A_dir;
    // Determine direction of speed
    if(B_dir != B_dir_set && abs(B_speed_prev) < 17 && abs(B_speed) > abs(B_speed_prev)){
      B_dir = B_dir_set;
    }
    // Set speed with direction
    B_speed = B_speed * B_dir;
  //}
  //B_speed = 0.3*B_speed + 0.7*B_speed_prev;
}

void getTheta(){
  getIMUValues();

  cycle_time = (float)(micros() - theta_time);
  theta_time = (float) micros();

  theta_A_prev = theta_A;
  dTheta_A_prev = dTheta_A;
  
  // Filter theta value
  theta_A = (float) (1-0.7)*angle_y+ 0.7*theta_A_prev;
  
  // Find dTheta and apply filter
  dTheta_A = (float) (theta_A - theta_A_prev)/cycle_time * 1000000.;
  dTheta_A = (float) (1-0.7)*dTheta_A + 0.7*dTheta_A_prev;

  theta_A_error = theta_A_zero - theta_A;

  /* ////// From Mike Rouleau's Self Balancing Stick /////////
   *  Moves the zero reference in order to compensate for center of mass
   *  not being at theta_B = 0
   *  Is also used if a weight is added to change center of mass
   */
  theta_A_zero_prev = theta_A_zero;
  theta_A_zero = theta_A_zero - 0.6 * theta_A_error;
  theta_A_zero = (1 - 0.995) * theta_A_zero + 0.995 * theta_A_zero_prev;
  /*
   * /////// End /////////////
   */
  
  // Calculations for theta_B and dTheta_B

  theta_B_prev = theta_B;
  dTheta_B_prev = dTheta_B;

  // Filter theta value and find error
  theta_B = (float) (1-0.7)*angle_y + 0.7*theta_B_prev;
  theta_B_error = theta_B_zero - theta_B;
  // Calculate dTheta and apply finltering
  dTheta_B = (float) (theta_B - theta_B_prev)/cycle_time * 1000000.;
  dTheta_B = (float) (1-0.7)*dTheta_B + 0.7*dTheta_B_prev;


  /* ////// From Mike Rouleau's Self Balancing Stick /////////
   *  Moves the zero reference in order to compensate for center of mass
   *  not being at theta_B = 0
   *  Is also used if a weight is added to change center of mass
   */

  theta_B_zero_prev = theta_B_zero;
  theta_B_zero = theta_B_zero - 0.6 * theta_B_error;
  theta_B_zero = (1 - 0.995) * theta_B_zero + 0.995 * theta_B_zero_prev;

  /*
   * /////// End /////////////
   */  
}


void getIMUValues(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
   
  angle_x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  angle_y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
}

void getPWM(){
  // Channel A
  // Calculate Control Output
  A_set_speed = (Kp1_A*theta_A_error - Kp2_A*dTheta_A - Kp3_A*A_speed);

  //Add speed to compensate for friction
  if(A_set_speed > 20){
    A_set_speed += 30;
  } else if (A_set_speed < -20){
    A_set_speed -= 30;
  }

  // Constrain to map to PWM min/max
  A_set_PWM = round(constrain(A_set_speed, -255, 255));

  // Set PWM = 0 for values that are too low
  if(A_set_PWM < 50 && A_set_PWM > -50){
    A_set_PWM = 0;
  }else{
    A_set_PWM = (int) A_set_PWM;
  }

  // Channel B
  //Calculate control output
  B_set_speed = (Kp1_B*theta_B_error - Kp2_B*dTheta_B -Kp3_B*B_speed);

  //Add speed to compensate for friction
  if(B_set_speed > 20){
    B_set_speed += 30;
  } else if (B_set_speed < -20){
    B_set_speed -= 30;
  }
  
  //Constrain to map tp PWM min/max
  B_set_PWM = round(constrain(B_set_speed, -255, 255));

  //Set PWM = 0 for values that are too low
  if(B_set_PWM < 50 && B_set_PWM > -50){
    B_set_PWM = 0;
  }else{
    B_set_PWM = (int) B_set_PWM;
  }
}

////////////////////////////////////////////
//    Set Motor output to desired value   //
////////////////////////////////////////////

void setMotorSpeeds(){
  //Set PWM for Motor A
  if(A_set_PWM != A_set_PWM_prev){
    if(A_set_PWM>0){
      digitalWrite(Ain1, LOW);
      digitalWrite(Ain2, HIGH);
      A_dir_set=1;
      analogWrite(PWMA, A_set_PWM);
    }else if(A_set_PWM<0){
      digitalWrite(Ain1, HIGH);
      digitalWrite(Ain2, LOW);
      A_dir_set=-1;
      analogWrite(PWMA, (abs(A_set_PWM)));
    }else{
      digitalWrite(Ain1, LOW);
      digitalWrite(Ain2, HIGH);
      analogWrite(PWMA, 0);
    }
  }

  //Set PWM for Motor B
  if(B_set_PWM != B_set_PWM_prev){
    if(B_set_PWM>0){
      digitalWrite(Bin1, LOW);
      digitalWrite(Bin2, HIGH);
      B_dir_set=1;
      analogWrite(PWMB, B_set_PWM);
    }else if(B_set_PWM<0){
      digitalWrite(Bin1, HIGH);
      digitalWrite(Bin2, LOW);
      B_dir_set=-1;
      analogWrite(PWMB, (abs(B_set_PWM)));
    }else{
      digitalWrite(Bin1, LOW);
      digitalWrite(Bin2, HIGH);
      analogWrite(PWMB, 0);
    }
  }

  // Save last PWM values
  A_set_PWM_prev = A_set_PWM;
  B_set_PWM_prev = B_set_PWM;
}

void printSerial(){  
  package = micros(); 
  package += "\t";
  package += A_speed;
  package += "\t";
  package += B_speed;
  package += "\t";
  package += theta_A;
  package += "\t";
  package += theta_B;
  package += "\t";
  package += dTheta_A;
  package += "\t";
  package += dTheta_B;
  package += "\t";
  /*
  package += A_set_speed;
  package += "\t";
  package += B_set_speed;
  package += "\t";
  package += A_set_PWM;
  package += "\t";
  package += B_set_PWM;
  package += "\t";
  */
  package += theta_A_zero;
  package += "\t";
  package += theta_B_zero;
  /*
  package += "\t";
  package += A_time_diff;
  package += "\t";
  package += B_time_diff;
  */
  Serial.println(package);
}
