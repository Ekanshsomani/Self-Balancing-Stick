
/*Here's how the code works:
 * Measure the angle (or angular velocity)
 * Apply PID on that which gives us the required Error Correction
 * According increase it in respective motor
 */ 
#include <Wire.h>

#define PWMA A0
#define in2 5
#define in1 6
#define PWMB A1
#define bin2 7
#define bin1 8

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ;

int minVal=265;
int maxVal=402;

double x, y, z, xprev, yprev, zprev;

int xspeed, yspeed;

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
  xprev = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  yprev = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  zprev = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  
  Wire.endTransmission(true);
}

void loop() {
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
   
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);

  xspeed = xspeed+10;
  yspeed = yspeed+10;
  
  if(x<0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else{
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
  }
  
  if((x-180)*(xprev-180)<0){
    xspeed=100;
  }
  if((y-180)*(yprev-180)<0){
    yspeed = 100;
  }

  if(y<0){
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
  }
  else{
    digitalWrite(bin2, HIGH);
    digitalWrite(bin1, LOW);
  }

  analogWrite(PWMA, xspeed);
  analogWrite(PWMB, yspeed);
  
  xprev = x;
  yprev = y;
}
