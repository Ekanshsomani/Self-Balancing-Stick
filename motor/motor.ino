#define PWMA A0
#define in2 A1
#define in1 A2

int pot, out;

void setup(){
  Serial.begin(9600);
  pinMode(PWMA, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);
}

void loop(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(PWMA, 150);
  delay(2000);
}
