#define R 3.2
#define L 6
int rSet1 = 4;
int rSet2 = 5;
int lSet1 = 6;
int lSet2 = 7;
int rEnable = 3;
int lEnable = 11;
int rEncoder = 12;
int lEncoder = 13;
volatile float rMotorTicks = 0;
volatile float lMotorTicks = 0;
float Dr=0.0;
float Dl=0.0;
float Dc=0.0;
float xOld =0.0;
float yOld=0.0;
float thetaOld=0.0;
float xNew,yNew,thetaNew;
//initializing variables
float rMotorSpeed = 0, lMotorSpeed = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(rSet1, OUTPUT);
  pinMode(rSet2, OUTPUT);
  pinMode(lSet1, OUTPUT);
  pinMode(lSet2, OUTPUT);
  pinMode(rEnable, OUTPUT);
  pinMode(lEnable, OUTPUT);
  pinMode(rEncoder, INPUT);
  digitalWrite(rEncoder, HIGH);       // turn on pull-up resistor
  pinMode(lEncoder, INPUT);
  digitalWrite(lEncoder, HIGH); 
  attachInterrupt(digitalPinToInterrupt(rEncoder), doRightEncoder, RISING); 
  attachInterrupt(digitalPinToInterrupt(lEncoder), doLeftEncoder, RISING);
  Serial.begin(9600);
leftWheel(255);
rightWheel(255);
}

void rightWheel(int rmos){
  digitalWrite(rSet1, LOW);
  digitalWrite(rSet2, HIGH);
  analogWrite(rEnable, rmos);
}

void leftWheel(int lmos){
  digitalWrite(lSet1, LOW);
  digitalWrite(lSet2, HIGH);
  analogWrite(lEnable, lmos);
}
void doRightEncoder(){
  rMotorTicks++;
}
void doLeftEncoder(){
  lMotorTicks++;
}
float actualPos(Dr,Dl,Dc){
  xNew=xOld+(Dc*cos(thetaOld));
  yNew=yOld+(Dc*sin(thetaOld);
  thetaNew=thetaOld+((Dr-Dl)/L);
  xOld=xNew;
  yOld=yNew;
  thetaOld=thetaNew;
}
void loop() {
  // put your main code here, to run repeatedly:
  //suppose we run in forward motion 
  
  Serial.print(lMotorTicks);
  Serial.print("_");
  Serial.println(rMotorTicks);
  rMotorSpeed = (rMotorTicks / 8.0)*60;
  lMotorSpeed = (lMotorTicks / 8.0)*60;
  Dr=rMotorSpeed*(2*PI*R);
  Dl=lMotorSpeed*(2*PI*R);
  Dc=(Dr+Dl)/2;
  actualPos(Dr,Dl,Dc);
  rMotorTicks=0;
  lMotorTicks=0;
  /*for (int i = 0 ; i<= 255 ; i+=15){
  leftWheel(i);
  Serial.println(" Vl");
  Serial.println(i);
  Serial.println(lMotorSpeed);
  rightWheel(i);
  Serial.println(" Vr");
  Serial.println(i);
  Serial.println(rMotorSpeed);
  }*/
}
