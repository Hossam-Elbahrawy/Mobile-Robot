int rSet1 = 4;
int rSet2 = 5;
int lSet1 = 6;
int lSet2 = 7;
int rEnable = 3;
int lEnable = 11;
int rEncoder = 12;
int lEncoder = 13;
volatile unsigned int rMotorTicks = 0;
volatile unsigned int lMotorTicks = 0;

//initializing variables
int rMotorSpeed = 0, lMotorSpeed = 0;

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
  attachInterrupt(digitalPinToInterrupt(rEncoder), doRightEncoder, HIGH); 
  attachInterrupt(digitalPinToInterrupt(lEncoder), doLeftEncoder, HIGH);
  Serial.begin(9600);

}

void rightWheel(int rmos){
  digitalWrite(rSet1, LOW);
  digitalWrite(rSet2, HIGH);
  analogeWrite(rEnable, rmos);
}

void leftWheel(int lmos){
  digitalWrite(lSet1, LOW);
  digitalWrite(lSet2, HIGH);
  analogeWrite(lEnable, lmos);
}
void doRightEncoder(){
  rMotorTicks++;
}
void doLeftEncoder(){
  lMotorTicks++;
}
void loop() {
  // put your +main code here, to run repeatedly:
  //suppose we run in forward motion 
  rMotorSpeed = rMotorTicks / 8;
  lMotorSpeed = lMotorTicks / 8;
  for (int i = 0 ; i<= 255 ; i+=10){
  leftWheel(i);
  Serial.printIn(" V ,Left  Mapped speed");
  Serial.print(i);
  Serial.print(lMotorSpeed);
  rightWheel(i);
  Serial.printIn(" V ,Right Mapped speed");
  Serial.print(i);
  Serial.print(rMotorSpeed);
  }


  Serial.println("");

}
