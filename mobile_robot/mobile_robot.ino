int rSet1 = 4;
int rSet2 = 5;
int lSet1 = 6;
int lSet2 = 7;
int rEnable = 3;
int lEnable = 11;

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
  Serial.begin(9600);

}

void rightWheel(rmos){
  digitalWrite(rset1, LOW);
  digitalWrite(rset2, HIGH);
  analogeWrite(rEnable, rmos);
}

void leftWheel(lmos){
  digitalWrite(lset1, LOW);
  digitalWrite(lset2, HIGH);
  analogeWrite(lEnable, lmos);
}
void loop() {
  // put your main code here, to run repeatedly:
  //suppose we run in forward motion 
  rMotorSpeed = 70;
  lMotorSpeed = 70;
  leftWheel(rMotorSpeed);
  rightWheel(lMotorSpeed);
  Serial.print(leftWheel)
  Serial.print(rightWheel)
  Serial.println("");

}
