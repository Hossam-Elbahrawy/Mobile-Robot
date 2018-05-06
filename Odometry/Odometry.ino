#define R 3.2
#define L 7.2
int rSet1 = 6;
int rSet2 = 7;
int lSet1 = 4;
int lSet2 = 5;
int rEnable = 11;
int lEnable = 10;
int rEncoder = 3;
int lEncoder = 2;
volatile float rMotorTicks = 0;
volatile float lMotorTicks = 0;
float Dr=0.0;
float Dl=0.0;
float Dc=0.0;
float xOld =0.0;
float yOld=0.0;
float thetaOld=0;
float xNew,yNew,thetaNew;

//initializing variables
float rMotorSpeed = 0, lMotorSpeed = 0;
int i =0;
int j=0;
float lVolt,rVolt;
float Lerror,Rerror;
int lspeed=180;
int rspeed=180;
float lControlActionVolt,rControlActionVolt,left,right,lControlAction,rControlAction;
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

void actualPos(){
  xNew=xOld+(Dc*cos(thetaOld));
  yNew=yOld+(Dc*sin(thetaOld));
  thetaNew=thetaOld+((Dr-Dl)/L);
  
 // if(thetaNew>PI){thetaNew-=2*PI;}
  //  if(thetaNew<(-PI)){thetaNew+=2*PI;}
  thetaNew=atan2(sin(thetaNew),cos(thetaNew));
  xOld=xNew;
  yOld=yNew;
  thetaOld=thetaNew;
}
void loop() {
  // put your main code here, to run repeatedly:
  //suppose we run in forward motion 
  
//  if (lMotorTicks >= 8){
//     leftWheel(0);
//    }
//  if (rMotorTicks >= 8){
//     rightWheel(0);
//    }
 
  //lVolt=map(i,0,5.2,0,255);
  
  lMotorSpeed = ((lMotorTicks * 60) / 8.0)/0.25; //RPM
  lVolt=(lMotorSpeed/38.35)-0.1;
  Lerror=lspeed-lMotorSpeed;
  lControlActionVolt=Lerror/38.35;
  left=lVolt+lControlActionVolt;
  lControlAction=map(left,0,5.2,0,255);
  leftWheel(lControlAction);

  rMotorSpeed = (((rMotorTicks * 60) / 8.0)/0.25); //RPM
  rVolt=(rMotorSpeed/40.5)+0.74;
  Rerror=rspeed-rMotorSpeed;
  rControlActionVolt=Rerror/40.5;
  right=rVolt+rControlActionVolt;
  rControlAction=map(right,0,5.2,0,255);
  rightWheel(rControlAction);
 
  Dr=(rMotorTicks/8.0)*(2*PI*R);        // cm
  Dl=(lMotorTicks/8.0)*(2*PI*R);
  Dc=(Dr+Dl)/2;
  actualPos();
  //rightWheel(i);
  lMotorTicks=0;
  rMotorTicks=0;
  Serial.print(xNew);
  Serial.print(" , ");
  Serial.print(yNew);
  Serial.print(" , ");
   Serial.println((thetaNew*180)/PI);
  delay(250);
  //}
}
