
#define R         (float) 0.032
#define L         (float) 0.072
#define FORWARD           1
#define BACKWARD          0
int rSet1 = 4;
int rSet2 = 5;
int lSet1 = 6;
int lSet2 = 7;
int rEnable = 10;
int lEnable = 11;
int rEncoder = 2;
int lEncoder = 3;
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
float v = 0, w = 0, k11 =1.1, k12 = 2, k13 = 0, k21 = 0, k22 = 0, k23 = 5, vr = 0,  vl = 0, e_x = 0, e_y = 0 , e_theta = 0;
float x_g = 1 , y_g = 1, theta_g = 0;
float vmr = 0, vml = 0, Vmr = 0, Vml = 0, rDcGain = 0, lDcGain = 0, gDcGain = 0 , l = 7.2, x_r = 0, y_r = 0, theta_r = 0;
float lVolt,rVolt;
float Lerror,Rerror;
float lspeed =0;
float rspeed=0;
float lControlActionVolt,rControlActionVolt,left,right,lControlAction=0,rControlAction=0;
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
  
  delay(1500);
}

void rightWheel(int rmos,int dir){
   switch(dir){
    
    case FORWARD:
      digitalWrite(rSet1, LOW);
      digitalWrite(rSet2, HIGH);
      break;

    case BACKWARD:
      digitalWrite(rSet1, HIGH);
      digitalWrite(rSet2, LOW);
      break;
      
    }
  analogWrite(rEnable, rmos);
}

void lApplyControlAction(float controlAction){
  
  if(controlAction > 255.0)
    lControlAction=255.0;
    
  if(controlAction<0)
    leftWheel(abs(lControlAction),BACKWARD);
  else
    leftWheel(abs(lControlAction),FORWARD);
}


void rApplyControlAction(float controlAction){
  
  if(controlAction > 255.0)
    rControlAction=255.0;
    
  if(controlAction<0)
    rightWheel(abs(rControlAction),BACKWARD);
  else
    rightWheel(abs(rControlAction),FORWARD);
}

 
void leftWheel(int lmos,int dir){
  switch(dir){
    
    case FORWARD:
      digitalWrite(lSet1, LOW);
      digitalWrite(lSet2, HIGH);
      break;

    case BACKWARD:
      digitalWrite(lSet1, HIGH);
      digitalWrite(lSet2, LOW);
      break;
    }
    
  analogWrite(lEnable, lmos);
}

//calculate R motor ticks
void doRightEncoder(){
  rMotorTicks++;
}

//calculate L motor ticks
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
// calcualte the with respect to the robot frame 

void calcErrors(){
  float d1 = 0, d2 = 0, d3 = 0;
  d1 = x_g - xNew;
  d2 = y_g - yNew;
  d3 = theta_g - thetaNew; 
  e_x = (cos(thetaNew)*d1)  + (sin(thetaNew)*d2);
  e_y = -1*(sin(thetaNew)*d2) + (cos(thetaNew)*d2);
  e_theta = (d3);
}


float gainMatrix(){
  v = (k11 * e_x) + (k12 * e_y )+ (k13 * e_theta);
  w = (k21 * e_x) + (k22 * e_y )+ (k23 * e_theta);
}

float kinematics(){
  vr = v + (w*L/2);
  vl = v - (w*L/2);
}
void loop() {
  // get the actual pose by the odemetery 
  
  //Gain matrix controller 
  //get error constants 
    calcErrors();

//  if (!(abs(e_x) >.01 ) && !(abs(e_y) > .01) && !(abs(e_theta) > .01)){ 
    //get v,w
    gainMatrix();
  // get vr, vl by the kinematics
    kinematics();

  // rad/s
    rspeed = vr / R;
    // RPM
    //rspeed *= 30.0/PI;
    lspeed = vl / R;
    //lspeed *= 30.0/PI;
    

  //lMotorSpeed = ((lMotorTicks * 60) / 8.0)/0.05;      //RPM Feedback
  lVolt=(lspeed/38.35)-0.1;
  lControlAction=map(lVolt,0,5.2,0,255);
  lApplyControlAction(lVolt);
 

  //rMotorSpeed = (((rMotorTicks * 60) / 8.0)/0.05); //RPM Feedback
  rVolt=(rspeed/40.5)+0.74;
  rControlAction=map(rVolt,0,5.2,0,255);
  rApplyControlAction(rVolt);
  
  Dr=(rMotorTicks/8.0)*(2*PI*R);        // m
  Dl=(lMotorTicks/8.0)*(2*PI*R);
  Dc=(Dr+Dl)/2;
  actualPos();
  
  
  //Serial.print(lspeed);
  //Serial.print(" , ");
  //Serial.println(rspeed);
  //Serial.print(rMotorSpeed);
  //Serial.print(" , ");
  //Serial.println(lMotorSpeed);
  //Serial.print(" , ");
  //Serial.println(thetaNew);
  //Serial.print(rControlAction);
  //Serial.print(" , ");
  //Serial.println(lControlAction);
  
  rMotorTicks = 0;
  lMotorTicks = 0;
  
  delay(50);
  //}
//}

  
}
