
#define R 3.2
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


//initializing variables
float rMotorSpeed = 0, lMotorSpeed = 0;
int i =0;
float v = 0, w = 0, k11 = 0, k12 = 0, k13 = 0, k21 = 0, k22 = 0, k23 = 0, vr = 0,  vl = 0, e_x = 0, e_y = 0 , e_theta = 0;
float x_g = 5 , y_g = 5, theta_g = 90;
float vmr = 0, vml = 0, Vmr = 0, Vml = 0, rDcGain = 0, lDcGain = 0, gDcGain = 0 , l = 7.2, x_r = 0, y_r = 0, theta_r = 0;
float lVolt,rVolt;
float Lerror,Rerror;
int lspeed =0;
int rspeed=0;
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
//right wheel movement 
void rightWheel(int rmos){
  digitalWrite(rSet1, LOW);
  digitalWrite(rSet2, HIGH);
  analogWrite(rEnable, rmos);
}
//left wheel movement
void leftWheel(int lmos){
  digitalWrite(lSet1, LOW);
  digitalWrite(lSet2, HIGH);
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
// calcualte the with respect to the robot frame 

void calcErrors( float x_r  , float y_r , float theta_r ){
  float d1 = 0, d2 = 0, d3 = 0;
  d1 = x_g - x_r;
  d2 = y_g - y_r;
  d3 = theta_g - theta_r; 
  e_x = (cos(theta_r)*d1)  + (sin(theta_r)*d2);
  e_y = -1*(sin(theta_r)*d2) + (cos(theta_r)*d2);
  e_theta = (d3);
}


float gainMatrix(){
  v = (k11 * e_x) + (k12 * e_y )+ (k13 * e_theta);
  w = (k21 * e_x) + (k22 * e_y )+ (k23 * e_theta);
}

float kinematics(){
  vr = v + (w*l/2);
  vl = v - (w*l/2);
}
void loop() {
  // get the actual pose by the odemetery 
  
  // Gain matrix controller 
    //get error constants 
    calcErrors(x_r, y_r , theta_r);

  if (!(abs(e_x) >.01 ) && !(abs(e_y) > .01) && !(abs(e_theta) > .01)){ 
    //get v,w
    gainMatrix();
  // get vr, vl by the kinematics
    kinematics(v, w);

  // Rpm
    rspeed = vr / R;
    lspeed = vl / R;

  // get the required voltage 
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
  
  Serial.print(lMotorSpeed);
  Serial.print(" _ ");
  Serial.println(rMotorSpeed);
 
  
  //rightWheel(i);
  lMotorTicks=0;
  rMotorTicks=0;
  delay(250);
  //}
  }

  
}
