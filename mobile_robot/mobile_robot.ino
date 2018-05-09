
#define R         (float) 0.032
#define L         (float) 0.135
#define FORWARD           1
#define BACKWARD          0
#define rIn1              4
#define rIn2              5
#define lIn1              5
#define lIn2              7
#define rEnable           10
#define lEnable           11
#define rEncoder          2
#define lEncoder          3
#define lDcGain (float) 38.35
#define rDcGain (float) 40.5

//initializing global variables

volatile float rMotorTicks = 0;
volatile float lMotorTicks = 0;
float dR=0.0, dL=0.0, dC=0.0;
float x=0.0, y=0.0, theta=0.0;
float rMotorSpeed = 0, lMotorSpeed = 0;
float v = 0, w = 0, k11 =2, k12 = 2, k13 = 0, k21 = 0, k22 = 0, k23 = 5, vR = 0,  vL = 0, eX = 0, eY = 0 , eTheta = 0;
float xGoal= 1 , yGoal = 0, thetaGoal = 0;
int rDir=FORWARD, lDir=FORWARD;
float vmr = 0, vml = 0, Vmr = 0, Vml = 0 ;
float lVolt,rVolt;
float lSpeed =0.0, rSpeed=0.0;
float lControlAction=0.0,rControlAction=0.0;
void setup() {
  // put your setup code here, to run once:
  pinMode(rIn1, OUTPUT);
  pinMode(rIn2, OUTPUT);
  pinMode(lIn1, OUTPUT);
  pinMode(lIn2, OUTPUT);
  pinMode(rEnable, OUTPUT);
  pinMode(lEnable, OUTPUT);
  pinMode(rEncoder, INPUT);
  digitalWrite(rEncoder, HIGH);       
  pinMode(lEncoder, INPUT);
  digitalWrite(lEncoder, HIGH); 
  attachInterrupt(digitalPinToInterrupt(rEncoder), doRightEncoder, RISING); 
  attachInterrupt(digitalPinToInterrupt(lEncoder), doLeftEncoder, RISING);
  Serial.begin(9600);
  
  delay(1500);
}

void rightWheel(int rspeed){
  
   switch(rDir){
    
    case FORWARD:
      digitalWrite(rIn1, LOW);
      digitalWrite(rIn2, HIGH);
      break;

    case BACKWARD:
      digitalWrite(rIn1, HIGH);
      digitalWrite(rIn2, LOW);
      break;
      
    }
  analogWrite(rEnable, rspeed);
}

void leftWheel(int lSpeed){
  switch(lDir){
    
    case FORWARD:
      digitalWrite(lIn1, LOW);
      digitalWrite(lIn2, HIGH);
      break;

    case BACKWARD:
      digitalWrite(lIn1, HIGH);
      digitalWrite(lIn2, LOW);
      break;
    }
    
  analogWrite(lEnable, lSpeed);
}


void lApplyControlAction(float controlAction){
  
    
  if(controlAction<0){
    
     if(controlAction < -255.0)
        lControlAction= -255.0;
        
    lDir=BACKWARD;
    leftWheel( abs( lControlAction) );
    
    }else{

      if(controlAction > 255.0)
        lControlAction= 255.0;
      
      lDir=FORWARD;
      leftWheel( abs( lControlAction) );
      
    }

     if(controlAction > 255.0)
    lControlAction= 255.0;
    
}


void rApplyControlAction(float controlAction){

  

  if(controlAction<0){
    
    if(controlAction < -255.0)
        rControlAction= -255.0;
        
    rDir=BACKWARD;
    rightWheel( abs( rControlAction) );
    
    }else{
      
      if(controlAction > 255.0)
        rControlAction= 255.0;
        
      rDir=FORWARD;
      rightWheel( abs( rControlAction) );
      
    }
    
}

 



void actualPos(){

  // Updating robot's pose
  x = x + (dC*cos(theta));
  y = y + (dC*sin(theta));
  theta = theta+ ( ( dR - dL ) / L );
  
  // Mapping theta from [-PI,PI]
  theta = atan2( sin( theta ), cos( theta ) );
  
}

void calcErrors(){
  
  float d1 = 0, d2 = 0, d3 = 0;
  d1 = xGoal - x;
  d2 = yGoal - y;
  d3 = thetaGoal - theta; 
  eX =    (cos(theta)*d1) + (sin(theta)*d2);
  eY = -1*(sin(theta)*d2) + (cos(theta)*d2);
  eTheta = (d3);
}


float gainMatrix(){
  
  v = (k11 * eX) + (k12 * eY )+ (k13 * eTheta);
  w = (k21 * eX) + (k22 * eY )+ (k23 * eTheta);
}

float kinematics(){
  
  vR = v + ( w * L/2);
  vL = v - ( w * L/2);
}
void loop() {
  
  /* Gain matrix controller */ 
  
  //get error constants 
    calcErrors();


  //get design parameters (v,w)
    gainMatrix();
    
    if(w>(PI/4))
      w=PI/4;
      
    if(w<(-PI/4))
      w=-PI/4;
  // get vR, vL from the kinematics
    kinematics();
   
    // wanted right wheel speed in rad/s
    rSpeed = vR / R;
    // wanted right wheel speed in RPM
    rSpeed *= 30.0/PI;
     
    // wanted left wheel speed in rad/s
    lSpeed = vL / R;
    
    // wanted right wheel speed in RPM
    lSpeed *= 30.0/PI;
    

  //lMotorSpeed = ((lMotorTicks * 60) / 8.0)/0.05;      //RPM Feedback

  // Calculating desired left wheel voltage 
  lVolt = ( lSpeed / lDcGain ) - 0.1;
  
  // Mapping control action from (0~5.2)volt to (0~255)
  lControlAction=map(lVolt,0,5.2,0,255);
  //Applying control action to left wheel
  lApplyControlAction(lControlAction);

  //rMotorSpeed = (((rMotorTicks * 60) / 8.0)/0.05); //RPM Feedback

  // Calculating desired right wheel voltage
  rVolt = ( rSpeed / rDcGain ) + 0.74;
  // Mapping control action from (0~5.2)volt to (0~255)
  rControlAction=map(rVolt,0,5.2,0,255);
  //Applying control action to right wheel
  rApplyControlAction(rControlAction);

  // Calculating current pose of the robot in (m)
  dR=(rMotorTicks/8.0)*(2*PI*R);        
  dL=(lMotorTicks/8.0)*(2*PI*R);
  dC=(dR+dL)/2;
  actualPos();
  rMotorTicks = 0;
  lMotorTicks = 0;
  
  Serial.print(x);
  Serial.print(" , ");
  Serial.println(y);
  //Serial.print(rMotorSpeed);
  //Serial.print(" , ");
  //Serial.println(lMotorSpeed);
  //Serial.print(" , ");
  //Serial.println(thetaNew);
  //Serial.print(rControlAction);
  //Serial.print(" , ");
  //Serial.println(lControlAction);
  
  
  delay(50);

}

// Calculate R motor ticks
void doRightEncoder(){
  
  if(rDir==FORWARD)
    rMotorTicks++;
  else
    rMotorTicks--;
    
}

// Calculate L motor ticks
void doLeftEncoder(){
  
  if(lDir==FORWARD)
    lMotorTicks++;
  else
    lMotorTicks--;
    
}
