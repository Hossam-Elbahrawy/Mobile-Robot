
#define R         (float) 0.032
#define L         (float) 0.135
#define FORWARD           1
#define BACKWARD          0
#define rIn1              4
#define rIn2              5
#define lIn1              6
#define lIn2              7
#define rEnable           10
#define lEnable           11
#define rEncoder          2
#define lEncoder          3
#define lDcGain           17
#define rDcGain            17
#define PCONTROLLER       2.5
#define goalsNr           3
#define ultraTrigger    12
#define ultraEcho       13
#define ledAlaa         8  

//initializing global variables

volatile float rMotorTicks = 0;
volatile float lMotorTicks = 0;
float dR=0.0, dL=0.0, dC=0.0;
float x=0.0, y=0.0, theta=0.0;
float v = 0.2, w = 0,  vR = 0,  vL = 0, eX = 0, eY = 0 , eTheta = 0;
float rGoals[][goalsNr]={{.25,0},{.75, .25},{1,1}};
float  thetaGoal = 0;
float vmr = 0, vml = 0, Vmr = 0, Vml = 0 ;
float lVolt,rVolt;
float lSpeed =0.0, rSpeed=0.0;
float lControlAction=0.0,rControlAction=0.0;
int i = 0;
long  duration;
float distanceM;
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
  pinMode(ultraTrigger, OUTPUT); 
  pinMode(ultraEcho, INPUT);

  //LED Alaa
  pinMode(ledAlaa, OUTPUT); 
 
}

void rightWheel(int rspeed){
  digitalWrite(rIn1, LOW);
  digitalWrite(rIn2, HIGH);
  analogWrite(rEnable, rspeed);
}

void leftWheel(int lSpeed){
    digitalWrite(lIn1, LOW);
    digitalWrite(lIn2, HIGH);
  analogWrite(lEnable, lSpeed);
}

float omegaSatauration(float w){
  if (w > PI/2) return PI/2;
  if (w < -PI/2) return -PI/2;
  return w;
}
void pidController(){
  eTheta = thetaGoal - theta;
  w = PCONTROLLER * eTheta ;
  w = omegaSatauration(w);
}

void actualPos(){
  // Updating robot's pose
  dR=(rMotorTicks/8.0)*(2*PI*R);        
  dL=(lMotorTicks/8.0)*(2*PI*R);
  dC=(dR+dL)/2;
  x = x + (dC*cos(theta));
  y = y + (dC*sin(theta));
  theta = theta+ ( ( dR - dL ) / L );
  // Mapping theta from [-PI,PI]
  theta = atan2( sin( theta ), cos( theta ) );
}



float kinematics(){
  
  vR = v + ( w * L/2);
  vL = v - ( w * L/2);
}
void loop() {
    
   if (((rGoals[i][0] - x)> 0.1) || (rGoals[i][1] - y)>.1){
    
    digitalWrite(ultraTrigger, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(ultraTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultraTrigger, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(ultraEcho, HIGH);
    // Calculating the distance
    distanceM= (duration*0.034/2);

    //Checking if there is an obstacle 
    if(distanceM <= 0.15){
      digitalWrite(ledAlaa,HIGH);
      leftWheel(180);
      rightWheel(100);
      delay(50);
      }
      digitalWrite(ledAlaa,LOW);
    thetaGoal = atan2(rGoals[i][1]-y, rGoals[i][0]-x);
    
    pidController();
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
    
  // Calculating desired left wheel voltage 
  lVolt = ( lSpeed / lDcGain );

  // Calculating desired right wheel voltage
  rVolt = ( rSpeed / rDcGain );
  
  // Mapping control action from (0~5.2)volt to (0~255)
  lControlAction=map(lVolt,0,5.2,0,255);

  // Mapping control action from (0~5.2)volt to (0~255)
  rControlAction=map(rVolt,0,5.2,0,255);
  //Applying control action to left wheel
  rightWheel(rControlAction-15);
  
  //Applying control action to right wheel
  leftWheel(lControlAction);

  // Calculating current pose of the robot in (m)

  actualPos();
    
  Serial.print(x);
  Serial.print(" , ");
  Serial.println(distanceM);  

    }
    else{
      if (i != (goalsNr-1)){
      rightWheel(0);
      leftWheel(0);
        delay(500);
        i++;
        }
       else{
      rightWheel(0);
      leftWheel(0);
       }
    }
  rMotorTicks = 0;
  lMotorTicks = 0;

  delay(100);
}

// Calculate R motor ticks
void doRightEncoder(){
    rMotorTicks++;    
}

// Calculate L motor ticks
void doLeftEncoder(){
    lMotorTicks++;    
}
