// Constants
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

//Initializing global variables
volatile float rMotorTicks = 0;
volatile float lMotorTicks = 0;
float dR=0.0, dL=0.0, dC=0.0;
float x=0.0, y=0.0, theta=0.0;
float v = 0.2, w = 0,  vR = 0,  vL = 0, eX = 0, eY = 0 , eTheta = 0;
float rGoals[][goalsNr]={{.25,0},{.75, .25},{1,1}};
float thetaGoal = 0;
float vmr = 0, vml = 0, Vmr = 0, Vml = 0 ;
float lVolt,rVolt;
float lSpeed =0.0, rSpeed=0.0;
float lControlAction=0.0,rControlAction=0.0;
int   currentGoal = 0;
long  duration, distanceM;


void setup() {
  // Right wheel direction and enable pins
  pinMode(rIn1, OUTPUT);
  pinMode(rIn2, OUTPUT);
  pinMode(rEnable, OUTPUT);

  // Left wheel direction and enable pins
  pinMode(lIn1, OUTPUT);
  pinMode(lIn2, OUTPUT);
  pinMode(lEnable, OUTPUT);

  // Right and left encoder pins with pull up on each
  pinMode(rEncoder, INPUT);
  digitalWrite(rEncoder, HIGH);       
  pinMode(lEncoder, INPUT);
  digitalWrite(lEncoder, HIGH); 

  // Activating external on rising edge interrupt
  attachInterrupt(digitalPinToInterrupt(rEncoder), doRightEncoder, RISING); 
  attachInterrupt(digitalPinToInterrupt(lEncoder), doLeftEncoder, RISING);

  // Ultrasonic sensor pins 
  pinMode(ultraTrigger, OUTPUT); 
  pinMode(ultraEcho, INPUT);

  // Obstacles LED for Alaa
  pinMode(ledAlaa, OUTPUT);

  // starting serial with 9600 baudrate
  Serial.begin(9600);  
 
}

// Moving right motor forward with a given speed
void rightWheel(int rspeed){
  digitalWrite(rIn1, LOW);
  digitalWrite(rIn2, HIGH);
  analogWrite(rEnable, rspeed);
}

// Moving left motor forward with a given speed
void leftWheel(int lSpeed){
    digitalWrite(lIn1, LOW);
    digitalWrite(lIn2, HIGH);
    analogWrite(lEnable, lSpeed);
}

// Saturating the angular velocity due to hardware limitations
float omegaSatauration(float w){
  if (w > PI/2) 
    return PI/2;
  if (w < -PI/2)
    return -PI/2;
  return w;
}

// Using PID contoller on the error in angular velocity 
void pidController(){
  eTheta = thetaGoal - theta;
  w = PCONTROLLER * eTheta ;
  w = omegaSatauration(w);
}

// Odometry module to update robot's pose
void actualPos(){
  dR=( rMotorTicks / 8.0 ) * ( 2 * PI * R );        
  dL=( lMotorTicks / 8.0 ) * ( 2 * PI * R );
  dC=( dR + dL ) / 2;
  x = x + ( dC * cos ( theta ) );
  y = y + ( dC * sin ( theta ) );
  theta = theta + ( ( dR - dL ) / L );
  // Mapping theta from [-PI,PI]
  theta = atan2( sin( theta ), cos( theta ) );
}

// Mapping (w,v) to (vR,vL) using differential drive model
float kinematics(){
  vR = v + ( w * L/2 );
  vL = v - ( w * L/2 );
}

// All the magic happens here
void loop() {
  
   // If the robot is not at the goal yet
   if (((rGoals[currentGoal][0] - x)> 0.1) || (rGoals[currentGoal][1] - y)>.1){
    
    /* Ultrasonic sensor code */
    
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

    //Turrning right if there is an obstacle
    if(distanceM <= 0.15){
      digitalWrite(ledAlaa,HIGH);
      leftWheel(180);
      rightWheel(100);
      delay(50);
      }
      digitalWrite(ledAlaa,LOW);
      
      /* Ultrasonic sensor code */

    // Updating goal angle  
    thetaGoal = atan2(rGoals[currentGoal][1]-y, rGoals[currentGoal][0]-x);

    // Generating contol action
    pidController();
    
    // Getting vR, vL from the kinematics
    kinematics();
   
    // Desired right wheel speed in (rad/s)
    rSpeed = vR / R;
    
    // Desired right wheel speed in (RPM)
    rSpeed *= 30.0/PI;
     
    // Desired left wheel speed in (rad/s)
    lSpeed = vL / R;
    
    // Desired right wheel speed in (RPM)
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
    Serial.println(y);  

    }
    // The robot reached the goal but not the last goal
    else{
      if (currentGoal != (goalsNr-1)){
        // Stop the robot and move to the next goal
        rightWheel(0);
        leftWheel(0);
        delay(500);
        currentGoal++;
      }
      // The robot reached the last goal
       else{        
        rightWheel(0);
        leftWheel(0);
       }
    }
    
  // Resetting both motor ticks 
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
