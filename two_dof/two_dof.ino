#include <Servo.h>

static int pinA = 2; // Our first hardware interrupt pin for the encoder is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin for the encoder is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile int encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte readingA = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
volatile byte readingB = 0;
float gamma = 0.85;
float alpha = 0.1;   
float epsilon;

Servo elbowservo;
Servo wristservo;

//Servo1 parameters
//This servo moves up with increased angle
const int numTheta1States = 4;
float theta1InitialAngle = 0.0;
float theta1Max = 90.0;                          //Can't exceed 110 or else arm will hit bar   //Need theta1Max and theta1Min between 0 and 180 
float theta1Min = 0.0;
float deltaTheta1 = (theta1Max - theta1Min)/(float(numTheta1States)-1.0);    //The change in servo1's angle when an action is performed on it
int s1 = int((theta1InitialAngle - theta1Min)/deltaTheta1);                  //This is an integer between zero and numTheta1States-1 used to index the state number of servo1
float delayTime1 = 4.5*deltaTheta1;                                 //The time in ms for the servo to move deltaTheta1

//Servo2 parameters
//This servo moves down with increased angle
const int numTheta2States = 4;
float theta2InitialAngle = 115.0;                
float theta2Max = 180.0;                        //Need theta2Max and theta2Min between 0 and 180 
float theta2Min = 115.0;
float deltaTheta2 = (theta2Max - theta2Min)/(float(numTheta2States)-1.0);    //The change in servo2's angle when an action is performed on it
int s2 = int((theta2InitialAngle - theta2Min)/deltaTheta2);                  //This is an integer between zero and numTheta2States-1 used to index the state number of servo2
float delayTime2 = 4.5*deltaTheta2;;

float finalAngle1;
float finalAngle2;

//Initialize Q to zeros
const int numStates = numTheta1States*numTheta2States;
const int numActions = 4;
float Q[numStates][numActions];

//Initialize the state number. The state number is calculated using the theta1 state number and 
//the theta2 state number.  This is the row index of the state in the matrix Q. Starts indexing at 0.
int s = int(s1*numTheta2States + s2);
int sPrime = s;

//Initialize vars for getDeltaDistanceRolled()
float distanceNew = 0.0;
float distanceOld = 0.0;
float deltaDistance = 0.0;

//These get used in the main loop
float r = 0.0;
float lookAheadValue = 0.0;
float sample = 0.0;
int a = 0;

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  elbowservo.attach(9);
  wristservo.attach(10); 
  pinMode(pinA, INPUT); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below) 
  wristservo.write(theta1InitialAngle);
  elbowservo.write(theta2InitialAngle);
}

void PinA(){
  cli(); //stop interrupts happening before we read pin values
  readingA = digitalRead(pinA); //read all eight pin values then strip away all but pinA and pinB's values
  readingB = digitalRead(pinB);
  if (readingA == HIGH && readingB == HIGH && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (readingA == HIGH) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  readingA = digitalRead(pinA); //read all eight pin values then strip away all but pinA and pinB's values
  readingB = digitalRead(pinB);
  if (readingA == HIGH && readingB == HIGH && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (readingB == HIGH) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

//Returns an action 0, 1, 2, or 3
int getAction(){
  int action;
  float valMax = -10000000.0;
  float val;
  int aMax;
  float randVal;
  int allowedActions[4] = {-1, -1, -1, -1};  //-1 if action of the index takes you outside the state space.  +1 otherwise
  boolean randomActionFound = false;
  
  //find the optimal action.  Exclude actions that take you outside the allowed-state space.
  if((s1 + 1) != numTheta1States){
    allowedActions[0] = 1;
    val = Q[s][0];
    if(val > valMax){
      valMax = val;
      aMax = 0;
    }
  }
  if(s1 != 0){
    allowedActions[1] = 1;
    val = Q[s][1];
    if(val > valMax){
      valMax = val;
      aMax = 1;
    }
  }
  if((s2 + 1) != numTheta2States){
    allowedActions[2] = 1;
    val = Q[s][2];
    if(val > valMax){
      valMax = val;
      aMax = 2;
    }
  }
  if(s2 != 0){
    allowedActions[3] = 1;
    val = Q[s][3];
    if(val > valMax){
      valMax = val;
      aMax = 3;
    }
  }
  
  //implement epsilon greedy
  randVal = float(random(0,101));
  if(randVal < (1.0-epsilon)*100.0){    //choose the optimal action with probability 1-epsilon
    action = aMax;
  }else{
    while(!randomActionFound){
      action = int(random(0,4));        //otherwise pick an action between 0 and 3 randomly (inclusive), but don't use actions that take you outside the state-space
      if(allowedActions[action] == 1){
        randomActionFound = true;
      }
    }
  }
    
  return(action);
}

//Given a and the global(s) find the next state.  Also keep track of the individual joint indexes s1 and s2.
void setSPrime(int action){
  
  if (action == 0){
    //joint1++
    sPrime = s + numTheta2States;
    s1++;
  }else if (action == 1){
    //joint1--
    sPrime = s - numTheta2States;
    s1--;
  }else if (action == 2){
    //joint2++
    sPrime = s + 1;
    s2++;
  }else{
    //joint2--
    sPrime = s - 1;
    s2--;
  }
}

//Update the position of the servos (this is the physical state transition command)
void setPhysicalState(int action){
 
  
  if (action == 0){
    finalAngle1 = theta1InitialAngle + deltaTheta1;
    wristservo.write(finalAngle1);
    theta1InitialAngle = finalAngle1;
    delay(delayTime1);
  }else if (action == 1){
    finalAngle1 = theta1InitialAngle - deltaTheta1;
    wristservo.write(finalAngle1);
    theta1InitialAngle = finalAngle1;
    delay(delayTime1);
  }else if (action == 2){
    finalAngle2 = theta2InitialAngle + deltaTheta2;
    elbowservo.write(finalAngle2);
    theta2InitialAngle = finalAngle2;
    delay(delayTime2);
  }else{
    finalAngle2 = theta2InitialAngle - deltaTheta2;
    elbowservo.write(finalAngle2);
    theta2InitialAngle = finalAngle2;
    delay(delayTime2);
  }
}

//Get max over a' of Q(s',a'), but be careful not to look at actions which take the agent outside of the allowed state space
float getLookAhead(){
  float valMax = -100000.0;
  float val;
  
  if((s1 + 1) != numTheta1States){
    val = Q[sPrime][0];
    if(val > valMax){
      valMax = val;
    }
  }
  if(s1 != 0){
    val = Q[sPrime][1];
    if(val > valMax){
      valMax = val;
    }
  }
  if((s2 + 1) != numTheta2States){
    val = Q[sPrime][2];
    if(val > valMax){
      valMax = val;
    }
  }
  if(s2 != 0){
    val = Q[sPrime][3];
    if(val > valMax){
      valMax = val;
    }
  }
  
  return valMax;
}

const int rollDelay = 200;                   //allow time for the agent to roll after it sets its physical state
const float explorationMinutes = 1;        //the desired exploration time in minutes 
const float explorationConst = (explorationMinutes*60.0)/((float(rollDelay))/1000.0);  //this is the approximate exploration time in units of number of times through the loop

int t = 0;
void loop(){
  t++;
  epsilon = exp(-float(t)/explorationConst);
  a = getAction();           //a is beween 0 and 3
  setSPrime(a);              //this also updates s1 and s2.
  encoderPos = 0;
  setPhysicalState(a);
  delay(rollDelay);                      //put a delay after the physical action occurs so the agent has time to move/roll before measuring the new position (before calling getDeltaDistanceRolled)
  r = encoderPos;
  if(r > 0)
    r = exp(r);
  else
  
  Serial.print(r);
  Serial.print(" ");
  Serial.println(t);
  lookAheadValue = getLookAhead();
  sample = r + gamma*lookAheadValue;
  Q[s][a] = Q[s][a] + alpha*(sample - Q[s][a]);
  s = sPrime;  
}

