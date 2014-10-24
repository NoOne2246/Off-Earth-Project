/* 
  Description:
  This program was written based off the system analysis written by the
  two other members of my team.
  The program is a fully automated system with a complete set of coded
  instructions for the drill.
  This version however will have some lines of code that are specific for
  debugging which may or may not be found in the final piece of code.
  Version 1.3 is a code for which the defined values may be wrong as we
  have not done testing and do not have the exact values for which the
  vehicle can run at, the code is purely theoretical at the moment.
  
  Newest version of this code is always stored on Github in my account NoOne2246
  under Off-Earth-Project as the file Mining.ino
  https://github.com/NoOne2246/Off-Earth-Project/blob/master/Mining.ino
  
  Author:
  Christopher Chun-Hung Ho
  z5019205
  
  History:
  
  v1.6  23/10/14Removed Locate function, added error correction to detect function
                also added the new rotation system
  v1.5	13/9/14 Changed the movement to another function of its own
                fixed up Ping control with the ultrasound library
  v1.4  10/9/14 added manual control to the code.
  v1.3  5/9/14  initial implementation
		changed ClampControl to boolean
  v1.2  3/9/14  changed the format of the layout to Arduino style
  v1.1  28/8/14	outline the main parts
  v1.0  21/8/14 Write out the basics of the code
*/

//-----------------------------------------------------------------------------------------------------
//Include
//-----------------------------------------------------------------------------------------------------
#include <NewPing.h>
/*http://playground.arduino.cc/Code/NewPing
This is a ping library written by Tim Eckel to use the
HC-S04 ultrasound sensor. The library can return a median
of pings as well as time in microseconds*/

//-----------------------------------------------------------------------------------------------------
//Define
//-----------------------------------------------------------------------------------------------------
//Time delay
#define ROTA_TIME 700              //time to rotate ball
#define READ_DELAY 100             //time between read of analogue pins
#define DRILL_DELAY 100            //time delay for drill to pick up speed
#define PAUSE 50                   //pause for motor to make sure it stops movement

//Ball location
#define DISPLACEMENT 10.0          //displacement of sensor and ball (mm)
#define SPEED 150.0                //define speed of vehicle as mm/ms

//Power Level definition
#define VFS 700                    //voltage minimum recieve to stop forward
#define VBS 400                    //voltage minimum recieve to stop backward movement
                                   //this voltage will also stop clamping
                                   
//rotation
#define ROTATION 100               // mill seconds to turn one degree

//Constant
#define SOS 0.34029                //Speed of Sound in mm/us

//temporary
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
#define STOP 4

//-----------------------------------------------------------------------------------------------------
//Const pins
//-----------------------------------------------------------------------------------------------------

/*Interrupt is on pin 2, this pin is referred to as interupt 0 and will not be
  used within the control of the vehicle as it is an interrupt pin to shutdown
  the machine after it has been initiated.
*/
//drill pin
const int driPin = 9;          //drill pin define
const int dForPin = 8;         //move drill forward
const int dBakPin = 7;         //move drill backward

//clamp pin
const int cloPin = 4;          //opens the clamp
const int clcPin = 3;          //closes the clamp
const int rota = 5;           //rotate ball motor

//Distance pin
const int LRec = A2;           //Left US receiver
const int CRec = A3;           //Centre US receiver
const int RRec = A1;           //Right US receiver
const int LTrig = A5;          //Left US trigger
const int CTrig = 6;          //Centre US trigger
const int RTrig = A4;          //Right US trigger

//Motor Pins
const int motPin1 = 11;        //motor pin 1 for logic gate
const int motPin2 = 10;        //motor pin 2 for logic gate
const int motEnable = 12;      //enable both motors.

//Analogue Pins
const int FBStp = 0;           //forward backward check

//manual overide
const int Override = 0;
const int On = 2;


//-----------------------------------------------------------------------------------------------------
//Declaring
//-----------------------------------------------------------------------------------------------------
//Variables
int aval;                     //read value of analogue pin
int movTime;                  //time to move.
int state;                    //state of the device

//Pings
NewPing leftPing(LTrig, LRec, 150);
NewPing cenPing(CTrig, CRec, 150);
NewPing rightPing(RTrig, RRec, 150);

//Functions                   Pretty much self explanatory, their use is written in the Functions Section
void clampCtrl(boolean dir, int timer);
void drillCtrl();
void rotateCtrl();
void manualCtrl();
void turnOff();
void locate();
void forward();
void moveCtrl(int dir);
void rotate(int angle);
float Ping(int echo);
float PingC();



//-----------------------------------------------------------------------------------------------------
// main
//-----------------------------------------------------------------------------------------------------
//setup function
void setup(){

  Serial.begin(9600);                              //initialise serial communications
  
  //Set up output pins
  pinMode(driPin, OUTPUT);
  pinMode(dForPin, OUTPUT);
  pinMode(dBakPin, OUTPUT);
  pinMode(cloPin, OUTPUT);
  pinMode(clcPin, OUTPUT);
  pinMode(rota, OUTPUT);
  pinMode(motPin1, OUTPUT);
  pinMode(motPin2, OUTPUT);
  pinMode(motEnable, OUTPUT);
  pinMode(LTrig, OUTPUT);
  pinMode(CTrig, OUTPUT);
  pinMode(RTrig, OUTPUT);
  pinMode(On, INPUT);
  
  //set up input pins
  pinMode(LRec, INPUT);
  pinMode(CRec, INPUT);
  pinMode(RRec, INPUT);
  
  /*
  attachInterrupt(Override, manualCtrl, CHANGE);   //switch to manual control the vehicle
  
  noInterrupts();                                  //disable interrupts as we wait for start signal
  */
  
  Serial.println("Initialisation Complete");
  Serial.println("Hit button to begin");
  
  //Wait for start signal
  while(digitalRead(On)==0){                    //wait until it receives start signal
  }
  attachInterrupt(Override, turnOff, CHANGE);   //switch off vehicle
  
  //interrupts();                                  //enable the interrupt
  
  //locate();                                        //make sure facing the ball
  
  forward();                                       //move to the ball.
  
  //clampCtrl(true, 0);                              //clamp the ball, close clamps
  rotate(90);
}

//loop function
void loop (){                                      //loop has been simplified as it may change to manual control
  drillCtrl();                                     //Run the ball drilling function.
  //clampCtrl(false, 5);                           loosen calmp a bit (maybe).
  //rotateCtrl();                                    //Run ball rotating process.
  //clampCtrl(true, 0);                            retighten the clamp.
  rotate(5);
}



//-----------------------------------------------------------------------------------------------------
// subroutines
//-----------------------------------------------------------------------------------------------------

/*old ping code, rendered redundant after the decidion to use the ultrasound ping library.
float Ping(int echo){
  
  float distance = 0;                       //declare internal variables
  float average = 0;
  float timer[5];
  
  for(int i = 0; i < 5; i++){
    
    digitalWrite(Trigger, LOW);
    delayMicroseconds(2);                   //clean Triggerger signal
    digitalWrite(Trigger, HIGH);            
    delayMicroseconds(10);                  //Triggerger signal for 10us to make sure the device knows to Triggerger
    digitalWrite(Trigger, LOW);             //stop Triggerger
    aval = micros();                        //read time since device began functioning
    
    while(digitalRead(echo)==HIGH){         //pause until no return ping heard
    }
    
    timer[i] = micros() - aval;             //calculate ping difference
    average += timer[i];                    //adds data to a total
  }
  
  average /= 5;                             //averages data
  distance = average * SOS;                 //convert to distance
  
  return distance;                          //return distance in mm
}
*/

void rotate(int angle){
  int time;				   //create time variable
  time = angle*ROTATION;                   //calculate the time to rotate for in ms
  digitalWrite(rota, HIGH);		   //rotate the ball
  delay(time);				   //delay for calculated time
  digitalWrite(rota, LOW);                 //stop rotating ball
}


float PingC(){
  float distance = 0;                       //declare internal variables
  float average = 0;
  int temp, store;
  float timer[10][2];
  
  for(int i = 0; i < 10; i++){
    do
    {
      timer[i][1] = cenPing.ping_median() * SOS;    //find distance using centre sensor
    }while(timer[i][1] < 750.0);		   //loop makes sure values are reasonable
    timer[i][2] = 1;                              //set up all counters to 1
  }
  
  for(int i = 0; i<10; i++){                      //bubble sort numbers from smallest to highest
    for (int j = 0; j<9-i; j++){
      if(timer[j][1]>timer[j+1][1]){
        temp = timer[j][1];
        timer[j][1] = timer[j+1][1];
        timer[j+1][1] = temp;
      }
    }
  }
  
  for(int i = 0; i < 9; i++){                      //count the amount of similar ones within 5mm
    j = 1;
    while(timer[i][2]+5.0 > timer[i+j][2] && j + i < 10){    //make sure not to overflow data.
      timer[i][2]++;
      j++;
    }
  }
  
  temp = 0;
  
  for(int i = 0; i<10; i++){	    	    //loop to find max count
    if(timer[i][2] >= temp){		    //take the value with the most with the largest similar values
      temp = timer[i][2];		    //take the count of the most and store it
      store = i;			    //store that value at which it starts
    }
  }
  
  for(int i = 0; i<temp+1; i++){	    //add all those values starting from the first value to the highest within range
    average += timer[store+i][1];
  }
  
  distance = average / store;		    //take the average of these similar values
  
  return distance;                          //return distance in mm
}
  



void moveCtrl(int dir){
  switch(dir){
    case FORWARD:
	
      digitalWrite(motPin1, HIGH);              //Both motors on
      digitalWrite(motPin2, HIGH);
      break;
  		
    case LEFT:
  		
      digitalWrite(motPin1, LOW);               //Right motor on
      digitalWrite(motPin2, HIGH);
      break;
  		
    case RIGHT:
    
      digitalWrite(motPin1, HIGH);              //left motor on
      digitalWrite(motPin2, LOW);
      break;
  		
    case STOP:
    
      digitalWrite(motPin1, LOW);               //stop both motors
      digitalWrite(motPin2, LOW);
      break;
	  
    default:
      break;
  }
}

void forward(){
  
  //declare local variables
  int timer;                                  //timer that will check how long motors run for
  float distance;
  
  /*motor only works with on and off so it must be able to run the motor so that it stops in time for the ball.
  need to know location of sensor and distance from the edge the ball will be loacted.
  */
  
  distance = PingC();
  timer = (distance - DISPLACEMENT) / SPEED; //calculate time it needs to run at
  
  moveCtrl(FORWARD);               	     //move forward
  delay(timer);                              //delay to stop using precalculated time
  moveCtrl(STOP);                            //Stop vehicle
}

/*
void locate(){
  
  //declare local variables
  float Left = 0;
  float Right = 0;
  
  Left = leftPing.ping_median() * SOS;                                 //read distance
  Right = rightPing.ping_median() * SOS;
  
  //loop until facing the ball properly
  do
  {
    //drive logic 
    if(Left > Right){    
      moveCtrl(RIGHT);                           						//rotate clockwise
    } else {
      moveCtrl(LEFT);                          							//rotate counter clockwise
    }      
    
    Left = leftPing.ping_median() * SOS;                                 //read distance
    Right = rightPing.ping_median() * SOS;
  
  } while(Left > (Right + 5) || Left < (Right - 5));        //variance of +-5mm as the sensors come with an accuracy of 2%.
  
  moveCtrl(STOP);                             							//Stop all movement
}
*/

/*void clampCtrl(boolean dir, int timer){
  
  if (dir == true){                        //decide whether to open or close (1 to close, 0 to open).
    digitalWrite (clcPin, HIGH);           //Start closing clamps
    digitalWrite (clcPin, LOW);            //Stop closing clamps
  } else {
    digitalWrite (cloPin, HIGH);           //Start opening clamps
    
    if (timer > 0){
      delay(timer);                        //movement time given
    } else {
      while (analogRead(FBStp) < VBS){     //check that the switch has been hit
        delay (READ_DELAY);                //time between checking for whether the switch has been hit
      }
    }
    
    digitalWrite (cloPin, LOW);            //Stop opening clamps 
  }
}
*/


void drillCtrl(){
  
  digitalWrite(driPin, HIGH);               //drill on
  delay(DRILL_DELAY);                       //give drill time to pick up speed
  digitalWrite(dForPin, HIGH);              //drill forward
  
  while (analogRead (FBStp) <= VFS){        //wait until the stop switch has been hit
    delay(READ_DELAY);                      //time between checking for whether the switch has been hit
  }
  
  digitalWrite (dForPin, LOW);              //stop forward movement of the drill
  delay(PAUSE);                             //pause for moment
  digitalWrite (dBakPin, HIGH);             //move drill backwards
  aval = analogRead(FBStp);                 //read the value of the voltage
  
  while (aval > VFS || aval < VBS){         //check that the switch has been hit
    delay (READ_DELAY);                     //time between checking for whether the switch has been hit
    aval = analogRead(FBStp);               //read the value of the voltage
  }
  
  digitalWrite (dBakPin, LOW);              //stop backward movement
  digitalWrite (driPin, LOW);               //Turn off drill
}

/*
void rotateCtrl(){
  
  digitalWrite (rota, HIGH);                //Turn the motor to turn the ball on
  delay (ROTA_TIME);                        //rotate the ball for how many milliseconds.
  digitalWrite (rota, LOW);                 //Stop turning the ball
}
*/

/* Remove manual control section as that is causing problems due to Serial usage within the interrupt
//Manual Control of the device
void manualCtrl(){
  char command;
  
  //turn off all running devices
  digitalWrite(driPin, LOW);
  digitalWrite(dForPin, LOW);
  digitalWrite(dBakPin, LOW);
  digitalWrite(cloPin, LOW);
  digitalWrite(clcPin, LOW);
  digitalWrite(rota, LOW);
  digitalWrite(Trigger, LOW);
  moveCtrl(STOP);
  
  noInterrupts();																												 //turn off interrupts
  
  Serial.println("Entered manual control mode");
  
  //load command in
  command = Serial.read();
  
  while(command!='r'){
  
    switch (command){
      case 'w':
        
        Serial.println("Moving Forward");                                //acknowledge command received
        moveCtrl(FORWARD);                                    					 //set motor control
        
        while(Serial.read()!='s'){                                       //wait for stop signal
        }
        
        moveCtrl(STOP);                                    							 //stop motor
        break;
        
      case 'a':
        
        Serial.println("Rotating CounterClockwise");                     //acknowledge command received
        moveCtrl(LEFT);                                      						 //set motor values
        
        while(Serial.read()!='s'){                                       //wait for stop signal
        }
        
        moveCtrl(STOP);                                    							 //stop motor
        break;
        
      case 'd':
      
        Serial.println("Rotating Clockwise");                            //acknowledge command received
        moveCtrl(RIGHT);                                      					 //set motor values
        
        while(Serial.read()!='s'){                                       //wait for stop signal
        }
        
        moveCtrl(STOP);                                    							 //stop motor
        break;
        
      case 'u':
        
        digitalWrite(driPin, HIGH);                                      //turn drill on
        Serial.println("drill turned on");                               //acknowledge command
        break;
        
      case 'i':
        
        digitalWrite(driPin, LOW);                                       //turn drill off
        Serial.println("Drill turned off");                              //acknowledge command
        break;
        
      case 'o':
        
        Serial.println("Drill moving forward");                          //acknowledge command
        digitalWrite(dForPin, HIGH);                                     //move drill forward
        
        while(analogRead(FBStp) <= VFS && Serial.read() !=' s'){         //wait until the stop switch has been hit or command to stop is given
          delay(10);                                                     //delay between reading
        }
        
        digitalWrite(dForPin, LOW);                                      //stop moving forward
        break;
        
      case 'p':
        
        Serial.println("Drill moving backward");                         //acknowledge command
        digitalWrite(dBakPin, HIGH);                                     //move drill back
        aval = analogRead(FBStp);                                        //read value to check whether the drill has moved back enough
        
        while((aval > VFS || aval < VBS) && Serial.read() !=' s'){       //wait until the stop switch has been hit or command to stop is given
          delay(10);                                                     //delay between reading
          aval = analogRead(FBStp);
        }
        
        digitalWrite(dBakPin, LOW);                                      //stop moving backward
        break;
        
      case 'j':
        
        Serial.println("Opening clamps");                                //acknowledge command
        digitalWrite(cloPin, HIGH);                                      //start opening the clamps
        //condition to stop opening
        digitalWrite(cloPin, LOW);                                       //stop opening the clamps
        
        break;
        
      case 'k':
        
        Serial.println("closing clamps");                                //acknowledge command
        digitalWrite(clcPin, HIGH);                                      //start closing the clamps
        //condition to stop clamp closing
        digitalWrite(clcPin, LOW);                                       //stop closing the clamps
        
        break;
        
      case 'l':
        
        Serial.println("Rotating ball");                                 //acknowledge command
        digitalWrite(rota, HIGH);                                        //start rotating the ball
        delay(ROTA_TIME);                                                //delay to rotate the ball
        digitalWrite(rota, LOW);                                         //stop rotating the ball
        
        break;
        
      case 'z':
        
        Serial.println("Are you sure you wish to shutdown? (Y/anything else to cancel)"); //acknowledge input and give options
        
        while(Serial.available() == 0){                                  //wait until receive input
        }
        
        if(Serial.read() == 'Y'){                                        //check for yes to shutdown
          shutdown();
        }
        
        break;
      
      default:
        break;                                                           //do nothing
        
    }
    
    while(Serial.available()==0){                                        //wait for next command
    }
    
    command = Serial.read();                                             //read command
  }
  
  interrupts();
  return;
} 
*/

//reset the machine and stop movement
void turnOff(){
  
  aval = analogRead(FBStp);                 //read the value of the voltage
  
  if(aval > VFS || aval < VBS){             //check if drill is all the way out, if not, pull out
  
    digitalWrite(driPin, HIGH);             //drill on
    delay(100);                             //give drill time to pick up speed
    digitalWrite(dBakPin, HIGH);            //move drill backwards
    aval = analogRead(FBStp);               //read the value of the voltage
    
    while(aval > VFS || aval < VBS){        //check that the switch has been hit
      delay(READ_DELAY);                    //time between checking for whether the switch has been hit
      aval = analogRead(FBStp);             //read the value of the voltage
    }
    
    digitalWrite(dBakPin, LOW);             //stop backward movement
    digitalWrite(driPin, LOW);              //Turn off drill
  }
  
  clampCtrl(false, 0);                      //open clamps
  
  while(0!=1){                              //loops infinitely to stop the machine
  }
}
