
#include <QTRSensors.h>  // Pololu QTR Library 

//line sensor defines
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN  // emitter control pin not used.  If added, replace QTR_NO_EMITTER_PIN with pin#

//line sensor declarations
// sensors 0 through 7 are connected to digital pins 2 through 10, respectively (pin 3 is skipped and use for motor control)
QTRSensorsRC qtrrc((unsigned char[]) {2, 4, 5, 6, 7, 8, 9,10},
NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];
unsigned int line_position=0; // value from 0-7000 to indicate position of line between sensor 0 - 7

// motor driver vars
// pwm_a/b sets speed.  Value range is 0-255.  For example, if you set the speed at 100 then 100/255 = 39% duty cycle = slow 
int pwm_a = 3;  //PWM control for motor outputs 1 and 2 is on digital pin 10  (Left motor)
int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11  (Right motor)
int dir_a = 12;  //direction control for motor outputs 1 and 2 is on digital pin 12  (Left motor)
int dir_b = 13;  //direction control for motor outputs 3 and 4 is on digital pin 13  (Right motor)

// motor tuning vars for maze navigating
int calSpeed = 165;   // tune value motors will run while auto calibration sweeping turn (0-255)
int turnSpeed = 200;  // tune value motors will run while turning (0-255)
int turnSpeedSlow = 125;  // tune value motors will run as they slow down from turning cycle to avoid overrun (0-255)
int drivePastDelay = 300; // tune value in mseconds motors will run past intersection to align wheels for turn

// pid loop vars
float error=0;
float lastError=0;
float PV =0 ;
float kp = 0;  // tune value in follow_line() function
//float ki = 0; // ki is not currently used
float kd =0;   // tune value in follow_line() function
int m1Speed=0;
int m2Speed=0;
int motorspeed=0;


// The path variable will store the path that the robot has taken.  It
// is stored as an array of characters, each of which represents the
// turn that should be made at one intersection in the sequence:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)
// You should check to make sure that the path_length of your 
// maze design does not exceed the bounds of the array.
char path[100] = "";
unsigned char path_length = 0; // the length of the path

void setup()
{
  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  analogWrite(pwm_a, 0);  //set both motors to stop at (100/255 = 39)% duty cycle (slow)
  analogWrite(pwm_b, 0);
  
  Serial.begin(9600);   
  
  Serial.println("Calibrating sensor");
  
  // calibrate line sensor, determines min/max range of sensed values for the current course
  delay(2000);
  
  for (int i = 0; i <= 200; i++)  // begin calibration cycle to last about 2.5 seconds (100*25ms/call)
  {
    qtrrc.calibrate();       // reads all sensors with the define set 2500 microseconds (25 milliseconds) for sensor outputs to go low.
    
  }  // end calibration cycle
  
  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  line_position = qtrrc.readLine(sensorValues);
  
 /* while (sensorValues[6] < 200)  // wait for line position to near center
  {
    line_position = qtrrc.readLine(sensorValues);
  }*/
  
  // slow down speed
    
  // print calibration results
  
  // print the calibration minimum values measured when emitters were on
  Serial.println();
  Serial.println("Calibration Complete");
  delay(1000);
  
} // end setup

void loop()
{

  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int line_position = qtrrc.readLine(sensorValues);
  
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.print(line_position); // comment this line out if you are using raw values
  Serial.print("  ");
  Serial.print(sensorValues[0]);
  Serial.print("  ");
  Serial.print(sensorValues[1]);
  Serial.print("  ");
  Serial.print(sensorValues[2]);
  Serial.print("  ");
  Serial.print(sensorValues[3]);
  Serial.print("  ");
  Serial.print(sensorValues[4]);
  Serial.print("  ");
  Serial.print(sensorValues[5]);
  Serial.print("  ");
   Serial.print(sensorValues[6]);
  Serial.print("  ");
  Serial.println(sensorValues[7]);
  
  // begin maze solving
  MazeSolve(); // comment out and run serial monitor to test sensors while manually sweeping across line
  

}  // end main loop



//line following subroutine
// PD Control
void follow_line()  //follow the line
{
  lastError = 0;
  while(1)
  {

  Serial.print(line_position); // comment this line out if you are using raw values
  Serial.print("  ");
  Serial.print(sensorValues[0]);
  Serial.print("  ");
  Serial.print(sensorValues[1]);
  Serial.print("  ");
  Serial.print(sensorValues[2]);
  Serial.print("  ");
  Serial.print(sensorValues[3]);
  Serial.print("  ");
  Serial.print(sensorValues[4]);
  Serial.print("  ");
  Serial.print(sensorValues[5]);
  Serial.print("  ");
  Serial.print(sensorValues[6]);
  Serial.print("  ");
  Serial.println(sensorValues[7]);
    line_position = qtrrc.readLine(sensorValues);
    switch(line_position)
    {
   
   
      default:
        error = (float)line_position - 3500;
   
        // set the motor speed based on proportional and derivative PID terms
        // kp is the a floating-point proportional constant (maybe start with a value around 0.5)
        // kd is the floating-point derivative constant (maybe start with a value around 1)
        // note that when doing PID, it's very important you get your signs right, or else the
        // control loop will be unstable
        kp=.5;
        kd=1;
     
        PV = kp * error + kd * (error - lastError);
        lastError = error;
    
        //this codes limits the PV (motor speed pwm value)  
        // limit PV to 55
        if (PV > 55)
        {
        
          PV = 55;
        }
    
        if (PV < -55)
        {
          PV = -55;
        }
        
        m1Speed = 200 + PV;
        m2Speed = 200 - PV;
       
        //set motor speeds
        digitalWrite(dir_a, LOW);  
        analogWrite(pwm_a, m2Speed);
        digitalWrite(dir_b, LOW);  
        analogWrite(pwm_b, m1Speed);
        break;
    }
  
    // We use the inner six sensors (1 thru 6) to
    // determine if there is a line straight ahead, and the
    // sensors 0 and 7 if the path turns.
    if(sensorValues[0] < 200 && sensorValues[1] < 200 && sensorValues[2] < 200 && sensorValues[3] < 200 && sensorValues[4] < 200 && sensorValues[5] < 200 && sensorValues[6] < 200 && sensorValues[7] < 200)
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      return;
    }

    else if(sensorValues[0] > 600 || sensorValues[7] > 600)
    {
      // Found an intersection.
      return;
    }

  } 

} // end follow_line  

  
     
// Turns to the sent variable of
// 'L' (left), 'R' (right), 'S' (straight), or 'B' (back)
// Tune 'turnSpeed' at declaration
void turn(char dir)
{
  switch(dir)
  {
    // Turn left 90deg
    case 'L':    
      digitalWrite(dir_a, LOW); 
      analogWrite(pwm_a,0);
      digitalWrite(dir_b, LOW);  
      digitalWrite(pwm_b, HIGH);
      
      line_position = qtrrc.readLine(sensorValues);
      
     while ( sensorValues[0] >500 && sensorValues[1] >500 )  // wait for outer most sensor to find the line
      {
        line_position = qtrrc.readLine(sensorValues);
      }
  
      // slow down speed
      analogWrite(pwm_a, turnSpeedSlow);
      analogWrite(pwm_b, turnSpeedSlow); 
      
      // find center
      while (line_position > 4350)  // tune - wait for line position to find near center
      {
        line_position = qtrrc.readLine(sensorValues);
      }
     
      // stop both motors
      analogWrite(pwm_b, 0);  // stop right motor first to better avoid over run
      analogWrite(pwm_a, 0);  
      break;
      
    // Turn right 90deg
    case 'R':        
      digitalWrite(dir_a, LOW); 
      digitalWrite(pwm_a, HIGH);
      digitalWrite(dir_b, LOW);  
      analogWrite(pwm_b, 0);
           
      line_position = qtrrc.readLine(sensorValues);
      
      while (sensorValues[6] >500 && sensorValues[7]>500)  // wait for outer most sensor to find the line
      {
        line_position = qtrrc.readLine(sensorValues);
      }
    
      // slow down speed
      analogWrite(pwm_a, turnSpeedSlow);
      analogWrite(pwm_b, turnSpeedSlow); 
      
      // find center
      while (line_position > 4350)  // tune - wait for line position to find near center
      {
        line_position = qtrrc.readLine(sensorValues);
      }
     
      // stop both motors
      analogWrite(pwm_a, 0);  
      analogWrite(pwm_b, 0);      
      break;
    
    // Turn right 180deg to go back
    case 'B':		
      digitalWrite(dir_a, LOW); 
       digitalWrite(pwm_a, HIGH);
      digitalWrite(dir_b, HIGH);  
      analogWrite(pwm_b, 0);
      
      line_position = qtrrc.readLine(sensorValues);
  
      while (sensorValues[0]<500 && sensorValues[1] <500 && sensorValues[6]<500 && sensorValues[7]<500)  // wait for outer most sensor to find the line
      {
        line_position = qtrrc.readLine(sensorValues);
      }
       
      // slow down speed
      analogWrite(pwm_a, turnSpeedSlow);
      analogWrite(pwm_b, turnSpeedSlow); 
      
      // find center
      while (line_position < 3250)  // tune - wait for line position to find near center
      {
        line_position = qtrrc.readLine(sensorValues);
      }
     
      // stop both motors
      analogWrite(pwm_a, 0);  
      analogWrite(pwm_b, 0);           
      break;

    // Straight ahead
    case 'S':
      // do nothing
      break;
  }
} // end turn


// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right)
{
  // Make a decision about how to turn.  The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if(found_left)
    return 'L';
  else if(found_straight)
    return 'S';
  else if(found_right)
    return 'R';
  else
    return 'B';
} // end select_turn



// This function is called once, from the main loop
void MazeSolve()
{
  // Loop until we have solved the maze.
  while(1)
  {
    // FIRST MAIN LOOP BODY  
    follow_line();

    // Drive straight a bit.  This helps us in case we entered the
    // intersection at an angle.
    digitalWrite(dir_a, LOW);  
    analogWrite(pwm_a, 60);
    digitalWrite(dir_b, LOW);  
    analogWrite(pwm_b, 60);   
    //delay(25); 
   
  Serial.print(line_position); // comment this line out if you are using raw values
  Serial.print("  ");
  Serial.print(sensorValues[0]);
  Serial.print("  ");
  Serial.print(sensorValues[1]);
  Serial.print("  ");
  Serial.print(sensorValues[2]);
  Serial.print("  ");
  Serial.print(sensorValues[3]);
  Serial.print("  ");
  Serial.print(sensorValues[4]);
  Serial.print("  ");
  Serial.print(sensorValues[5]);
  Serial.print("  ");
  Serial.print(sensorValues[6]);
  Serial.print("  ");
  Serial.println(sensorValues[7]);
    // These variables record whether the robot has seen a line to the
    // left, straight ahead, and right, whil examining the current
    // intersection.
    unsigned char found_left=0;
    unsigned char found_straight=0;
    unsigned char found_right=0;
		
    // Now read the sensors and check the intersection type.
    //line_position = qtrrc.readLine(sensorValues);

    // Check for left and right exits.
            
      
     if(sensorValues[0]> 500)
     {
        found_left = 1;
        if(sensorValues[7]<500)
           {
             found_right=0;
           
        
        if(sensorValues[1]<500&& sensorValues[3]<500 && sensorValues[4]<500&& sensorValues[2]<500 && sensorValues[5]<500 && sensorValues[6]<500 )
          {
            
            digitalWrite(dir_a,HIGH);
            analogWrite(pwm_a,180);
            digitalWrite(dir_b,HIGH);
            analogWrite(pwm_b,180);
            delay(1000);
            unsigned char dir = select_turn(found_left, found_straight, found_right);
            turn(dir);
            
          }
          }
          if(sensorValues[7]>500)
          {
            found_right=1;
            delay(1000);
            digitalWrite(dir_a,HIGH);
            analogWrite(pwm_a,180);
            digitalWrite(dir_b,HIGH);
            analogWrite(pwm_b,180);
            delay(1000);
            unsigned char dir = select_turn(found_left, found_straight, found_right);
            turn(dir);
            
          }
     }
    if(sensorValues[7] > 500){
        found_right = 1;
        if(sensorValues[7]<500)
           {
             found_right=0;
           
        
        if(sensorValues[1]<500 && sensorValues[3]<500 && sensorValues[4]<500&& sensorValues[2]<500 && sensorValues[5]<500 && sensorValues[6]<500 )
          {
            
            digitalWrite(dir_a,HIGH);
            analogWrite(pwm_a,180);
            digitalWrite(dir_b,HIGH);
            analogWrite(pwm_b,180);
            delay(1000);
            unsigned char dir = select_turn(found_left, found_straight, found_right);
            turn(dir);
            
          }
          }
    }
    line_position = qtrrc.readLine(sensorValues);
    if(sensorValues[3]>500 && sensorValues[4]>500){
         found_straight=1;
    }     
unsigned char dir = select_turn(found_left, found_straight, found_right);

    // Make the turn indicated by the path.
    turn(dir);

    // Drive straight a bit more - this is enough to line up our
    // wheels with the intersection.
    /*digitalWrite(dir_a, LOW);  
    analogWrite(pwm_a, 60);
    digitalWrite(dir_b, LOW);  
    analogWrite(pwm_b, 60);
    delay(drivePastDelay); 
  
    line_position = qtrrc.readLine(sensorValues);
    if(sensorValues[3] > 500 && sensorValues[4] > 500)
    found_straight = 1;
*/
    // Check for the ending spot.
    // If all six middle sensors are on dark black, we have
    // solved the maze.
   /* if(sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] > 600)
	break;
*/
    // Intersection identification is complete.
    // If the maze has been solved, we can follow the existing
    // path.  Otherwise, we need to learn the solution.
    
  }

} // end MazeSolve