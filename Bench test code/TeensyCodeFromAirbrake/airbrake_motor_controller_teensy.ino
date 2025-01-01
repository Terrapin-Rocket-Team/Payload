#include "sdcard.h"
#include "timing.h"

#include "timing.h"
#include "sdcard.h"

#define HWSERIAL Serial1

// Motor Constants
#define MOTOR_DIRECTION_PIN 3
#define MOTOR_STEP_PIN 2
#define MOTOR_ENABLE_PIN 4
#define MOTOR_FAULT_PIN 5
#define MOTOR_SPEED 250                 // microseconds between each step; 200 microseconds is fastest with accurate movement, going below will make it too fast
#define MOTOR_MAX_DEGREES 1300.0        // Maximum actuation
#define MOTOR_MIN_DEGREES 0.0           // Minimum actuation
#define MOTOR_ANGLE_MOVEMENT 130.0     // How many degrees to move at a time max

#define LNot 209.423
#define Wh 38.1
#define Wa 23.2918

int curActuation = 0;              // Used to track active actuation state
int desiredActuation = 0;          // Used to track desired actuation state

char receivedChars[64];
char receivedTime[32];
char receivedSteps[32];
char receivedIsLaunched;
boolean newData = false;

void setup() {
  Serial.begin(115200);
  //while (!Serial);
  
  HWSERIAL.begin(115200);
  while (!HWSERIAL);
  //Serial1.setTX(0);
  //Serial1.setRX(1);
  //Serial.println(F("Starting Actuation Test Program!"));

  // Setup Actuator Flaps
  // Declare pins as Outputs
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH); //disable motor by default

  if (!setupSDCard()) {
    Serial.println("SD Error!");
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW); 
  //Serial.println(F("Test Program Ready!"));

  //Serial.println(angleToSteps(20.0f));

  while (HWSERIAL.available() > 0) {
      HWSERIAL.read();
  }

  float timeNow = micros() / (1000000.0f);
  setStartTime(timeNow);
}

void loop() {
  float timeNow = micros() / (1000000.0f);
  
  recvWithEndMarker();
  while(newData){
    digitalWrite(LED_BUILTIN, HIGH);
    newData = false;
    //float curTime = String(receivedTime).toFloat();

    // Get Angle
    int newAngle = String(receivedSteps).toInt();

    // Get IsLaunched
    bool newIL = false;
    if(strcmp(receivedIsLaunched, "T") == 0){
      newIL = true;
    }

    // Set new desired actuation
    desiredActuation = (int)angleToSteps(newAngle);

    // set new launch time
    if(newIL && getLaunchTime() < 0.1f){
      setLaunchTime(timeNow);
    }else if(!newIL && getLaunchTime() > 0.1f){
      setLaunchTime(0.0f);
      switchToNewFile();
    }

    Serial.println(receivedChars);
    Serial.println("---");
    Serial.println(newIL);
    Serial.println(getLaunchTime());
    Serial.println("---");
    Serial.println(newAngle);
    Serial.println(desiredActuation);
    Serial.println(curActuation);
    Serial.println("--------------------------------------");
    
    recvWithEndMarker();
  }
  
  //while (HWSERIAL.available() > 0) {
  //    HWSERIAL.read();
  //}

  outputSDData(timeNow, desiredActuation, curActuation, receivedChars);
  
  rotateFlaps();
  digitalWrite(LED_BUILTIN, LOW);
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (HWSERIAL.available() > 0) {
        rc = HWSERIAL.read();
        //Serial.print(" - ");
        //Serial.println(rc);

        if (rc != endMarker && rc != '\r') {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= 64) {
                ndx = 63;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string

            //separate the stuff
            // Serial.println(receivedChars);
            
            // Steps
            char *startPos = receivedChars;
            for(int i = 0; i < 3; i++){
                char *endPos = strstr(startPos, "@");
                if(!endPos){
                  ndx = 0;
                  return;
                }

                char newSteps[32] = {};
                strncpy(newSteps, startPos, endPos-startPos);
                newSteps[endPos-startPos] = '\0';

                if(i > 0){
                  if(strcmp(newSteps, receivedSteps) != 0){
                    ndx = 0;
                    return;
                  }
                }else{
                  strcpy(receivedSteps, newSteps);
                }

                startPos = endPos + 1;
            }

            // Is Launched
            for(int i = 0; i < 3; i++){
                char *endPos = strstr(startPos, "|");
                if(!endPos){
                  ndx = 0;
                  return;
                }

                if(endPos - 1 != startPos){
                  ndx = 0;
                  return;
                }

                char newIsLaunched = *startPos;
                if(!(newIsLaunched == 'T' || newIsLaunched == 'F')){
                  ndx = 0;
                  return;
                }

                if(i > 0){
                  if(newIsLaunched != receivedIsLaunched){
                    ndx = 0;
                    return;
                  }
                }else{
                  receivedIsLaunched = newIsLaunched;
                }

                startPos = endPos + 1;
            }

            /* 
            Serial.println(receivedChars);
            Serial.println(receivedSteps);
            Serial.println(receivedIsLaunched);
            Serial.println("-------------");
            */
            
            ndx = 0;
            newData = true;
            return;
        }
    }
}

// Wrapper for motor functions, to prevent misuse and allow for gradual actuation and adjustment between sensor readings
void rotateFlaps() {
  // Figure out how much we WANT to actuate and in what direction
  int diff = curActuation - desiredActuation;
  bool isOpening = false;
  if (diff < 0) {
    isOpening = true;
    diff *= -1;
  }

  // Cap how much we want to actuate to prevent delaying other code for too long
  int angle = diff;
  if (angle > MOTOR_ANGLE_MOVEMENT) {
    angle = MOTOR_ANGLE_MOVEMENT;
  }

  // Do nothing if actuation would violate bounds
  if (isOpening && curActuation + angle > MOTOR_MAX_DEGREES) {
    return;
  } else if (!isOpening && curActuation - angle < MOTOR_MIN_DEGREES) {
    return;
  }

  // If no issues found, and angle is substantial enough, actuate in the needed direction
  if (angle > 0) {
    angle += 1;
    if (isOpening) {
      rotateClockwise(angle);
    } else {
      rotateCounterClockwise(angle);
    }

    //Serial.println(curActuation);
    //Serial.println(desiredActuation);
    //Serial.println(angle);
  }
}

float angleToSteps(float angle){
  float newMM = LNot + (Wh * sin(angle * (3.1415/180.0))) - sqrt((LNot * LNot) + ((Wh - Wa) * (Wh - Wa)) - (((Wh * cos(angle * (3.1415/180.0))) - Wa) * ((Wh * cos(angle * (3.1415/180.0))) - Wa)));
  return newMM * (200.0/6.0);
}

// close the flaps
void rotateClockwise(int angle)
{
  //float steps = (angle / 360) * MOTOR_STEPS_PER_REVOLUTION;
  int steps = angle;
  // Set motor direction clockwise
  digitalWrite(MOTOR_DIRECTION_PIN, HIGH);
  // Enable the motor
  digitalWrite(MOTOR_ENABLE_PIN, LOW);

  for (int x = 0; x < steps; x++)
  {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(MOTOR_SPEED); //don't go below 200!!!
    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(MOTOR_SPEED);
  }
  // Disable the motor
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  curActuation += angle;
}

// Open the flaps
void rotateCounterClockwise(int angle)
{
  //float steps = (angle / 360) * MOTOR_STEPS_PER_REVOLUTION;
  int steps = angle;
  // Set motor direction counter-clockwise
  digitalWrite(MOTOR_DIRECTION_PIN, LOW);
  // Enable the motor
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  for (int x = 0; x < steps; x++)
  {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(MOTOR_SPEED); //don't go below 200!!!
    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(MOTOR_SPEED);
  }
  // Disable the motor
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  curActuation -= angle;
}
