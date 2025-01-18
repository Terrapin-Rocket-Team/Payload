#include <Arduino.h>
#include "TADPOLState.h"

Barometer TADPOLBarometer("BMP280");
IMU TADPOLIMU("BNO055");
GPS TADPOLGPS("NEO-M9N");
TADPOLState TADPOLSTATE;

PWMServo TADPOLState::leftServo; //allows for PMWServo objects to be accessed by servo functions in file
PWMServo TADPOLState::rightServo;

double DEFUALT_GOAL = 0; //defined between [0,360] going ccw from north
int BUZZER_PIN = 9;

void setup() {
    buzz(BUZZER_PIN, 1000);
    Serial.begin(115200);
    delay(2000);
    //add sensors
    TADPOLSTATE.addBarometer(TADPOLBarometer);
    TADPOLSTATE.addIMU(TADPOLIMU);
    TADPOLSTATE.addGPS(TADPOLGPS);

    //setup steps
    TADPOLSTATE.stateBarometer.setupBarometer();
    TADPOLSTATE.stateIMU.setupIMU();
    TADPOLSTATE.stateGPS.setupGPS();

    TADPOLSTATE.setcsvHeader();
    String csvHeader = TADPOLSTATE.csvHeader + "Left Servo PWM" + "Right Servo PWM" + "Target Lat" + "Target Long" + "Goal Angle";
    setupPSRAM(csvHeader);
    setupSDCard(csvHeader);

    TADPOLSTATE.servoSetup(2,3,90,90);
    //pinMode(6, OUTPUT);
    //digitalWrite(6, HIGH); //Set the servos to be on
    buzz(BUZZER_PIN, 3000);
 }

void loop() {
    TADPOLSTATE.settimeAbsolute();
    TADPOLSTATE.stateBarometer.readBarometer();
    TADPOLSTATE.stateIMU.readIMU();
    TADPOLSTATE.stateGPS.readGPS();

    TADPOLSTATE.determineTADPOLStage();
    //Serial.println(TADPOLSTATE.stage);

    // Test code for finding the correct goal angle without active gps onsite
    // Point targetCoords = TADPOLSTATE.getTargetCoordinates();
    // double goal = getgoal(targetCoords, Point(TADPOLSTATE.stateGPS.latitude, TADPOLSTATE.stateGPS.longitude));
    // goal = getgoal(targetCoords, Point(-75.87361, 39.08521));
    // TADPOLSTATE.goDirection(goal);
    // Serial.println(targetCoords.x, 8);
    // Serial.println(targetCoords.y, 8);
    // Serial.println(goal);
    Point targetCoords;
    double goalAngle = 0;
    if(TADPOLSTATE.stage == "Main"){
        if(TADPOLSTATE.stateGPS.satellites > 3){
            targetCoords = TADPOLSTATE.getTargetCoordinates();
            goalAngle = getGoalAngle(targetCoords, Point(TADPOLSTATE.stateGPS.longitude, TADPOLSTATE.stateGPS.latitude));
            TADPOLSTATE.goDirection(goalAngle);
        }
        else{
            TADPOLSTATE.goDirection(DEFUALT_GOAL);
        }
    }
    else if(TADPOLSTATE.stage == "Landed"){
        TADPOLSTATE.servoSetup(2,3,90,90);
    }

    TADPOLSTATE.setdataString();
    String dataString = TADPOLSTATE.getdataString() + String(TADPOLSTATE.left_servo_value) + String(TADPOLSTATE.right_servo_value) + String(targetCoords.x) + String(targetCoords.y) + String(goalAngle);
    recordData(dataString, TADPOLSTATE.getrecordDataStage());
}

double getGoalAngle(Point target, Point current){
    //Returns an angle from [0:360] from north to get to a target from a current point
    double theta = atan2(target.y - current.y, target.x - current.x);
    theta = theta*180/3.14;
    double goal = 270 + theta;
    if(goal > 360){goal -= 360;}
    return goal;
}
