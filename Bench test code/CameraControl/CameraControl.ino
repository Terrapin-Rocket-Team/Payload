#include <iostream>
#include <Servo.h>
#include <cmath>
#include "mule_state.h"
#include "servo_vehicle_state.h"
#define M_PI 3.14159265358979323846
using namespace std;

float getDistance(float rlat, float rlong, float ralt, float vlat, float vlong, float valt) {
    float dist;
    float vy = (rlat - vlat)*111111;
    float vx = (rlong - vlong)*111111;
    float vz = (ralt - valt);
    dist = sqrt(pow(vy, 2) + pow(vx, 2) + pow(vz, 2));
    return dist;
}   

float getPhi(float rlat, float rlong, float vlat, float vlong) {
    float phi;
    float vy = (rlat - vlat)*111111;
    float vx = (rlong - vlong)*111111;
    phi = atan2(vx, vy)*180/M_PI;
    return phi;

    //angle between north and vector to rocket
    //phi = 0 when rocket is due north
    //phi = 90 when rocket is due east
    //phi = 180 when rocket is due south
    //phi = -90 when rocket is due west
}

float getTheta(float rlat, float rlong, float ralt, float vlat, float vlong, float valt) {
    float theta;
    float vz = (ralt - valt);
    float r = getDistance(rlat, rlong, ralt, vlat, vlong, valt);
    theta = -1*(acos(vz / r)*180/M_PI - 90);
    return theta;

    //angle between horizon and vector to rocket
    //theta = 0 when rocket is on horizon
    //theta = 90 when rocket is directly overhead
    //theta = -90 when rocket is directly below
}

Servo pitch;
Servo yaw;

void setup() {
    
  Serial.begin(9600);
  pinMode(10,OUTPUT);
  pinMode(9,OUTPUT);
  pitch.attach(10);
  yaw.attach(9);

}

void loop() {

    float rlat, rlong, ralt, vlat, vlong, valt;


    // This creates random values, need to figure out how to get actual values from vehicle/rocket
    rlat = random(24, 48);
    rlong = random(64, 128);
    ralt = random(30000);
    vlat = random(24, 48);
    vlong = random(64, 128);
    valt = random(30000);

    /*

    is this right?

    ralt = muleState :: PackedData :: pz;
    rlat = muleState :: PackedData :: px;
    rlong = muleState :: PackedData :: py;

    valt = ServoVehicleState :: PackedData :: pz;
    vlat = ServoVehicleState :: PackedData :: px;
    vlong = ServoVehicleState :: PackedData :: py;

    */

    float distance = getDistance(rlat, rlong, ralt, vlat, vlong, valt);
    float theta = getTheta(rlat, rlong, vlat, vlong);
    float phi = getPhi(rlat, rlong, ralt, vlat, vlong, valt);

    Serial.print("distance (m): ");
    Serial.print(distance);
    Serial.print(" theta (deg): ");
    Serial.print(theta);
    Serial.print(" phi (deg): ");
    Serial.println(phi);

    pitch.write(theta);
    yaw.write(phi);

    delay(2000); 

 
}
