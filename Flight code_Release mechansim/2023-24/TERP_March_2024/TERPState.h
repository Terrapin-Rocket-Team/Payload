#ifndef TERP_STATE_H
#define TERP_STATE_H

#include <Multi_Mission_Flight_Software.h>

class TERPState: public State{
public:
    void determineTERPStage();

    int buzzerPin;

private:

};

void TERPState::determineTERPStage(){
    //Stages: "Pre Launch", "Powered Ascent", "Coasting", "Drogue", "Main", "Landed"
    //determineaccelerationMagnitude(acceleration);
    determinetimeSincePreviousStage();
    determineapogee(stateBarometer.relativeAltitude);
    if(stage == "Pre Launch" && stateIMU.acceleration.z() > 30){
        buzz(buzzerPin, 100, 2);
        timeLaunch = millis();
        timePreviousStage = millis();
        stage = "Powered Ascent";
        stageNumber = 1;
        recordDataStage = "Flight";
    }
    else if(stage == "Powered Ascent" && stateIMU.acceleration.z() < 0){
        timePreviousStage = millis();
        stage = "Coasting";
        stageNumber = 2;
    }
    else if(stage == "Coasting" && timeAbsolute > (apogeeTime+5)){
        timePreviousStage = millis();
        stage = "Drogue";
        stageNumber = 3;
    }
    else if(stage == "Drogue" && stateBarometer.relativeAltitude < 300){ 
        timePreviousStage = millis();
        stage = "Main";
        stageNumber = 4;
    }
    else if(stage == "Main" && ((stateBarometer.relativeAltitude < 50) || ((millis() - timePreviousStage) > 600000))){
        timePreviousStage = millis();
        stage = "Landed";
        recordDataStage = "PostFlight";
        stageNumber = 5;
    }
    else if((stage == "Pre Launch" || stage == "Powered Ascent") && stateBarometer.relativeAltitude > 100){
        buzz(buzzerPin, 100, 2);
        timePreviousStage = millis()/1000;
        stage = "Coasting";
        recordDataStage = "Flight";
    }
}

#endif
