#include <Arduino.h>
#include "TADPOLState.h"
#include <Kalman_Filter.h>

Barometer TADPOLBarometer("BMP280");
IMU TADPOLIMU("BNO055");
GPS TADPOLGPS("NEO-M9N");
TADPOLState TADPOLSTATE;

long KF_PREVIOUS_ITER_TIME; //in milliseconds
bool HAS_GPS = false;
KFState KFSTATE;

void setup() {
    TADPOLSTATE.buzzerPin = 9;
    buzz(TADPOLSTATE.buzzerPin, 1000, 1);
    Serial.begin(115200);

    //add sensors
    TADPOLSTATE.addBarometer(TADPOLBarometer);
    TADPOLSTATE.addIMU(TADPOLIMU);
    TADPOLSTATE.addGPS(TADPOLGPS);

    //setup steps
    TADPOLSTATE.stateBarometer.setupBarometer();
    TADPOLSTATE.stateIMU.setupIMU();
    TADPOLSTATE.stateGPS.setupGPS();

    //setup datalogging
    TADPOLSTATE.setcsvHeader();
    setupPSRAM(TADPOLSTATE.csvHeader);
    bool sdSuccess = setupSDCard(TADPOLSTATE.csvHeader);

    //setup KF
    //Initialize kf state
    int KFStateSize = 6;
    int KFMeasSize = 3;
    int KFControlSize = 3;
    double* initialState = new double[6] {0, 0, 0, 0, 0, 0};
    double* initialControl = new double[3] {0, 0, 0};
    KFSTATE = initialize(KFStateSize, KFMeasSize, KFControlSize, initialState, initialControl);

    if(sdSuccess && TADPOLSTATE.successfulSetup()){buzz(TADPOLSTATE.buzzerPin, 3000, 1);}
    else{buzz(TADPOLSTATE.buzzerPin, 250, 5);}

    KF_PREVIOUS_ITER_TIME = millis();
}

void loop() {
    TADPOLSTATE.settimeAbsolute();
    TADPOLSTATE.stateBarometer.readBarometer();
    TADPOLSTATE.stateIMU.readIMU();
    TADPOLSTATE.stateGPS.readGPS();

    //Run the KF
    double timeStep = (TADPOLSTATE.timeAbsolute - KF_PREVIOUS_ITER_TIME)/1000;
    double* meas = new double[3] {TADPOLSTATE.stateGPS.gpsPosition.x(), TADPOLSTATE.stateGPS.gpsPosition.y(), TADPOLSTATE.stateBarometer.relativeAltitude};
    double* control = new double[3] {TADPOLSTATE.stateIMU.acceleration.x(), TADPOLSTATE.stateIMU.acceleration.y(), TADPOLSTATE.stateIMU.acceleration.z() - 9.8};

    if (TADPOLSTATE.stateGPS.satellites > 5){
      HAS_GPS = true;
    }
    
    KFSTATE = iterate(KFSTATE, timeStep, meas, control, HAS_GPS);
    delete[] meas;
    delete[] control;
    double* KFOutputState = KFSTATE.X;
    KF_PREVIOUS_ITER_TIME = millis();
    
    // //Set the kf output data in the TADPOLSTATE 
    TADPOLSTATE.position.x() = KFOutputState[0]; TADPOLSTATE.position.y() = KFOutputState[1]; TADPOLSTATE.position.z() = KFOutputState[2];
    TADPOLSTATE.velocity.x() = KFOutputState[3]; TADPOLSTATE.velocity.y() = KFOutputState[4]; TADPOLSTATE.velocity.z() = KFOutputState[5];
    TADPOLSTATE.stateIMU.acceleration.x() = TADPOLSTATE.stateIMU.acceleration.x(); TADPOLSTATE.acceleration.y() = TADPOLSTATE.stateIMU.acceleration.y(); TADPOLSTATE.acceleration.z() = TADPOLSTATE.stateIMU.acceleration.z();
    
    TADPOLSTATE.determineTADPOLStage();
    
    TADPOLSTATE.setdataString();
    recordData(TADPOLSTATE.getdataString(), TADPOLSTATE.getrecordDataStage());
}
