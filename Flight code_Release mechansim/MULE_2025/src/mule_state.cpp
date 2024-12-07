#include "mule_state.h"

using namespace mmfs;

MuleState::MuleState(Sensor **sensors, int numSensors, LinearKalmanFilter *kfilter, int buzzPin) : State(sensors, numSensors, kfilter)
{
    stage = PRELAUNCH;
    timeOfLaunch = 0;
    timeOfLastStage = 0;
    timeOfDay = 0;
    buzzerPin = buzzPin;
}

void MuleState::updateState(double newTime)
{
    State::updateState(newTime); // call base version for sensor updates
    determineStage(); // determine the stage of the flight
}

void MuleState::determineStage()
{   
    int timeSinceLaunch = currentTime - timeOfLaunch;
    IMU *imu = reinterpret_cast<IMU *>(getSensor(IMU_));
    Barometer *baro = reinterpret_cast<Barometer *>(getSensor(BAROMETER_));
    Serial.println(imu->getAccelerationGlobal().z());
    if(stage == PRELAUNCH && imu->getAccelerationGlobal().z() > 20){
        logger.setRecordMode(FLIGHT);
        bb.aonoff(buzzerPin, 200);
        stage = BOOST;
        timeOfLaunch = currentTime;
        timeOfLastStage = currentTime;
        logger.recordLogData(INFO_, "Launch detected.");
        logger.recordLogData(INFO_, "Printing static data.");
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensorOK(sensors[i]))
            {
                sensors[i]->setBiasCorrectionMode(false);
            }
        }
    }
    else if(stage == BOOST && imu->getAccelerationGlobal().z() < 0){
        bb.aonoff(buzzerPin, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        logger.recordLogData(INFO_, "Coasting detected.");
    }
    else if(stage == COAST && baroVelocity <= 0 && timeSinceLaunch > 5){ // TODO fix this, this baroVelocity is not resistant to noise
        bb.aonoff(buzzerPin, 200, 2);
        timeOfLastStage = currentTime;
        char logData[100];
        snprintf(logData, 100, "Apogee detected at %.2f m.", position.z());
        logger.recordLogData(INFO_, logData);
        stage = DROUGE;
        logger.recordLogData(INFO_, "Drogue detected.");
    }
    else if(stage == DROUGE && baro->getAGLAltFt() < 1000){ // TODO get this number of drogue deployment
        bb.aonoff(buzzerPin, 200, 2);
        timeOfLastStage = currentTime;
        stage = MAIN;
        logger.recordLogData(INFO_, "Main detected.");
    }
    else if(stage == MAIN && ((baro->getAGLAltFt() < 100) || ((currentTime - timeOfLastStage) > 600000))){
        bb.aonoff(buzzerPin, 200, 2);
        timeOfLastStage = currentTime;
        stage = LANDED;
        logger.setRecordMode(GROUND);
        logger.recordLogData(INFO_, "Landing detected.");
    }
    else if (stage == LANDED && currentTime - timeOfLastStage > 60) // TODO check if it can dump data in 60 seconds
    {
        stage = DUMPED;
        logger.recordLogData(INFO_, "Dumped data after landing.");
    }
    else if((stage == PRELAUNCH || stage == BOOST) && baro->getAGLAltFt() > 250){
        logger.setRecordMode(FLIGHT);
        bb.aonoff(buzzerPin, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        logger.recordLogData(INFO_, "Launch detected. Using Backup Condition.");
        logger.recordLogData(INFO_, "Printing static data.");
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensorOK(sensors[i]))
            {
                sensors[i]->setBiasCorrectionMode(false);
            }
        }
    }
}