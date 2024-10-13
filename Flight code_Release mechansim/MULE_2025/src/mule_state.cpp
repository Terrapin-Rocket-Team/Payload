#include "mule_state.h"

using namespace mmfs;

MuleState::MuleState(Sensor **sensors, int numSensors, LinearKalmanFilter *kfilter, bool stateRecordsOwnData) : State(sensors, numSensors, kfilter, stateRecordsOwnData)
{
    stage = PRELAUNCH;
    timeOfLaunch = 0;
    timeOfLastStage = 0;
    timeOfDay = 0;
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
    if (stage == 0 &&
        (sensorOK(imu) || sensorOK(baro)) &&
        (sensorOK(imu) ? abs(imu->getAccelerationGlobal().z()) > 25 : true) &&
        (sensorOK(baro) ? baro->getAGLAltFt() > 60 : true))
    // if we are in preflight AND
    // we have either the IMU OR the barometer AND
    // imu is ok AND the z acceleration is greater than 29 ft/s^2 OR imu is not ok AND
    // barometer is ok AND the relative altitude is greater than 30 ft OR baro is not ok
    // essentially, if we have either sensor and they meet launch threshold, launch. Otherwise, it will never detect a launch.
    {
        bb.aonoff(buzzerPin, 200);
        logger.setRecordMode(FLIGHT);
        stage = BOOST;
        timeOfLaunch = currentTime;
        timeOfLastStage = currentTime;
        logger.recordLogData(INFO_, "Launch detected.");
        logger.recordLogData(INFO_, "Printing static data.");
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensorOK(sensors[i]))
            {
                char logData[200];
                snprintf(logData, 200, "%s: %s", sensors[i]->getName(), sensors[i]->getStaticDataString());
                logger.recordLogData(INFO_, logData);
                sensors[i]->setBiasCorrectionMode(false);
            }
        }
    }
    else if (stage == BOOST && abs(acceleration.z()) < 10)
    {
        bb.aonoff(buzzerPin, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        logger.recordLogData(INFO_, "Coasting detected.");
    }
    else if (stage == COAST && baroVelocity <= 0 && timeSinceLaunch > 5)
    {
        bb.aonoff(buzzerPin, 200, 3);
        char logData[100];
        snprintf(logData, 100, "Apogee detected at %.2f m.", position.z());
        logger.recordLogData(INFO_, logData);
        timeOfLastStage = currentTime;
        stage = DROUGE;
        logger.recordLogData(INFO_, "Drogue conditions detected.");
    }
    else if (stage == DROUGE && baro->getAGLAltFt() < 1000 && timeSinceLaunch > 10)
    {
        bb.aonoff(buzzerPin, 200, 4);
        stage = MAIN;
        timeOfLastStage = currentTime;
        logger.recordLogData(INFO_, "Main parachute conditions detected.");
    }
    else if (stage == MAIN && baroVelocity > -1 && baro->getAGLAltFt() < 66 && timeSinceLaunch > 15)
    {
        bb.aonoff(buzzerPin, 200, 5);
        timeOfLastStage = currentTime;
        stage = LANDED;
        logger.recordLogData(INFO_, "Landing detected. Waiting for 5 seconds to dump data.");
    }
    else if (stage == LANDED && currentTime - timeOfLastStage > 5)
    {
        logger.setRecordMode(GROUND);
        logger.recordLogData(INFO_, "Dumped data after landing.");
    }
}