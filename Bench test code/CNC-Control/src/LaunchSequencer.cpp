#include "LaunchSequencer.h"
#include <RecordData/Logging/EventLogger.h>

LaunchSequencer::LaunchSequencer(CncState& inState, FileLoader& inLoader, BMI088& inImu)
    : state(inState), loader(inLoader), imu(inImu) {}


void LaunchSequencer::update() {
    updateAcceleration();
    checkLaunchDetection();
    runSequence();
    state.prevAccel = state.totalAccel;
}

void LaunchSequencer::updateAcceleration() {
    state.accel = imu.getAccelSensor()->getAccel();

    state.totalAccel = 0.8f * sqrt(
        state.accel.x() * state.accel.x() +
        state.accel.y() * state.accel.y() +
        state.accel.z() * state.accel.z()
    ) + 0.2f * state.prevAccel;

    Serial.print("Total Acceleration: ");
    Serial.println(state.totalAccel);
}

void LaunchSequencer::checkLaunchDetection() {
    const float LAUNCH_THRESHOLD = 40.0f;
    const unsigned long DETECT_DEBOUNCE_MS = 500;

    if (state.totalAccel > LAUNCH_THRESHOLD && !state.detect) {
        state.detectTime = millis();
        state.detect = true;
    }

    if (state.totalAccel < LAUNCH_THRESHOLD && state.detect) {
        state.detect = false;
        state.detectTime = 0;
    }

    bool launchConfirmed = (state.totalAccel > LAUNCH_THRESHOLD)
        && !state.commandSent
        && ((millis() - state.detectTime) > DETECT_DEBOUNCE_MS)
        && (state.step == 0);

    if (launchConfirmed) {
        LOGI("LAUNCH_DETECTED: Acceleration threshold exceeded.");
        state.start = millis();
        state.spindleStart();
        state.commandSent = true;
        LOGI("STATE: Sequence started.");
    }
}

void LaunchSequencer::runSequence() {
    if (!state.commandSent || state.commandStopped) return;

    if (state.step < loader.countLine()) {
        const char* cmd = loader.getLine(state.step);
        if (!cmd) return;

        state.send(cmd);
        Serial.print("Sending step ");
        Serial.print(state.step);
        Serial.print(": ");
        Serial.println(cmd);

        if (!waitForOk()) {
            Serial.println("WARNING: No 'ok' received, continuing anyway.");
        }
        waitForIdle();

        state.step++;

    } else {
        state.spindleStop();
        state.commandStopped = true;
        LOGI("STATE: Sequence complete. System Idle.");
    }
}

bool LaunchSequencer::waitForOk(unsigned long timeoutMs) {
    unsigned long timeout = millis();
    while (millis() - timeout < timeoutMs) {
        if (Serial8.available()) {
            String response = Serial8.readStringUntil('\n');
            response.trim();
            if (response == "ok") return true;
            if (response.startsWith("error")) {
                Serial.print("GRBL error: ");
                Serial.println(response);
                return false;
            }
        }
    }
    return false;
}

bool LaunchSequencer::waitForIdle() {
    while (true) {
        Serial8.println("?");
        delay(100);
        if (Serial8.available()) {
            String status = Serial8.readStringUntil('\n');
            status.trim();
            if (status.startsWith("<Idle")) return true;
        }
    }
}

