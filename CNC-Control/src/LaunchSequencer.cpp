#include "LaunchSequencer.h"
#include <RecordData/Logging/EventLogger.h>

LaunchSequencer::LaunchSequencer(CncState& inState, FileLoader& inLoader, BMI088& inImu)
    : state(inState), loader(inLoader), imu(inImu),
      streamState(StreamState::IDLE), lastQueryTime(0), lastLogTime(0) {}

void LaunchSequencer::update() {
    updateAcceleration();
    checkLaunchDetection();
    runSequence();
    state.prevAccel = state.totalAccel;
}

void LaunchSequencer::updateAcceleration() {
    state.accel = imu.getAccelSensor()->getAccel();

    // Exponential Moving Average filter to smooth out vibrational noise
    state.totalAccel = 0.8f * sqrt(
        state.accel.x() * state.accel.x() +
        state.accel.y() * state.accel.y() +
        state.accel.z() * state.accel.z()
    ) + 0.2f * state.prevAccel;

    // Rate-limit telemetry prints to 10Hz to preserve processing cycles
    if (millis() - lastLogTime > 100) {
        Serial.print("Total Acceleration: ");
        Serial.println(state.totalAccel);
        lastLogTime = millis();
    }
}

void LaunchSequencer::checkLaunchDetection() {
    const float LAUNCH_THRESHOLD = 40.0f;
    const unsigned long DETECT_DEBOUNCE_MS = 500;

    if (state.totalAccel > LAUNCH_THRESHOLD) {
        if (!state.detect) {
            state.detectTime = millis();
            state.detect = true;
        }
    } else {
        if (state.detect && (millis() - state.detectTime < DETECT_DEBOUNCE_MS) && !state.commandSent) {
            state.detect = false;
        }
    }

    if (state.detect && !state.commandSent && (state.step == 0)) {
        if (millis() - state.detectTime > DETECT_DEBOUNCE_MS) {
            LOGI("LAUNCH_DETECTED: Acceleration threshold sustained.");
            state.start = millis();
            state.spindleStart();
            state.commandSent = true;
            streamState = StreamState::SEND_NEXT;
            LOGI("STATE: Sequence started.");
        }
    }
}

void LaunchSequencer::runSequence() {
    if (!state.commandSent || state.commandStopped) return;

    parseGrblResponse();

    switch (streamState) {
        case StreamState::IDLE:
            break;

        case StreamState::SEND_NEXT:
            if (state.step < loader.countLine()) {
                const char* cmd = loader.getLine(state.step);
                if (cmd) {
                    state.send(cmd);
                    Serial.print("Sending step ");
                    Serial.print(state.step);
                    Serial.print(": ");
                    Serial.println(cmd);

                    streamState = StreamState::WAITING_FOR_OK;
                } else {
                    state.step++;
                }
            } else {
                state.spindleStop();
                state.commandStopped = true;
                streamState = StreamState::IDLE;
                LOGI("STATE: Sequence complete. System Idle.");
            }
            break;

        case StreamState::WAITING_FOR_OK:
            break;

        case StreamState::WAITING_FOR_IDLE:
            if (millis() - lastQueryTime > 100) {
                Serial8.write('?');
                lastQueryTime = millis();
            }
            break;
    }
}

void LaunchSequencer::parseGrblResponse() {
    while (Serial8.available() > 0) {
        char c = Serial8.read();
        if (c == '\n' || c == '\r') {
            if (rxBuffer.length() > 0) {
                rxBuffer.trim();

                if (streamState == StreamState::WAITING_FOR_OK) {
                    if (rxBuffer == "ok") {
                        streamState = StreamState::WAITING_FOR_IDLE;
                    } else if (rxBuffer.startsWith("error")) {
                        Serial.print("GRBL error encountered: ");
                        Serial.println(rxBuffer);
                        streamState = StreamState::WAITING_FOR_IDLE;
                    }
                }
                else if (streamState == StreamState::WAITING_FOR_IDLE) {
                    if (rxBuffer.startsWith("<Idle")) {
                        state.step++;
                        streamState = StreamState::SEND_NEXT;
                    }
                }
                rxBuffer = "";
            }
        } else {
            rxBuffer += c;
        }
    }
}
