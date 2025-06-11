#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <Servo.h>
#include "Si4463.h"
#include "422Mc80_4GFSK_009600H.h"
#include "MMFS.h"
#include "VehicleState.h"


mmfs::MAX_M10S gps;
mmfs::DPS368 baro;
mmfs::BMI088andLIS3MDL vehicle_imu;
Logger logger;

Sensor *sensors[] = {&gps, &baro, &vehicle_imu};

VehicleState vehicle(sensors, 3, nullptr);

MMFSConfig config = MMFSConfig()
                        .withState(&vehicle)
                        .withBuzzerPin(13)
                        .withBBPin(13)
                        .withUpdateRate(10)
                        .withBBAsync(true, 50);

MMFSSystem computer = MMFSSystem(&config);
int itterationloop = 0;

Servo actuator;

APRSConfig aprsConfigAvionics = {"KD3BBD", "ALL", "WIDE1-1", PositionWithoutTimestampWithoutAPRS, '\\', 'M'};
APRSConfig aprsConfigPayload = {"KQ4TCN", "ALL", "WIDE1-1", PositionWithoutTimestampWithoutAPRS, '\\', 'M'};
uint8_t encoding[] = {7, 4, 4};
APRSTelem avionicsTelem(aprsConfigAvionics);
Message msgAvionics;
Message msgPayload;

const char *avionicsCall = "KD3BBD";
const char *airbrakeCall = "KC3UTM";
Si4463HardwareConfig hwcfg = {
    MOD_4GFSK,        // modulation
    DR_4_8k,          // data rate
    (uint32_t)430e6,  // frequency (Hz)
    POWER_COTS_30dBm, // tx power (127 = ~20dBm)
    48,               // preamble length
    16,               // required received valid preamble
};

Si4463PinConfig pincfg = {
    &SPI, // spi bus to use
    10,   // cs
    15,   // sdn
    14,   // irq
    24,   // gpio0
    25,   // gpio1
    36,   // gpio2
    37,   // gpio3
};

Si4463 radio(hwcfg, pincfg);
void setup()
{
  computer.init();
  getLogger().recordLogData(mmfs::INFO_, "Entering Setup");  
  actuator.attach(2);
  if (radio.begin(CONFIG_422Mc80_4GFSK_009600H, sizeof(CONFIG_422Mc80_4GFSK_009600H)))
    {
        bb.onoff(BUZZER, 1000);
        getLogger().recordLogData(ERROR_, "Radio initialized.");
    }
    else
    {
        bb.onoff(BUZZER, 200, 3);
        getLogger().recordLogData(INFO_, "Radio failed to initialize.");
    }
  getLogger().recordLogData(mmfs::INFO_, "Leaving Setup");
}
uint32_t payloadTimer = millis();
uint32_t debugTimer = millis();
uint32_t txTimeout = millis();
bool sendPayload = false;
bool lookingForAvionics = true;

void loop() {
  if (computer.update()) {  
    if (vehicle.stage == GLIDING) {
      //This is ugly I know
      if (itterationloop < 40) {
        vehicle.servoOutput = 1350;
      }
      else if (itterationloop < 80) {
        vehicle.servoOutput = 1550;
      }
      else if (itterationloop < 120) {
        vehicle.servoOutput = 1750;
      }
      else if (itterationloop < 160) {
        vehicle.servoOutput = 1850;
      }
      else if (itterationloop < 260) {
        vehicle.servoOutput = 900;
      }
      else {
        itterationloop = 0;
      }
      itterationloop++;
      actuator.writeMicroseconds(vehicle.servoOutput);
    }
  }
   if (millis() - txTimeout > 200 && lookingForAvionics && radio.avail())
    {
        msgAvionics.size = radio.readRXBuf(msgAvionics.buf, Message::maxSize);
        char call[7] = {0};
        memset(call, 0, sizeof(char));
        memcpy(call, msgAvionics.buf, 6);
        Serial.println(call);
        if (strcmp(call, avionicsCall) == 0)
        {
            Serial.println("Got avionics telem");
            msgAvionics.decode(&avionicsTelem);
            payloadTimer = millis();
            lookingForAvionics = false;
            sendPayload = true;
        }
        radio.available = false;
    }

    if (millis() - payloadTimer > 500 && sendPayload)
    // if (millis() - payloadTimer > 1000)
    {
        // payloadTimer = millis();
        sendPayload = false;
        Serial.print("Timer after avionics is ");
        Serial.print(millis() - payloadTimer);
        Serial.println(", sending own telem");

        double orient[3] = {vehicle_imu.getAngularVelocity().x(), vehicle_imu.getAngularVelocity().y(), vehicle_imu.getAngularVelocity().z()};
        APRSTelem aprs = APRSTelem(aprsConfigPayload, gps.getPos().x(), gps.getPos().y(), baro.getAGLAltFt(), vehicle.getVelocity().z(), gps.getHeading(), orient, 0);

        aprs.stateFlags.setEncoding(encoding, 3);
        uint8_t arr[] = {(uint8_t)(int)baro.getTemp(), (uint8_t)vehicle.getStage(), (uint8_t)gps.getFixQual()};
        aprs.stateFlags.pack(arr);
        radio.send(aprs);

        txTimeout = millis();
        lookingForAvionics = true;
    }

    if (millis() - debugTimer > 500)
    {
        debugTimer = millis();
        Serial.println("here");
        Serial.println(radio.state);
        Serial.println(radio.readFRR(0));
    }

    radio.update();

}