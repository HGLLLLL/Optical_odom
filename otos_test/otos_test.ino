// SparkFun Optical Tracking Odometry Sensor (OTOS, PAA5160E1) test on ESP32
// Wiring: OTOS 3V3 -> ESP32 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22
//
// Serial commands (115200 baud):
//   r : reset tracking (set current position as origin)
//   c : recalibrate IMU (keep the sensor still and flat!)
//   v : toggle velocity/acceleration output

#include <Wire.h>
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"

#define I2C_SDA 21
#define I2C_SCL 22
#define PRINT_PERIOD_MS 100

QwiicOTOS myOtos;
bool showVel = false;

void calibrate()
{
    Serial.println("Calibrating IMU, keep sensor still and flat...");
    myOtos.calibrateImu();
    myOtos.resetTracking();
    Serial.println("Done. Tracking reset to (0, 0, 0).");
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== SparkFun OTOS test ===");

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    while (myOtos.begin() == false)
    {
        Serial.println("OTOS not found on I2C (0x17). Check wiring...");
        delay(1000);
    }
    Serial.println("OTOS connected!");

    sfe_otos_version_t hw, fw;
    myOtos.getVersionInfo(hw, fw);
    Serial.printf("Hardware v%d.%d, Firmware v%d.%d\n", hw.major, hw.minor, fw.major, fw.minor);

    myOtos.setLinearUnit(kSfeOtosLinearUnitMeters);
    myOtos.setAngularUnit(kSfeOtosAngularUnitDegrees);

    // Tune these after measuring a known distance / full rotation
    myOtos.setLinearScalar(1.0);
    myOtos.setAngularScalar(1.0);

    calibrate();
    Serial.println("Commands: r=reset  c=calibrate  v=toggle velocity");
}

void loop()
{
    while (Serial.available())
    {
        char ch = Serial.read();
        if (ch == 'r')
        {
            myOtos.resetTracking();
            Serial.println(">> Tracking reset");
        }
        else if (ch == 'c')
        {
            calibrate();
        }
        else if (ch == 'v')
        {
            showVel = !showVel;
        }
    }

    sfe_otos_pose2d_t pos;
    myOtos.getPosition(pos);

    if (showVel)
    {
        sfe_otos_pose2d_t vel, acc;
        myOtos.getVelocity(vel);
        myOtos.getAcceleration(acc);
        Serial.printf("X:%8.2f cm  Y:%8.2f cm  H:%8.2f deg | vX:%7.2f vY:%7.2f cm/s  vH:%7.2f deg/s | aX:%6.2f aY:%6.2f m/s^2\n",
                      pos.x * 100, pos.y * 100, pos.h,
                      vel.x * 100, vel.y * 100, vel.h,
                      acc.x, acc.y);
    }
    else
    {
        Serial.printf("X:%8.2f cm  Y:%8.2f cm  H:%8.2f deg\n", pos.x * 100, pos.y * 100, pos.h);
    }

    delay(PRINT_PERIOD_MS);
}
