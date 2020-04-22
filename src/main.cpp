#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion orientation;
VectorInt16 linAccel;
VectorInt16 linAccelNoGravity;
VectorFloat gravity;
VectorInt16 rotAccel;

uint8_t msg[42];

void setup()
{
    Wire.begin();

    Serial.begin(115200);

    mpu.initialize();
    mpu.dmpInitialize();

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);

    packetSize = mpu.dmpGetFIFOPacketSize();
}

void loop()
{
    while (fifoCount < packetSize)
    {
        fifoCount = mpu.getFIFOCount();
    }

    if (fifoCount >= 1024)
    {
        mpu.resetFIFO();
    }
    else
    {
        while (fifoCount >= packetSize)
        {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }

        mpu.dmpGetQuaternion(&orientation, fifoBuffer);
        mpu.dmpGetAccel(&linAccel, fifoBuffer);
        mpu.dmpGetGyro(&rotAccel, fifoBuffer);

        mpu.dmpGetGravity(&gravity, &orientation);
        mpu.dmpGetLinearAccel(&linAccelNoGravity, &linAccel, &gravity);

        int32_t linX = (int32_t)(linAccelNoGravity.x);
        int32_t linY = (int32_t)(linAccelNoGravity.y);
        int32_t linZ = (int32_t)(linAccelNoGravity.z);

        int32_t rotX = (int32_t)(rotAccel.x);
        int32_t rotY = (int32_t)(rotAccel.y);
        int32_t rotZ = (int32_t)(rotAccel.z);

        int32_t orientationX = (int32_t)(orientation.x * 1000);
        int32_t orientationY = (int32_t)(orientation.y * 1000);
        int32_t orientationZ = (int32_t)(orientation.z * 1000);
        int32_t orientationW = (int32_t)(orientation.w * 1000);

        msg[0] = 0x80;

        msg[1] = (uint8_t)(linX & 0xFF);
        msg[2] = (uint8_t)((linX >> 8) & 0xFF);
        msg[3] = (uint8_t)((linX >> 16) & 0xFF);
        msg[4] = (uint8_t)((linX >> 24) & 0xFF);

        msg[5] = (uint8_t)(linY & 0xFF);
        msg[6] = (uint8_t)((linY >> 8) & 0xFF);
        msg[7] = (uint8_t)((linY >> 16) & 0xFF);
        msg[8] = (uint8_t)((linY >> 24) & 0xFF);

        msg[9]  = (uint8_t)(linZ & 0xFF);
        msg[10] = (uint8_t)((linZ >> 8) & 0xFF);
        msg[11] = (uint8_t)((linZ >> 16) & 0xFF);
        msg[12] = (uint8_t)((linZ >> 24) & 0xFF);

        msg[13] = (uint8_t)(rotX & 0xFF);
        msg[14] = (uint8_t)((rotX >> 8) & 0xFF);
        msg[15] = (uint8_t)((rotX >> 16) & 0xFF);
        msg[16] = (uint8_t)((rotX >> 24) & 0xFF);

        msg[17] = (uint8_t)(rotY & 0xFF);
        msg[18] = (uint8_t)((rotY >> 8) & 0xFF);
        msg[19] = (uint8_t)((rotY >> 16) & 0xFF);
        msg[20] = (uint8_t)((rotY >> 24) & 0xFF);

        msg[21] = (uint8_t)(rotZ & 0xFF);
        msg[22] = (uint8_t)((rotZ >> 8) & 0xFF);
        msg[23] = (uint8_t)((rotZ >> 16) & 0xFF);
        msg[24] = (uint8_t)((rotZ >> 24) & 0xFF);

        msg[25] = (uint8_t)(orientationX & 0xFF);
        msg[26] = (uint8_t)((orientationX >> 8) & 0xFF);
        msg[27] = (uint8_t)((orientationX >> 16) & 0xFF);
        msg[28] = (uint8_t)((orientationX >> 24) & 0xFF);

        msg[29] = (uint8_t)(orientationY & 0xFF);
        msg[30] = (uint8_t)((orientationY >> 8) & 0xFF);
        msg[31] = (uint8_t)((orientationY >> 16) & 0xFF);
        msg[32] = (uint8_t)((orientationY >> 24) & 0xFF);

        msg[33] = (uint8_t)(orientationZ & 0xFF);
        msg[34] = (uint8_t)((orientationZ >> 8) & 0xFF);
        msg[35] = (uint8_t)((orientationZ >> 16) & 0xFF);
        msg[36] = (uint8_t)((orientationZ >> 24) & 0xFF);

        msg[37] = (uint8_t)(orientationW & 0xFF);
        msg[38] = (uint8_t)((orientationW >> 8) & 0xFF);
        msg[39] = (uint8_t)((orientationW >> 16) & 0xFF);
        msg[40] = (uint8_t)((orientationW >> 24) & 0xFF);

        msg[41] = 0x40;

        // Serial.write(msg, 42);
        // Serial.println(linAccel.x);
        // Serial.println(linAccel.y);
        // Serial.println(linAccel.z);
        // Serial.println();

        delay(100);
    }
}
