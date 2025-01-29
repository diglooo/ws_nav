/*
  ===============================================
  Example sketch for CurieIMU library for Intel(R) Curie(TM) devices.
  Copyright (c) 2015 Intel Corporation.  All rights reserved.

  Based on I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050
  class by Jeff Rowberg: https://github.com/jrowberg/i2cdevlib

  ===============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2011 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

#include "CurieIMU.h"
int ax, ay, az; // accelerometer values
int gx, gy, gz; // gyrometer values

const int ledPin = 13;      // activity LED pin
boolean blinkState = false; // state of the LED

void setup()
{
  // configure Arduino LED for activity indicator
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 1);
  Serial.begin(115200); // initialize Serial communication
  while (!Serial)
    ; // wait for the serial port to open

  // initialize device
  CurieIMU.begin();

  // verify connection
  if (!CurieIMU.begin())
  {
    while (1)
    {
      delay(100);
      Serial.println("0,0,0,0,0,0");
    }
  }

  CurieIMU.setGyroRate(50);          // Hz
  CurieIMU.setAccelerometerRate(50); // Hz
  CurieIMU.setGyroRange(500);        // deg/s
  CurieIMU.setAccelerometerRange(2); // G

  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  CurieIMU.accelerometerOffsetEnabled();
  CurieIMU.gyroOffsetEnabled();

  digitalWrite(ledPin, 0);
}

unsigned long lastMillis = 0;
void loop()
{
  if (millis() - lastMillis > 50)
  {
    digitalWrite(ledPin,1);

    lastMillis = millis();
    CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);

    // display tab-separated accel/gyro x/y/z values
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.println(gz);

    // blink LED to indicate activity
    //blinkState = !blinkState;
    digitalWrite(ledPin, 0);
  }
}
