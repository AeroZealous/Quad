/*
  AeroQuad v3.0 - March 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#include <Wire.h>
#include <Device_I2C.h>
#include <GlobalDefined.h>
#include <AQMath.h>
#include <Accelerometer_BMA180.h>

unsigned long timer;

float avg = 0;
void setup() {
  
  Serial.begin(115200);
//  /Serial.println("Accelerometer library test (BMA180)");

  Wire.begin();
  
  initializeAccel();
  computeAccelBias();
  // changing the scale factor is necessary to get non-0 values from this test sketch
  // don't worry about how accurate the data is at this point, just make sure it changes as you move the board
  accelScaleFactor[XAXIS]=accelScaleFactor[YAXIS]=accelScaleFactor[ZAXIS]=-0.038;

}



#define window_size 20

float temp, ds_bufferz[window_size], ds_bufferx[window_size], ds_buffery[window_size], ds_buffer_net[window_size];
float digital_smooth(float *temp_buffer, float new_val) {
  static int i = 0, balance = 0;
  if(balance <= window_size)
	balance++;
  temp_buffer[i] = new_val;
  i = ((i + 1) % window_size);
  float buffer[balance];
  for(int k = 0; k < balance; k++) {
    buffer[k] = temp_buffer[k];
  }
  for(int k = 0; k < balance; k++) {
    for(int j = k + 1; j < balance; j++) {
      if(buffer[k] > buffer[j]) {
        temp = buffer[j];
        buffer[j] = buffer[k];
        buffer[k] = temp;
      }
    }
  }
  int lbound, ubound;
  float avg = 0;
  lbound = (balance*15)/100;
  ubound = balance - lbound;
  for(int k = lbound; k <= ubound; k++) {
	avg += buffer[k];
  }    
  return (avg / (ubound -lbound));
}

float accel_z, accel_x, accel_y, accel_net;
void loop() {
  
  if((millis() - timer) > 20) {
    timer = millis();
    measureAccel();
    accel_net = sqrt(meterPerSecSec[ZAXIS]*meterPerSecSec[ZAXIS] + meterPerSecSec[XAXIS]*meterPerSecSec[XAXIS] + meterPerSecSec[YAXIS]*meterPerSecSec[YAXIS]);		//accel_net acceleration
    accel_z = digital_smooth(ds_bufferz, meterPerSecSec[ZAXIS]);
    accel_x = digital_smooth(ds_bufferx, meterPerSecSec[XAXIS]);
    accel_y = digital_smooth(ds_buffery, meterPerSecSec[YAXIS]);
    accel_net = sqrt(accel_z*accel_z + accel_x*accel_x + accel_y*accel_y);
    accel_net = digital_smooth(ds_buffer_net, accel_net);
   
    Serial.print("Roll: ");
    Serial.print(accel_x);
    Serial.print("\tPitch: ");
    Serial.print(accel_y);
    Serial.print("\tYaw: ");
    Serial.print(accel_z);
  }
}
