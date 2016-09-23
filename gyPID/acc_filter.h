#include <Wire.h>
#include <Device_I2C.h>
#include <GlobalDefined.h>
#include <AQMath.h>
#include <Accelerometer_BMA180.h>


float avg = 0;
//--------------to remove noise, not impulse!----------------------------------//

#define window_size 3
float temp, med_bufferz[window_size], med_bufferx[window_size], med_buffery[window_size], med_buffer_net[window_size];
float accel_x_prev, accel_y_prev, accel_z_prev;

float sort_med(float *temp_buffer) {

  float buffer[window_size];
  for(int i = 0; i < window_size; i++) {
    buffer[i] = temp_buffer[i];
  }
  for(int i = 0; i < window_size; i++) {
    for(int j = i + 1; j < window_size; j++) {
      if(buffer[i] > buffer[j]) {
        temp = buffer[j];
        buffer[j] = buffer[i];
        buffer[i] = temp;
      }
    }
  }
  return buffer[(window_size / 2)];

}



//replace input with some seed value if you want
void initiate(float input, float *temp_buffer) {

  for(int i = 0; i < window_size; i++) {
	temp_buffer[i] = input;
  }

}


void pushupdate(float *temp_buffer, float input) {

  for(int i = 0; i < (window_size - 1); i++) {
	temp_buffer[i] = temp_buffer[i+1];
  }
  temp_buffer[(window_size - 1)] = input;

}

int check = 0;
float accel_z, accel_x, accel_y, accel_net;

