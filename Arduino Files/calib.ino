#include "I2Cdev.h"
#include "MPU9250.h"
#include "Wire.h"

// Configuration
int buffersize = 1000;
int acel_deadzone = 8;
int giro_deadzone = 1;

// MPU6050 object
MPU9250 accelgyro;

int ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
long buff_ax, buff_ay, buff_az, buff_gx, buff_gy, buff_gz;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  accelgyro.initialize();
  
  // Check MPU6050 connection
  if (accelgyro.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  // Perform initial sensor reading
  meansensors();
}

void loop() {
  calibration();
  // The rest of the code or loop to handle calibrated sensor readings
}

void meansensors() {
  long i = 0;
  buff_ax = buff_ay = buff_az = buff_gx = buff_gy = buff_gz = 0;

  while (i < (buffersize + 100)) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (i > 100 && i <= (buffersize + 100)) { // First 100 measures are discarded
      buff_ax += ax;
      buff_ay += ay;
      buff_az += az;
      buff_gx += gx;
      buff_gy += gy;
      buff_gz += gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); // Needed so we don't get repeated measures
  }
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;
  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;

  while (1) {
    int ready = 0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("Calibrating...");

    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset -= mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset -= mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset += (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset -= mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset -= mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset -= mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
}
