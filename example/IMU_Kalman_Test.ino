#include <MPU6050_tockn.h>
#include <TinyEKF.hpp>
#include <Wire.h>

MPU6050 mpu6050(Wire);

TinyEKF myekf(AX6_ONLY);
FTYPE last_time = 0;

long timer = 0;

void setup() {
  Serial.begin(9600);
  //Serial.println("=== Start ===");
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  // EKF Test
  myekf.Set_Propagation_Noise(1e-3, 1e-3, 1e-3,
                        1e-3, 1e-3, 1e-3,
                        1e-3, 1e-3, 1e-3, 1e-3);
  myekf.Set_Observation_Noise(1e-3, 1e-3, 1e-3,
                        1e-3, 1e-3, 1e-3, 1e-3);
  myekf.Set_References(0, 0, -1);            
  
}

void loop() {
  
  // Take IMU Measurement
  mpu6050.update();
  FTYPE ax = mpu6050.getAccX();
  FTYPE ay = mpu6050.getAccY();
  FTYPE az = mpu6050.getAccZ();
  FTYPE gx = mpu6050.getGyroX();
  FTYPE gy = mpu6050.getGyroY();
  FTYPE gz = mpu6050.getGyroZ();
  // Take dt
  
  FTYPE now = (FTYPE) millis()/1000;
  FTYPE dt = now - last_time;
  last_time = now;
  // Predict Step For EKF
  myekf.EKF_predict(dt);
  // Set Measurement
  myekf.Set_IMU_Measurements(ax, ay, az, gx, gy, gz);
  // Update
  myekf.EKF_update();
  // Get Filtered Value
  FTYPE wx, wy, wz;
  myekf.Get_Angular_Velocity(&wx, &wy, &wz);
  
  Serial.print("=== Before gyro x:");
  Serial.print(gx,4);
  //Serial.print(",");
  Serial.print(", y:");
  Serial.print(gy,4);
  //Serial.print(",");
  Serial.print(", z:");
  Serial.print(gz,4);
  
  //Serial.print(",");
  
  Serial.print(", After gyro x:");
  Serial.print(wx,4);
  //Serial.print(",");
  Serial.print(", y:");
  Serial.print(wy,4);
  //Serial.print(",");
  Serial.print(", z:");
  Serial.print(wz,4);
  Serial.print(",");
  //Serial.println("");
  Serial.println(" ===");
}
