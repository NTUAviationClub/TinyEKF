// #include "ekf.h"
// #include "quaternion_6axis.h"
// #include "quaternion_9axis.h"
#include <TinyEKF.hpp>
#include <iostream>

int main() {
  TinyEKF myekf(AX6_ONLY);
  myekf.Set_Propagation_Noise(1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3,
                              1e-3, 1e-3);
  myekf.Set_Observation_Noise(1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3);
  myekf.Set_References(0, 0, -1);
  FTYPE ax = 0;
  FTYPE ay = 0;
  FTYPE az = 1;
  FTYPE gx = 0;
  FTYPE gy = 0;
  FTYPE gz = 0;
  // Predict Step For EKF
  myekf.EKF_predict(0.1);
  // Set Measurement
  myekf.Set_IMU_Measurements(ax, ay, az, gx, gy, gz);
  // Update
  myekf.EKF_update();
  // Get Filtered Value
  FTYPE wx, wy, wz;
  myekf.Get_Angular_Velocity(&wx, &wy, &wz);
  std::cout << "wx: " << wx << std::endl;
  return 0;
}
