#ifndef SENSOR_FUSION_H_
#define SENSOR_FUSION_H_

//#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>


class SensorFusion
{
 public:


 private:
  constexpr double pi();
  double deg2rad(double angle);
  double rad2deg(double angle);
  vector<double> getPolarCoordinate(double ob_x, double ob_y, double car_x, double car_y, double car_yaw);

}

#endif // SENSOR_FUSION_H_