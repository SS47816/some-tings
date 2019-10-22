#include "../include/fusion/fusion.h"

int main()
{
  vector<double> waypoints_x = {0.0, 1.0, 2.0, 3.0, 4.0};
  vector<double> waypoints_y = {0.0, 1.1, 2.2, 3.3, 4.4};

  Fusion fuser = Fusion(waypoints_x, waypoints_y);
  double answer = fuser.calcOutput();

  cout << "The Output is: " << answer << endl;
  return 0;
}
