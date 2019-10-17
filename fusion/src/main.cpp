//#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>

using std::cout;
using std::endl;
using std::vector;

int main()
{
    // double x, y, current_x, current_y, current_yaw, l;
    double camera_x, camera_y, r, theta;

    vector<double> x = {3};
    vector<double> y = {5};
    vector<double> current_x = {0};
    vector<double> current_y = {0};
    vector<double> current_yaw = {M_PI / 2};
    vector<double> l = {2};

    for (int i = 0; i < x.size(); ++i)
    {
        camera_x = current_x[i] + l[i] * cos(current_yaw[i]);
        camera_y = current_y[i] + l[i] * sin(current_yaw[i]);
        r = sqrt(pow(x[i] - camera_x, 2) + pow(y[i] - camera_y, 2));
        theta = atan2(y[i] - camera_y, x[i] - camera_x) - current_yaw[i];

        cout << camera_x << " " << camera_y << " " << r << " " << theta << endl;
    }
    
    return 0;
}
