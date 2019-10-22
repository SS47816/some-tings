#include "../include/sensor_fusion.h"

#define BASELINK_TO_CAMERA 2.2 // in [m]

using std::cout;
using std::endl;
using std::max;
using std::vector;

constexpr double pi() { return M_PI; }
double deg2rad(double angle) 
{ 
    return angle * pi() / 180; 
}
double rad2deg(double angle) { return angle / pi() * 180; }

vector<double> SensorFusion::getPolarCoordinate(double ob_x, double ob_y, double car_x, double car_y, double car_yaw)
{
    // Compute lidar obstacles into camera polar coordinate
    double camera_x = car_x + BASELINK_TO_CAMERA * cos(car_yaw);
    double camera_y = car_y + BASELINK_TO_CAMERA * sin(car_yaw);
    double r = sqrt(pow(ob_x - camera_x, 2) + pow(ob_y - camera_y, 2));

    double theta = atan2(ob_y - camera_y, ob_x - camera_x) - car_yaw;

    cout << "camera_x: " << camera_x << ", camera_y: " << camera_y
         << ", r: " << r << ", theta: " << theta << endl;

    return {r, theta};
}

int main()
{
    // Camera Properties
    const double fov = 78; //in deg
    const vector<double> img_shape = {640, 480};
    double tolerance = 0.035; // in radians
    int k = 0;
    bool flag = 0;

    // double x, y, current_x, current_y, current_yaw, l;
    double camera_x, camera_y, r, theta;

    vector<double> radius;
    vector<double> x = {3, 3};
    vector<double> y = {5, 4};
    vector<double> current_x = {0, 2};
    vector<double> current_y = {0, 1};
    vector<double> current_yaw = {M_PI / 2, -M_PI / 2};
    vector<double> lidar_theta, camera_theta;
    vector<vector<double>> fused_obstacles;

    vector<double> img_x = {500, 29};
    vector<double> img_y = {};

    for (int i = 0; i < x.size(); i++)
    {
        // Compute lidar obstacles into camera polar coordinate
        camera_x = current_x[i] + BASELINK_TO_CAMERA * cos(current_yaw[i]);
        camera_y = current_y[i] + BASELINK_TO_CAMERA * sin(current_yaw[i]);
        r = sqrt(pow(x[i] - camera_x, 2) + pow(y[i] - camera_y, 2));
        radius.push_back(r);
        theta = atan2(y[i] - camera_y, x[i] - camera_x) - current_yaw[i];
        lidar_theta.push_back(theta);

        cout << "camera_x: " << camera_x << ", camera_y: " << camera_y
             << ", r: " << r << ", theta: " << theta << endl;
    }

    for (int i = 0; i < img_x.size(); i++)
    {
        // TODO: compute image obstacles' theta angle
        double alpha = deg2rad(fov / 2);

        double img_theta = atan(tan(alpha) * ((img_shape[0] / 2) - img_x[i]) / (img_shape[0] / 2));
        camera_theta.push_back(img_theta);

        cout << "alpha: " << alpha << ", img_theta: " << img_theta << endl;
    }

    for (int i = 0; i < lidar_theta.size(); i++)
    {
        for (int j = 0; j < camera_theta.size(); j++)
        {
            if (abs(lidar_theta[i] - camera_theta[j]) <= tolerance)
            {
                fused_obstacles.push_back({x[i], y[i], lidar_theta[i], radius[i]}); // to include camera data
                cout << fused_obstacles[k][0] << " " << fused_obstacles[k][1] << " " << fused_obstacles[k][2] << " " << fused_obstacles[k][3] << endl;
                k++;
                flag = 1;
                break;
            }
        }

        if (flag == 0)
        {
            fused_obstacles.push_back({x[i], y[i], lidar_theta[i], radius[i]}); // to include camera data
            cout << fused_obstacles[k][0] << " " << fused_obstacles[k][1] << " " << fused_obstacles[k][2] << " " << fused_obstacles[k][3] << endl;
            k++;
        }

        flag = 0;
    }

    return 0;
}