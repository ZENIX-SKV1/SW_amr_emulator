#include "localizer.h"
#include <iostream>

int main() {
    Localizer loc(0.6, 0.1); // wheel_base, wheel_radius
    loc.setInitialPose(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; ++i) 
    {
        loc.update(50.0, 60.0, 0.1); // left_rpm, right_rpm, dt
        double x, y, theta;
        loc.getPose(x, y, theta);
        std::cout << "Pose: " << x << ", " << y << ", " << theta << std::endl;
    }
}