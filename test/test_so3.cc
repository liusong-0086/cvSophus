#include "so3.hpp"
#include "common.hpp"

using namespace cvSophus;

int main()
{
    SO3f original(cv::Vec3f(0.f, 0.f, CV_PI / 4));
    SO3f trans_x = rotation33_z<float>(CV_PI / 4);
    SO3f new_1 = original * trans_x;
    std::cout<<"Original a:"<<original<<std::endl;
    std::cout<<"New a after trans_x:"<<new_1<<std::endl;

    return 0;
}