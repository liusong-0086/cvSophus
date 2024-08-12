#include "se3.hpp"


int main()
{
    cv::Vec3d rvec(0.f, 0.f, CV_PI / 4);
    cv::Matx33d R;
    cv::Rodrigues(rvec, R);

    cv::Vec3d t(1, 2, 3);

    cvSophus::SE3<double> se3(R, t);

    std::cout<<"SE3 log: "<<se3.log().t()<<std::endl;

    return 0;
}