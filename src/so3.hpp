/// @file
/// Special Euclidean group SO(3) - rotation and translation in 3d.

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/quaternion.hpp>

namespace cvSophus
{
    template <typename T>
    class SO3
    {
    public:
        /// @brief non-parametric constructor
        SO3() = default;

        /// @brief constructor from quaternion
        /// @param q : quaternion
        SO3(const cv::Quat<T> &q) : data_(q) {}

        /// @brief constructor from rotation matrix
        /// @param R : rotation matrix
        SO3(const cv::Matx<T, 3, 3> &R) { data_ = data_.createFromRotMat(R); }


        /// @brief constructor from rotation vector
        /// @param rvec : rotation vector
        SO3(const cv::Vec<T, 3> &rvec)
        {
            // if Quaternions are directly converted rotation vector, rotation vector will be singular.
            cv::Matx<T, 3, 3> R;
            cv::Rodrigues(rvec, R);
            data_ = data_.createFromRotMat(R);
        }


        /// @brief constructor from euler angle
        /// @param euler : euler angle
        /// @param euler_type : euler angel type
        SO3(const cv::Vec<T, 3> &euler, const cv::QuatEnum::EulerAnglesType &euler_type)
        {
            data_ = data_.createFromEulerAngles(euler, euler_type);
        }

        SO3<T> operator*(const SO3<T> &rhs)
        {
            return SO3<T>(data_.toRotMat3x3() * rhs.matrix());
        }

        /// @brief generate the adjoint matrix of SO(3)
        /// @return adjoint matrix 
        cv::Matx<T, 3, 3> adjoint() { return data_; }


        /// @brief generate the Derivative by left perturbation
        /// @param p : the point of 3D space
        /// @return jacobian matrix
        cv::Matx<T, 3, 3> left_jacobian(const cv::Vec<T, 3> &p)
        {
            return -skew(data_.toRotMat3x3() * p);
        }

        /// @brief generate the Derivative by right perturbation
        /// @param p : the point of 3D space
        /// @return jacobian matrix
        cv::Matx<T, 3, 3> right_jacobian(const cv::Vec<T, 3> &p)
        {
            return -data_.toRotMat3x3() * skew(p);
        }

        /// @brief log map(to rotation vector)
        /// @return the Lie Algebra of SO(3) (rotation vector)
        cv::Vec<T, 3> log() const
        {
            cv::Matx<T, 3, 3> R = matrix();

            cv::Vec<T, 3> rvec;
            cv::Rodrigues(R, rvec);

            return rvec;
        }

        /// @brief get rotation matrix
        /// @return rotation matrix
        cv::Matx<T, 3, 3> matrix() const { return data_.toRotMat3x3(); }


        /// @brief get euler angle
        /// @param euler_type : euler angle type
        /// @return euler angle by rad
        cv::Vec<T, 3> to_euler_angle(const cv::QuatEnum::EulerAnglesType &euler_type) const
        {
            return data_.toEulerAngles(euler_type);
        }

        /// @brief get quterntion
        /// @return quterntion
        cv::Quat<T> to_querntion() const { return data_ };

        /// @brief Exponential Map
        /// @param w : the Lie Algebra of SO(3) (rotation vector)
        /// @return SO(3)
        static SO3 exp(const cv::Vec<T, 3> &w) { return SO3(w); }

        /// @brief get skew matrix
        /// @param w : the Lie Algebra of SO(3) (rotation vector)
        /// @return skew matrix
        static cv::Matx<T, 3, 3> skew(const cv::Vec<T, 3> &w)
        {
            cv::Matx<T, 3, 3> skew_matrix;
            skew_matrix << 0, -w(2), w(1),
                w(2), 0, -w(0),
                -w(1), w(0), 0;

            return skew_matrix;
        }

    private:
        cv::Quat<T> data_;
    };

    template<typename T>
    std::ostream& operator<<(std::ostream& out, const SO3<T> &data)
    {
        out << data.matrix();
        return out;
    }

    using SO3d = SO3<double>;
    using SO3f = SO3<float>;
}
