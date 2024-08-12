/// @file
/// Special Euclidean group SE(3) - rotation and translation in 3d.

#pragma once

#include <opencv2/opencv.hpp>
#include "so3.hpp"


namespace cvSophus
{
    template <typename datatype>
    class SE3
    {
    public:
        /// @brief non-parametric constructor
        SE3() = default;

        /// @brief constructor from rotation matrix and translation
        /// @param R : rotation matrix
        /// @param T : translation
        SE3(const cv::Matx<datatype, 3, 3> &R, const cv::Vec<datatype, 3> &T) { data_ = cv::Affine3<datatype>(R, T); }

        /// @brief constructor from SO(3) and translation
        /// @param R : SO(3)
        /// @param T : translation
        SE3(const SO3<datatype> &R, const cv::Vec<datatype, 3> &T) { data_ = cv::Affine3<datatype>(R.log(), T); }

        /// @brief constructor from rotation vector and translation
        /// @param rvec : rotation vector
        /// @param T : translation
        SE3(const cv::Vec<datatype, 3> &rvec, const cv::Vec<datatype, 3> &T) { data_ = cv::Affine3<datatype>(rvec, T); }


        /// @brief constructor from rotation vector and translation
        /// @param t_rvec : the std::vector of translation and rotation vector
        SE3(const std::vector<datatype> &t_rvec)
        {
            assert(t_rvec.size() == 6);
            cv::Vec<datatype, 3> t(t_rvec[0], t_rvec[1], t_rvec[2]);
            cv::Vec<datatype, 3> rvec(t_rvec[3], t_rvec[4], t_rvec[5]);

            data_ = cv::Affine3<datatype>(rvec, t);
        }

        /// @brief  constructor from euler angle and translation
        /// @param euler : euler angle
        /// @param t : translation
        /// @param euler_type : the type of euler angle
        SE3(const cv::Vec<datatype, 3> &euler, const cv::Vec<datatype, 3> &t,
            const cv::QuatEnum::EulerAnglesType &euler_type)
        {
            SO3<datatype> tmp(euler, euler_type);
            data_ = cv::Affine3<datatype>(tmp.log(), t);
        }


        /// @brief constructor from the euler angle and translation of std::vector
        /// @param t_euler : the euler angle and translation of std::vector
        /// @param euler_type : the type fo euler angle
        SE3(const std::vector<datatype> &t_euler, const cv::QuatEnum::EulerAnglesType &euler_type)
        {
            cv::Vec<datatype, 3> t(t_euler[0], t_euler[1], t_euler[2]);
            cv::Vec<datatype, 3> euler(t_euler[3], t_euler[4], t_euler[5]);

            SO3<datatype> tmp(euler, euler_type);
            data_ = cv::Affine3<datatype>(tmp.log(), t);
        }

        SE3 operator*(const SE3 &rhs)
        {
            cv::Affine3<datatype> rhs_affine(rhs.rvec(), rhs.translation());
            cv::Affine3<datatype> result = data_ * rhs_affine;

            return SE3<datatype>(result.rvec(), result.translation());
        }


        /// @brief get translaton
        /// @return translaton
        cv::Vec<datatype, 3> translation() const { return data_.translation(); }


        /// @brief get rotation vector
        /// @return  rotation vector
        cv::Vec<datatype, 3> rvec() const { return data_.rvec(); }


        /// @brief generate the adjoint matrix of SE(3)
        /// @return adjoint matrix
        cv::Matx<datatype, 6, 6> adjoint() const
        {
            cv::Matx<datatype, 3, 3> R = data_.rotation();
            cv::Matx<datatype, 3, 3> zero = cv::Matx<T, 3, 3>::zeros();

            cv::Matx<datatype, 3, 6> upper;
            cv::hconcat(R, SO3<T>::skew(data_.translation() * R), upper);
            cv::Matx<datatype, 3, 6> lower;
            cv::hconcat(zero, R, lower);

            cv::Matx<datatype, 6, 6> result;
            cv::vconcat(upper, lower, result);

            return result;
        }

        /// @brief generate the Derivative by left perturbation
        /// @param p : the point of 3D space
        /// @return jacobian matrix
        cv::Matx<datatype, 4, 6> left_jacobian(const cv::Vec<datatype, 3> &p)
        {
            cv::Matx<datatype, 3, 3> left = cv::Matx<datatype, 3, 3>::eye();
            cv::Matx<datatype, 3, 3> right = -SO3<datatype>::skew(data_.rotation() * p + data_.translation());

            cv::Matx<datatype, 3, 6> upper;
            cv::hconcat(left, right, upper);

            cv::Matx<datatype, 1, 6> lower = cv::Matx<datatype, 1, 6>::zeros();

            cv::Matx<datatype, 4, 6> result;
            cv::vconcat(upper, lower, result);

            return result;
        }

        /// @brief generate the Derivative by right perturbation
        /// @param p : the point of 3D space
        /// @return jacobian matrix
        cv::Matx<datatype, 4, 6> right_jacobian(const cv::Vec<datatype, 3> &p)
        {
            cv::Matx<datatype, 3, 3> left = data_.rotation();
            cv::Matx<datatype, 3, 3> right = -left * SO3<datatype>::skew(p);

            cv::Matx<datatype, 3, 6> upper;
            cv::hconcat(left, right, upper);

            cv::Matx<datatype, 1, 6> lower = cv::Matx<datatype, 1, 6>::zeros();

            cv::Matx<datatype, 4, 6> result;
            cv::vconcat(upper, lower, result);

            return result;
        }

        /// @brief log map(to twist)
        /// @return the Lie Algebra of SE(3) (twist)
        cv::Vec<datatype, 6> log() const
        {
            cv::Vec<datatype, 3> rvec = data_.rvec();

            cv::Vec<datatype, 3> t = data_.translation();
            cv::Matx<datatype, 3, 3> J = jacobian(rvec, 1e-8);
            cv::Vec<datatype, 3> v = J.solve(t, cv::DECOMP_SVD);

            return {v(0), v(1), v(2), rvec(0), rvec(1), rvec(2)};
        }

        cv::Matx<datatype, 4, 4> matrix() const
        {
            return data_.matrix();
        }

        /// @brief to the std::vector of rotation vector and translation
        /// @return the std::vector of rotation vector and translation
        std::vector<datatype> to_stdvector_t_rvec()
        {
            cv::Vec<datatype, 3> rvec = data_.rvec();
            cv::Vec<datatype, 3> t = data_.translation();

            return {t(0), t(1), t(2), rvec(0), rvec(1), rvec(2)};
        }

        /// @brief Exponential Map
        /// @param w : the Lie Algebra of SE(3) (twist)
        /// @return SE(3)
        static SE3 exp(const cv::Vec<datatype, 6> &rho)
        {
            cv::Vec<datatype, 3> v(rho(0), rho(1), rho(2));
            cv::Vec<datatype, 3> w(rho(3), rho(4), rho(5));

            cv::Vec<datatype, 3> t = jacobian(w, 1e-8) * v;

            return SE3<T>(w, t);
        }

        /// @brief generate jacobian matrix(to generate the Lie Algerba of SE(3))
        /// @param w : the Lie Algebra of SO(3) (rotaton vector)
        /// @return jacobian matrix
        static cv::Matx<datatype, 3, 3> jacobian(const cv::Vec<datatype, 3> &w, const double &eps)
        {
            double theta_ = cv::norm(w);

            cv::Matx<datatype, 3, 3> J = cv::Matx<datatype, 3, 3>::eye() + ((1 - std::cos(theta_)) / (theta_ * theta_)) * SO3<datatype>::skew(w) +
                                  ((theta_ - std::sin(theta_)) / (theta_ * theta_ * theta_)) * SO3<datatype>::skew(w) * SO3<datatype>::skew(w);

            // To numerical stability
            if (theta_ < eps)
            {
                J = cv::Matx<datatype, 3, 3>::eye() + 0.5 * SO3<datatype>::skew(w) +
                    (1.f + 1.f / 6.0) * SO3<datatype>::skew(w) * SO3<datatype>::skew(w);
            }

            return J;
        }

    private:
        cv::Affine3<datatype> data_;
    };

    template <typename datatype>
    std::ostream& operator<<(std::ostream& out, const SE3<datatype> &data)
    {
        out << data.matrix();
        return out;
    }

    using SE3f = SE3<float>;
    using SE3d = SE3<double>;
}