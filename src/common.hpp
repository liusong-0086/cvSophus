#pragma once

#include <opencv2/opencv.hpp>
#include "so3.hpp"
#include "se3.hpp"

namespace cvSophus
{
    /// @brief orthogonalized matrix by SVD
    /// @tparam T : data type
    /// @tparam dim : dimension
    /// @param matrix : matrix to be orthogonalized
    /// @return orthogonalized matrix
    template <typename T, int dim>
    cv::Matx<T, dim, dim> orthogonalized_matrix(const cv::Matx<T, dim, dim> &matrix)
    {
        cv::Matx<T, dim, dim> U, S, Vt;
        cv::SVD::compute(matrix, S, U, Vt);

        cv::Matx<T, dim, dim> D = cv::Matx<T, dim, dim>::eye();
        D(dim - 1, dim - 1) = cv::determinant(U * Vt);

        cv::Matx<T, dim, dim> R = U * D * Vt;

        return R;
    }

    /// @brief generate 3x3 the X of rotation matrix
    /// @tparam data type
    /// @param theta : ratation angle
    /// @return 3x3 rotation matrix
    template <typename T>
    SO3<T> rotation33_x(const T &theta)
    {
        cv::Vec<T, 3> rvec(theta, 0.f, 0.f);
        return SO3<T>(rvec);
    }

    /// @brief generate 3x3 the Y of rotation matrix
    /// @tparam data type
    /// @param theta : ratation angle
    /// @return 3x3 rotation matrix
    template <typename T>
    SO3<T> rotation33_y(const T &theta)
    {
        cv::Vec<T, 3> rvec(0.f, theta, 0.f);
        return SO3<T>(rvec);
    }

    /// @brief generate 3x3 the Z of rotation matrix
    /// @tparam data type
    /// @param theta : ratation angle
    /// @return 3x3 rotation matrix
    template <typename T>
    SO3<T> rotation33_z(const T &theta)
    {
        cv::Vec<T, 3> rvec(0.f, 0.f, theta);
        return SO3<T>(rvec);
    }

    /// @brief generate 4x4 the X of rotation matrix
    /// @tparam data type
    /// @param theta : ratation angle
    /// @return 4x4 rotation matrix
    template <typename T>
    SE3<T> rotation44_x(const T &theta)
    {
        cv::Vec<T, 3> rvec(theta, 0.f, 0.f);
        return SE3<T>(rvec, cv::Vec<T, 3>::zeros());
    }

    /// @brief generate 4x4 the Y of rotation matrix
    /// @tparam data type
    /// @param theta : ratation angle
    /// @return 4x4 rotation matrix
    template <typename T>
    SE3<T> rotation44_y(const T &theta)
    {
        cv::Vec<T, 3> rvec(0.f, theta, 0.f);
        return SE3<T>(rvec, cv::Vec<T, 3>::zeros());
    }

    /// @brief generate 4x4 the Z of rotation matrix
    /// @tparam data type
    /// @param theta : ratation angle
    /// @return 4x4 rotation matrix
    template <typename T>
    SE3<T> rotation44_z(const T &theta)
    {
        cv::Vec<T, 3> rvec(0.f, 0.f, theta);
        return SE3<T>(rvec, cv::Vec<T, 3>::zeros());
    }
}