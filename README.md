# cvSophus

Lie group types **SO(3)** and **SE(3)** implemented in C++ using **OpenCV** types (`cv::Quat`, `cv::Matx`, `cv::Vec`, `cv::Affine3`, etc.). Suitable for computer vision and robotics when you already depend on OpenCV.

## Features

- **SO(3)** — 3D rotation group: quaternion, rotation matrix, rotation vector (Rodrigues), Euler angles; exp/log, adjoint, left/right Jacobians.
- **SE(3)** — 3D rigid body group: rotation + translation; constructors from R+t, rvec+t, Euler+t, twist; exp/log, adjoint, left/right Jacobians.
- **Common utilities** — Axis rotations (X/Y/Z), orthogonalization via SVD, all using OpenCV matrices and vectors.

## Requirements

- **C++17**
- **OpenCV** (with `core` and `quaternion` support)
