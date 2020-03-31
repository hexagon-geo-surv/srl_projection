/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab, Imperial College London
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: March, 2020
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/OusterLidar.hpp
 * @brief Header implementation file for the OusterLidar class.
 * @author Stefan Leutenegger
 */

#include <Eigen/Geometry>

// \brief Main namespace of this package.
namespace srl {
// \brief Namespace for camera-related functionality.
namespace projection {

OusterLidar::OusterLidar(int imageWidth, int imageHeight, const Eigen::VectorXd & beamAzimuthAngles,
                         const Eigen::VectorXd & beamElevationAngles)
    : ProjectionBase(imageWidth, imageHeight),
      beamAzimuthAngles_(beamAzimuthAngles), beamElevationAngles_(beamElevationAngles)
{
}

//////////////////////////////////////////
// Methods to project points

// Projects a Euclidean point to a 2d image point (projection).
ProjectionStatus OusterLidar::project(
    const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint) const
{
  // handle singularity
  if (point.norm() < 1.0e-12) {
    return ProjectionStatus::Invalid;
  }

  // compute azimuth and elevation angles (projection) [rad]
  const double R = sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]);
  const double azimuth = (2.0 * M_PI - std::atan2(point[1], point[0])) * 360.0 / (2 * M_PI);
  const double elevation = std::asin(point[2]/R) * 360.0 / (2 * M_PI);

  // check bounds
  if(elevation > beamElevationAngles_[0]) {
    return ProjectionStatus::OutsideImage;
  }
  if(elevation < beamElevationAngles_[beamElevationAngles_.rows()-1]) {
    return ProjectionStatus::OutsideImage;
  }
  // find the right row
  /// \todo make this more elegant with binary search
  bool found = false;
  int i = 1;
  for(; i < beamElevationAngles_.rows(); ++i) {
    if(double(beamElevationAngles_[i]) < elevation) {
      found = true;
      break;
    }
  }
  if(!found) {
    return ProjectionStatus::OutsideImage;
  }

  // for interpolation
  const double r = (elevation-beamElevationAngles_[i])
      /(beamElevationAngles_[i-1] - beamElevationAngles_[i]);
  const double azimuthOffset = (1.0 - r) * beamAzimuthAngles_[i] + r * beamAzimuthAngles_[i-1];

  // scale and offset
  (*imagePoint)[0] = (azimuth - azimuthOffset)/360.0 * imageWidth_;
  (*imagePoint)[1] = i - r;

  // azimuthal wrap-around
  if((*imagePoint)[0]<-0.5) {
    (*imagePoint)[0] = (*imagePoint)[0] + imageWidth_;
  }
  if((*imagePoint)[0]>imageWidth_-0.5) {
    (*imagePoint)[0] = (*imagePoint)[0] - imageWidth_;
  }

  // checks
  if (ProjectionBase::isMasked(*imagePoint)) {
    return ProjectionStatus::Masked;
  }
  return ProjectionStatus::Successful;
}

// Projects a Euclidean point to a 2d image point (projection).
ProjectionStatus OusterLidar::project(
    const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint,
    Eigen::Matrix<double, 2, 3> * pointJacobian,
    Eigen::Matrix2Xd * intrinsicsJacobian) const
{
  // handle singularity
  if (point.norm() < 1.0e-12) {
    return ProjectionStatus::Invalid;
  }

  // compute azimuth and elevation angles (projection) [rad]
  const double R = sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]);
  const double azimuth = (2.0 * M_PI - std::atan2(point[1], point[0])) * 360.0 / (2 * M_PI);
  const double elevation = std::asin(point[2]/R) * 360.0 / (2 * M_PI);

  // check bounds
  if(elevation > beamElevationAngles_[0]) {
    return ProjectionStatus::OutsideImage;
  }
  if(elevation < beamElevationAngles_[beamElevationAngles_.rows()-1]) {
    return ProjectionStatus::OutsideImage;
  }
  // find the right row
  /// \todo make this more elegant with binary search
  bool found = false;
  int i = 1;
  for(; i < beamElevationAngles_.rows(); ++i) {
    if(double(beamElevationAngles_[i]) < elevation) {
      found = true;
      break;
    }
  }
  if(!found) {
    return ProjectionStatus::OutsideImage;
  }

  // for interpolation
  const double scalingElevation = 1.0/(beamElevationAngles_[i-1] - beamElevationAngles_[i]);
  const double r = (elevation-beamElevationAngles_[i]) * scalingElevation;
  const double azimuthOffset = (1.0 - r) * beamAzimuthAngles_[i] + r * beamAzimuthAngles_[i-1];

  // scale and offset
  const double scalingAzimuth = imageWidth_/360.0;
  (*imagePoint)[0] = (azimuth - azimuthOffset) * scalingAzimuth;
  (*imagePoint)[1] = i - r;

  // azimuthal wrap-around
  if((*imagePoint)[0]<-0.5) {
    (*imagePoint)[0] = (*imagePoint)[0] + imageWidth_;
  }
  if((*imagePoint)[0]>imageWidth_-0.5) {
    (*imagePoint)[0] = (*imagePoint)[0] - imageWidth_;
  }

  // Jacobians
  if(pointJacobian) {
    pointJacobian->setZero();
    const double R2 = point[0]*point[0] + point[1]*point[1];
    const double sqrt_R2 = sqrt(R2);
    const double D2 = R2 + point[2]*point[2];
    //(*pointJacobian)(0,0) = -(point[1] * scalingAzimuth)/R2;
    //(*pointJacobian)(0,1) = (point[0] * scalingAzimuth)/R2;
    //(*pointJacobian)(0,2) = 0.0; /// \todo
    //(*pointJacobian)(1,0) = -scalingElevation*(point[0]*point[2])/(sqrt_R2*D2);
    //(*pointJacobian)(1,1) = -scalingElevation*(point[1]*point[2])/(sqrt_R2*D2);
    //(*pointJacobian)(1,2) = scalingElevation*sqrt_R2/D2;
    (*pointJacobian)(0,0) = imageWidth_*((point[1]*180.0)/(M_PI*R2)+(beamAzimuthAngles_[i]*point[0]*point[2]/sqrt_R2*180.0)/(M_PI*(beamElevationAngles_[i]-beamElevationAngles_[i-1])*D2)-(beamAzimuthAngles_[i-1]*point[0]*point[2]*1.0/sqrt_R2*180.0)/(M_PI*(beamElevationAngles_[i]-beamElevationAngles_[i-1])*D2))*(1.0/360.0);
    (*pointJacobian)(0,1) = imageWidth_*((point[0]*180.0)/(M_PI*R2)-(beamAzimuthAngles_[i]*point[1]*point[2]/sqrt_R2*180.0)/(M_PI*(beamElevationAngles_[i]-beamElevationAngles_[i-1])*D2)+(beamAzimuthAngles_[i-1]*point[1]*point[2]*1.0/sqrt_R2*180.0)/(M_PI*(beamElevationAngles_[i]-beamElevationAngles_[i-1])*D2))*(-1.0/360.0);
    (*pointJacobian)(0,2) = (imageWidth_*sqrt_R2*(beamAzimuthAngles_[i]-beamAzimuthAngles_[i-1])*(-0.5))/(M_PI*(beamElevationAngles_[i]-beamElevationAngles_[i-1])*D2);
    (*pointJacobian)(1,0) = (point[0]*point[2]*1.0/sqrt_R2*(-180.0))/(M_PI*(beamElevationAngles_[i]-beamElevationAngles_[i-1])*D2);
    (*pointJacobian)(1,1) = (point[1]*point[2]*1.0/sqrt_R2*(-180.0))/(M_PI*(beamElevationAngles_[i]-beamElevationAngles_[i-1])*D2);
    (*pointJacobian)(1,2) = (180.0*sqrt_R2)/(M_PI*(beamElevationAngles_[i]-beamElevationAngles_[i-1])*D2);

  }
  if(intrinsicsJacobian) {
    throw std::runtime_error("OusterLidar does not support intrinsics Jacobian");
  }

  // checks
  if (ProjectionBase::isMasked(*imagePoint)) {
    return ProjectionStatus::Masked;
  }
  return ProjectionStatus::Successful;
}

// Projects a Euclidean point to a 2d image point (projection).
ProjectionStatus OusterLidar::projectWithExternalParameters(
    const Eigen::Vector3d &, const Eigen::VectorXd &,
    Eigen::Vector2d *, Eigen::Matrix<double, 2, 3> *,
    Eigen::Matrix2Xd *) const
{
  throw std::runtime_error("external parameters projection for OusterLidar not implemented");
  return ProjectionStatus::Invalid;
}

// Projects Euclidean points to 2d image points (projection) in a batch.
void OusterLidar::projectBatch(
    const Eigen::Matrix3Xd & points, Eigen::Matrix2Xd * imagePoints,
    std::vector<ProjectionStatus> * stati) const
{
  const int numPoints = points.cols();
  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector3d point = points.col(i);
    Eigen::Vector2d imagePoint;
    ProjectionStatus status = project(point, &imagePoint);
    imagePoints->col(i) = imagePoint;
    if(stati)
      stati->push_back(status);
  }
}

// Projects a point in homogenous coordinates to a 2d image point (projection).
ProjectionStatus OusterLidar::projectHomogeneous(
    const Eigen::Vector4d & point, Eigen::Vector2d * imagePoint) const
{
  Eigen::Vector3d head = point.head<3>();
  if (point[3] < 0) {
    return project(-head, imagePoint);
  } else {
    return project(head, imagePoint);
  }
}

// Projects a point in homogenous coordinates to a 2d image point (projection).
ProjectionStatus OusterLidar::projectHomogeneous(
    const Eigen::Vector4d & point, Eigen::Vector2d * imagePoint,
    Eigen::Matrix<double, 2, 4> * pointJacobian,
    Eigen::Matrix2Xd * intrinsicsJacobian) const
{
  Eigen::Vector3d head = point.head<3>();
  Eigen::Matrix<double, 2, 3> pointJacobian3;
  ProjectionStatus status;
  if (point[3] < 0) {
    status = project(-head, imagePoint, &pointJacobian3, intrinsicsJacobian);
  } else {
    status = project(head, imagePoint, &pointJacobian3, intrinsicsJacobian);
  }
  pointJacobian->template bottomRightCorner<2, 1>() = Eigen::Vector2d::Zero();
  pointJacobian->template topLeftCorner<2, 3>() = pointJacobian3;
  return status;
}

// Projects a point in homogenous coordinates to a 2d image point (projection).
ProjectionStatus OusterLidar::projectHomogeneousWithExternalParameters(
    const Eigen::Vector4d &, const Eigen::VectorXd &,
    Eigen::Vector2d *, Eigen::Matrix<double, 2, 4> *,
    Eigen::Matrix2Xd *) const
{
  throw std::runtime_error("intrinsics Jacobian for OusterLidar not implemented");
  return ProjectionStatus::Invalid;
}

// Projects points in homogenous coordinates to 2d image points (projection) in a batch.
void OusterLidar::projectHomogeneousBatch(
    const Eigen::Matrix4Xd & points, Eigen::Matrix2Xd * imagePoints,
    std::vector<ProjectionStatus> * stati) const
{
  const int numPoints = points.cols();
  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector4d point = points.col(i);
    Eigen::Vector2d imagePoint;
    ProjectionStatus status = projectHomogeneous(point, &imagePoint);
    imagePoints->col(i) = imagePoint;
    if(stati)
      stati->push_back(status);
  }
}

//////////////////////////////////////////
// Methods to backproject points

// Back-project a 2d image point into Euclidean space (direction vector).
bool OusterLidar::backProject(
    const Eigen::Vector2d & imagePoint, Eigen::Vector3d * direction) const
{
  // adapted from
  // https://github.com/ouster-lidar/ouster_example/blob/master/ouster_client/src/os1_util.cpp

  // compute elevation and azimuth angles from pixel coordinates with interpolation
  const uint32_t v0 = std::floor(imagePoint[1]);
  const uint32_t v1 = std::ceil(imagePoint[1]);
  const double r = imagePoint[1] - v0;

  // azimuth angle with interpolated offset
  const double azimuth0 = 2.0 * M_PI * imagePoint[0] / double(imageWidth_);
  const double azimuth = ((1.0-r) * beamAzimuthAngles_[v0] +  r * beamAzimuthAngles_[v1])
      * 2.0 * M_PI / 360.0 + azimuth0;

  // interpolate elevation angle
  double elevation = ((1.0-r) * beamElevationAngles_[v0] + r * beamElevationAngles_[v1])
      * 2.0 * M_PI / 360.0;

  // project as ray
  (*direction)[0] = std::cos(elevation) * std::cos(azimuth);
  (*direction)[1] = -std::cos(elevation) * std::sin(azimuth);
  (*direction)[2] = std::sin(elevation);

  return true;
}

// Back-project a 2d image point into Euclidean space (direction vector).
inline bool OusterLidar::backProject(
    const Eigen::Vector2d & imagePoint, Eigen::Vector3d * direction,
    Eigen::Matrix<double, 3, 2> * pointJacobian) const
{
  // adapted from
  // https://github.com/ouster-lidar/ouster_example/blob/master/ouster_client/src/os1_util.cpp

  // compute elevation and azimuth angles from pixel coordinates with interpolation
  const uint32_t v0 = std::floor(imagePoint[1]);
  const uint32_t v1 = std::ceil(imagePoint[1]);
  const double r = imagePoint[1] - v0;

  // azimuth angle with interpolated offset
  const double azimuth0 = 2.0 * M_PI * imagePoint[0] / double(imageWidth_);
  const double azimuth = ((1.0-r) * beamAzimuthAngles_[v0] +  r * beamAzimuthAngles_[v1])
      * 2.0 * M_PI / 360.0 + azimuth0;

  // interpolate elevation angle
  double elevation = ((1.0-r) * beamElevationAngles_[v0] + r * beamElevationAngles_[v1])
      * 2.0 * M_PI / 360.0;

  // project as ray
  (*direction)[0] = std::cos(elevation) * std::cos(azimuth);
  (*direction)[1] = -std::cos(elevation) * std::sin(azimuth);
  (*direction)[2] = std::sin(elevation);

  // Jacobian w.r.t. imagePoint
  pointJacobian->setZero(); /// \todo

  return true;
}

// Back-project 2d image points into Euclidean space (direction vectors).
bool OusterLidar::backProjectBatch(
    const Eigen::Matrix2Xd & imagePoints, Eigen::Matrix3Xd * directions,
    std::vector<bool> * success) const
{
  const int numPoints = imagePoints.cols();
  directions->row(3) = Eigen::VectorXd::Ones(numPoints);
  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector2d imagePoint = imagePoints.col(i);
    Eigen::Vector3d point;
    bool suc = backProject(imagePoint, &point);
    if(success)
      success->push_back(suc);
    directions->col(i) = point;
  }
  return true;
}

// Back-project a 2d image point into homogeneous point (direction vector).
bool OusterLidar::backProjectHomogeneous(
    const Eigen::Vector2d & imagePoint, Eigen::Vector4d * direction) const
{
  Eigen::Vector3d ray;
  bool success = backProject(imagePoint, &ray);
  direction->template head<3>() = ray;
  (*direction)[3] = 1.0;  // arbitrary
  return success;
}

// Back-project a 2d image point into homogeneous point (direction vector).
bool OusterLidar::backProjectHomogeneous(
    const Eigen::Vector2d & imagePoint, Eigen::Vector4d * direction,
    Eigen::Matrix<double, 4, 2> * pointJacobian) const
{
  Eigen::Vector3d ray;
  Eigen::Matrix<double, 3, 2> pointJacobian3;
  bool success = backProject(imagePoint, &ray, &pointJacobian3);
  direction->template head<3>() = ray;
  (*direction)[4] = 1.0;  // arbitrary
  pointJacobian->template bottomRightCorner<1,2>() = Eigen::Vector2d::Zero();
  pointJacobian->template topLeftCorner<3, 2>() = pointJacobian3;
  return success;
}

// Back-project 2d image points into homogeneous points (direction vectors).
bool OusterLidar::backProjectHomogeneousBatch(
    const Eigen::Matrix2Xd & imagePoints, Eigen::Matrix4Xd * directions,
    std::vector<bool> * success) const
{
  const int numPoints = imagePoints.cols();
  directions->row(3) = Eigen::VectorXd::Ones(numPoints);
  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector2d imagePoint = imagePoints.col(i);
    Eigen::Vector3d point;
    bool suc = backProject(imagePoint, &point);
    if(success)
      success->push_back(suc);
    directions->template block<3, 1>(0, i) = point;
  }
  return true;
}

Eigen::VectorXd OusterLidar::beamAzimuthAngles() const
{
  return beamAzimuthAngles_;
}

void OusterLidar::setBeamAzimuthAngles(const Eigen::VectorXd &beamAzimuthAngles)
{
  beamAzimuthAngles_ = beamAzimuthAngles;
}

Eigen::VectorXd OusterLidar::beamElevationAngles() const
{
  return beamElevationAngles_;
}

void OusterLidar::setBeamElevationAngles(const Eigen::VectorXd &beamElevationAngles)
{
  beamElevationAngles_ = beamElevationAngles;
}

// needed for test instance:
const Eigen::VectorXd beam_altitude_angles = (Eigen::VectorXd(64) <<
    16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
    12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
    8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
    3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
    -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
    -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
    -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
    -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611).finished();

const Eigen::VectorXd beam_azimuth_angles = (Eigen::VectorXd(64) <<
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164).finished();

// get a test instance
std::shared_ptr<ProjectionBase> OusterLidar::createTestObject()
{
  return std::shared_ptr<ProjectionBase>(new OusterLidar(2048, 64, beam_azimuth_angles,
                                                     beam_altitude_angles));
}
// \brief get a test instance
OusterLidar OusterLidar::testObject()
{
  return OusterLidar(2048, 64, beam_azimuth_angles, beam_altitude_angles);
}

// \brief Get the intrinsics as a concatenated vector.
// \param[out] intrinsics The intrinsics as a concatenated vector.
void OusterLidar::getIntrinsics(Eigen::VectorXd &) const {
  throw std::runtime_error("not implemented");
}

// \brief overwrite all intrinsics - use with caution !
// \param[in] intrinsics The intrinsics as a concatenated vector.
bool OusterLidar::setIntrinsics(const Eigen::VectorXd &) {
  throw std::runtime_error("not implemented");
}

}  // namespace projection
}  // namespace srl
