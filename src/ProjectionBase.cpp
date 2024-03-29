/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
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
 *  Created on: Feb 2, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file ProjectionBase.cpp
 * @brief Source file for the CameraBase class.
 * @author Stefan Leutenegger
 */

#include <srl/projection/ProjectionBase.hpp>

/// \brief Main namespace of this package.
namespace srl {
/// \brief Namespace for camera-related functionality.
namespace projection {

// Creates a random (uniform distribution) image point.
srl::Vector2f ProjectionBase::createRandomImagePoint() const
{
  // Uniform random sample in image coordinates.
  // Add safety boundary for later inaccurate backprojection
  srl::Vector2f outPoint = srl::Vector2f::Random();
  outPoint += srl::Vector2f::Ones();
  outPoint *= 0.5;
  outPoint[0] *= srl::float_t(imageWidth_-1);
  outPoint[1] *= srl::float_t(imageHeight_-1);
  return outPoint;
}

// Creates a random visible point in Euclidean coordinates.
srl::Vector3f ProjectionBase::createRandomVisiblePoint(srl::float_t minDist,
                                                     srl::float_t maxDist) const
{
  // random image point first:
  srl::Vector2f imagePoint = createRandomImagePoint();
  // now sample random depth:
  srl::Vector2f depth = srl::Vector2f::Random();
  srl::Vector3f ray;
  backProject(imagePoint, &ray);
  ray.normalize();
  ray *= (0.5 * (maxDist - minDist) * (depth[0] + 1.0) + minDist);  // rescale and offset
  return ray;
}

// Creates a random visible point in homogeneous coordinates.
srl::Vector4f ProjectionBase::createRandomVisibleHomogeneousPoint(
    srl::float_t minDist, srl::float_t maxDist) const
{
  srl::Vector3f point = createRandomVisiblePoint(minDist, maxDist);
  return srl::Vector4f(point[0], point[1], point[2], 1.0);
}

}  // namespace projection
}  // namespace srl
