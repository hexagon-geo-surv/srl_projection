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
 *  Created on: Feb 3, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/ProjectionBase.hpp
 * @brief Header implementation file for the ProjectionBase class.
 * @author Stefan Leutenegger
 */

#include <iostream>

/// \brief Main namespace of this package.
namespace srl {
/// \brief Namespace for camera-related functionality.
namespace projection {

// Set the mask. It must be the same size as the image and
bool ProjectionBase::setMask(const cv::Mat & mask)
{
  // check type
  if (mask.type() != CV_8UC1) {
    return false;
  }
  // check size
  if (mask.rows != imageHeight_) {
    return false;
  }
  if (mask.cols != imageWidth_) {
    return false;
  }
  mask_ = mask;
  return true;
}

/// Was a nonzero mask set?
bool ProjectionBase::removeMask()
{
  mask_.resize(0);
  return true;
}

// Was a nonzero mask set?
bool ProjectionBase::hasMask() const
{
  return (mask_.data);
}

// Get the mask.
const cv::Mat & ProjectionBase::mask() const
{
  return mask_;
}

bool ProjectionBase::isMasked(const Vector2f& imagePoint) const
{
  if (!isInImage(imagePoint)) {
    return true;
  }
  if (!hasMask()) {
    return false;
  }
  return mask_.at<uchar>(int(imagePoint[1]), int(imagePoint[0]));
}

// Check if the keypoint is in the image.
bool ProjectionBase::isInImage(const Vector2f& imagePoint) const
{
  if (imagePoint[0] < 0.0 || imagePoint[1] < 0.0) {
    return false;
  }
  if (imagePoint[0] >= imageWidth_ || imagePoint[1] >= imageHeight_) {
    return false;
  }
  return true;
}

} // namespace projection
} // namespace srl

