/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */
/* 
 * Software License Agreement (BSD License)
 * 
 * Copyright (c) 2014, Autonomous Intelligent Systems Group, Rheinische
 * Friedrich-Wilhelms-Universität Bonn
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Rheinische Friedrich-Wilhelms-Universität Bonn
 *       nor the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define DEBUG_MASK 0

#include "image_mask.h"

#include <set>

#include <boost/make_shared.hpp>
#include <boost/iterator/iterator_concepts.hpp>

#include <opencv2/highgui/highgui.hpp> // For CV_LOAD_IMAGE_GRAYSCALE

#include <ros/console.h>

namespace ros_openni2_multicam
{


  ImageMask::ImageMask(std::string filename) : m_useMask(true)
  {
    cv::Mat mask_image(cv::imread(filename, (CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH)));
    if (!mask_image.data){
      ROS_WARN_STREAM("Could not find image file '" << filename << "'. Mask will be disabled");
      m_useMask = false;
      return;
    }
    m_mask.create(mask_image.rows, mask_image.cols);
    for (int y = 0; y < mask_image.rows; ++y){
      for (int x = 0; x < mask_image.cols; ++x){
        if (mask_image.at<uint8_t>(y, x) > UCHAR_MAX/2){
          m_mask(y, x) = 0xFFFF;
        }else{
          m_mask(y, x) = 0x0000;
        }
      }
    }
#if DEBUG_MASK
    cv::namedWindow( "Original", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "Mask", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "Masked", CV_WINDOW_AUTOSIZE );
#endif
  }

  ImageMask::~ImageMask()
  {
  }

  void ImageMask::maskImage(cv::Mat& image)
  {
    if (m_useMask) {
      if (image.cols == m_mask.cols && image.rows == m_mask.rows){
#if DEBUG_MASK
        cv::imshow("Original", image);
#endif
        cv::bitwise_and(image, m_mask, image);
#if DEBUG_MASK
        cv::imshow("Mask", m_mask);
        cv::imshow("Masked", image);
        cv::waitKey(10);
#endif
      } else {
        ROS_ERROR("Image and Mask do not have the same size!");
      }
    } else {
      ROS_DEBUG_ONCE("I do not mask.");
    }
  }


}
