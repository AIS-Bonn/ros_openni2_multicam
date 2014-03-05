// masks out all parts of the image which are given by a specific file
// Author: Daniel Reuter <daniel.robin.reuter@gmail.com>
// Contact (package maintainer): Dirk Holz <holz@ais.uni-bonn.de>

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
