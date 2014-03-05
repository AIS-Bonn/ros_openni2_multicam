// masks out all parts of the image which are given by a specific file
// Author: Daniel Reuter <daniel.robin.reuter@gmail.com>
// Contact (package maintainer): Dirk Holz <holz@ais.uni-bonn.de>

#ifndef IMAGE_MASK_H
#define IMAGE_MASK_H

#include <boost/shared_ptr.hpp>

#include <string>

#include <opencv2/core/core.hpp>

namespace ros_openni2_multicam
{

  class ImageMask
  {
    public:
      ImageMask(std::string filename);
      ~ImageMask();
      void maskImage(cv::Mat& image);

    private:
      cv::Mat_<uint16_t> m_mask;
      bool m_useMask;
  };

}

#endif
