// Handles one camera
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
// Contact (package maintainer): Dirk Holz <holz@ais.uni-bonn.de>

#ifndef CAMERA_HANDLER_H
#define CAMERA_HANDLER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <string>

#include <OpenNI.h>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>

#include "image_mask.h"

namespace ros_openni2_multicam
{

  class CameraHandler
  {
    public:
      CameraHandler(ros::NodeHandle* handle, const std::string& name, const std::string& id, int dataSkip, int width, int height);
      ~CameraHandler();
      bool open(const boost::shared_ptr<openni::Device>& device);

      void shutdown();

      static bool isAlreadyOpen(const std::string& uri);
    private:
      sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f) const;
      bool reset();

      void run();
      void step();
      void reopenDevice();
      void publishPointCloud();

      boost::shared_ptr<openni::Device> m_device;
      std::string m_id;
      std::string m_name;
      openni::VideoStream m_depthStream;
      openni::VideoStream m_colorStream;
      openni::VideoFrameRef m_frame;
      boost::thread m_thread;
      bool m_shouldExit;
      ros::NodeHandle* m_nh;

      image_transport::ImageTransport m_it;
      image_transport::CameraPublisher m_pub_color;
      image_transport::Publisher m_pub_depth;
      ros::Publisher m_pub_cloud;

      boost::shared_ptr<ImageMask> m_mask;

      sensor_msgs::ImagePtr m_colorImage;
      sensor_msgs::ImagePtr m_depthImage;

      double m_focalLength;
      int m_droppedFrames;
      int m_dataSkip;
      bool m_takeRGBImage;

      int m_width, m_height;
  };

}

#endif
