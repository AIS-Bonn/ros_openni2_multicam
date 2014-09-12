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

#ifndef CAMERA_HANDLER_H
#define CAMERA_HANDLER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <string>

#include <OpenNI.h>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
#include <pcl_conversions/pcl_conversions.h>
#endif
#include "image_mask.h"

namespace ros_openni2_multicam
{

  class CameraHandler
  {

    public:
      typedef pcl::PointXYZRGB PointType;
      typedef typename pcl::PointCloud<PointType> PointCloud;
      
      CameraHandler(ros::NodeHandle* handle, const std::string& name, const std::string& id, int dataSkip, int width, int height);
      ~CameraHandler();
      bool open(const boost::shared_ptr<openni::Device>& device);

      void shutdown();

      static bool isAlreadyOpen(const std::string& uri);

      inline void setFocalLength(double f) { m_focalLength = f; }; 

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
      image_transport::Publisher m_pub_rect;
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
