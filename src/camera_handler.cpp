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

#include "camera_handler.h"

#include <PS1080.h>

#include <set>

#include <boost/make_shared.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sstream>

typedef union
{
  struct
  {
    unsigned char b;
    unsigned char g;
    unsigned char r;
    unsigned char a;
  };
  float float_value;
  long long_value;
} RGBValue;

namespace ros_openni2_multicam
{

  boost::mutex g_openURIsMutex;
  std::set<std::string> g_openURIs;

  CameraHandler::CameraHandler(ros::NodeHandle* handle, const std::string& name, const std::string& id, int dataSkip, int width, int height)
    : m_id(id)
    , m_name(name)
    , m_nh(handle)
    , m_it(*handle)
    , m_focalLength(525.0*(double)width / 640.0)
    , m_droppedFrames(0)
    , m_dataSkip(dataSkip)
    , m_takeRGBImage(false)
    , m_width(width)
    , m_height(height)
  {
    m_pub_color = m_it.advertiseCamera(name + "/rgb/image_color", 1);
    m_pub_depth = m_it.advertise(name + "/depth_registered", 1);
    m_pub_rect  = m_it.advertise(name + "/rgb/image_rect", 1);
    m_pub_cloud = m_nh->advertise<PointCloud>(name + "/depth_registered/points", 1);

    // Start grabber thread
    m_shouldExit = false;
    m_thread = boost::thread(boost::bind(&CameraHandler::run, this));
	
    // Ask for the mask filename and create the mask object
    std::stringstream mask_filename;
    std::string mask_folder("/tmp");
    m_nh->getParam("mask", mask_folder);
    mask_filename << mask_folder << name << ".png";
    m_mask = boost::make_shared<ImageMask>(mask_filename.str());
  }

  CameraHandler::~CameraHandler()
  {
    m_shouldExit = true;
    m_thread.join();
  }

  bool CameraHandler::open(const boost::shared_ptr<openni::Device>& device)
  {
    // Setup depth stream
    openni::VideoMode videoMode;
    videoMode.setFps(30);
    videoMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    videoMode.setResolution(m_width, m_height);

    if(m_depthStream.create(*device, openni::SENSOR_DEPTH) != openni::STATUS_OK)
    {
      fprintf(stderr, "Could not create depth stream\n");
      return false;
    }
    if(m_depthStream.setVideoMode(videoMode) != openni::STATUS_OK)
    {
      fprintf(stderr, "Could not set video mode\n");
      return false;
    }

    m_depthStream.setMirroringEnabled(false);

    if(m_depthStream.start() != openni::STATUS_OK)
    {
      fprintf(stderr, "Could not start depth stream\n");
      return false;
    }

    // Setup color stream
    videoMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);

    if(m_colorStream.create(*device, openni::SENSOR_COLOR) != openni::STATUS_OK)
    {
      fprintf(stderr, "Could not create color stream\n");
      return false;
    }
    if(m_colorStream.setVideoMode(videoMode) != openni::STATUS_OK)
    {
      fprintf(stderr, "Could not set video mode\n");
      return false;
    }
    m_colorStream.setMirroringEnabled(false);
    if(m_colorStream.start() != openni::STATUS_OK)
    {
      fprintf(stderr, "Could not start color stream\n");
      return false;
    }

    // Enable registration
    if(!device->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
      fprintf(stderr, "Device does not provide hardware registration\n");
      return false;
    }

    openni::ImageRegistrationMode mode = device->getImageRegistrationMode();
    if(mode != openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)
    {
      if(device->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) != openni::STATUS_OK)
      {
        fprintf(stderr, "Could not enable image registration: %s\n", openni::OpenNI::getExtendedError());
        return false;
      }
    }

    // Enable sync
    device->setDepthColorSyncEnabled(true);

    {
      boost::mutex::scoped_lock lock(g_openURIsMutex);
      g_openURIs.insert(device->getDeviceInfo().getUri());
      m_device = device;
    }

    return true;
  }

  bool CameraHandler::reset()
  {
    // Tell the device to do a hard reset. This will cause USB re-enumeration!
    uint64_t type = XN_RESET_TYPE_POWER;
    if(m_device->setProperty(XN_MODULE_PROPERTY_RESET, &type, sizeof(type)) != openni::STATUS_OK)
    {
      fprintf(stderr, "Could not reset device %s: %s\n", m_id.c_str(), openni::OpenNI::getExtendedError());
      return false;
    }

    return true;
  }

  void CameraHandler::step()
  {
    if(!m_device)
    {
      usleep(200 * 1000);
      return;
    }

    openni::VideoStream* streams[2] = {&m_depthStream, &m_colorStream};

    int index;
    openni::Status status = openni::OpenNI::waitForAnyStream(streams, 2, &index, 10000);
    /*
      if (m_takeRGBImage){
      openni::Status status = openni::OpenNI::waitForAnyStream(&(&m_colorStream), 1, &index, 10000);
      } else {
      openni::Status status = openni::OpenNI::waitForAnyStream(&(&m_depthStream), 1, &index, 10000);
      }
    */

    if(status == openni::STATUS_TIME_OUT)
    {
      boost::mutex::scoped_lock lock(g_openURIsMutex);

      fprintf(stderr, "Timeout on camera ID %s, trying to reset camera...\n", m_id.c_str());

      std::string uri = m_device->getDeviceInfo().getUri();

      m_colorStream.destroy();
      m_depthStream.destroy();

      reset();

      m_device->close();

      m_device.reset();

      g_openURIs.erase(uri);

      return;
    }

    if(status != openni::STATUS_OK)
    {
      fprintf(stderr, "Could not wait\n");
      return;
    }

    ros::Time now = ros::Time::now();
    streams[index]->readFrame(&m_frame);

    switch(index)
    {
      case 0:
      {
        // Frame dropping to limit CPU usage
        if(m_droppedFrames < m_dataSkip)
        {
          m_droppedFrames++;
          return;
        }

        m_droppedFrames = 0;

        // Take a RGB frame after this one
        m_takeRGBImage = true;

        if(m_frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM)
        {
          fprintf(stderr, "Unknown pixel format %d", m_frame.getVideoMode().getPixelFormat());
          return;
        }

        m_depthImage = boost::make_shared<sensor_msgs::Image>();

        m_depthImage->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        m_depthImage->width = m_frame.getWidth();
        m_depthImage->height = m_frame.getHeight();
        m_depthImage->step = m_frame.getWidth() * sizeof(uint16_t);

        m_depthImage->data.resize(m_frame.getDataSize());
        memcpy(m_depthImage->data.data(), m_frame.getData(), m_depthImage->data.size());

        cv::Mat_<uint16_t> cvImage(m_depthImage->height, m_depthImage->width, reinterpret_cast<uint16_t*>(m_depthImage->data.data()), m_depthImage->step);
        m_mask->maskImage(cvImage);

        // depth_registered is published in rgb frame
        m_depthImage->header.frame_id = m_name + "_rgb_optical_frame";
        m_depthImage->header.stamp = now;
        m_pub_depth.publish(m_depthImage);

        break;
      }
      case 1:
      {
        // Capture one RGB image after we took a depth image
        if(!m_takeRGBImage)
          return;
        m_takeRGBImage = false;

        if(m_frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_RGB888)
        {
          fprintf(stderr, "Unknown pixel format %d", m_frame.getVideoMode().getPixelFormat());
          return;
        }

        m_colorImage.reset(new sensor_msgs::Image);
        m_colorImage->encoding = sensor_msgs::image_encodings::RGB8;
        m_colorImage->width = m_frame.getWidth();
        m_colorImage->height = m_frame.getHeight();
        m_colorImage->step = m_frame.getStrideInBytes();

        m_colorImage->data.resize(m_frame.getDataSize());
        memcpy(m_colorImage->data.data(), m_frame.getData(), m_colorImage->data.size());

        m_colorImage->header.frame_id = m_name + "_rgb_optical_frame";
        m_colorImage->header.stamp = now; // FIXME

        sensor_msgs::CameraInfoPtr info = getDefaultCameraInfo(m_colorImage->width, m_colorImage->height, m_focalLength);
        info->header.frame_id = m_name + "_rgb_optical_frame";
        info->header.stamp = now;

        m_pub_color.publish(m_colorImage, info);

        if (m_pub_rect.getNumSubscribers() != 0)
        {
          image_geometry::PinholeCameraModel model;
          model.fromCameraInfo(info);

          const cv::Mat image = cv_bridge::toCvShare(m_colorImage)->image;

          cv::Mat rect;
          model.rectifyImage(image, rect); // TODO: insert variable interpolation method? 

          sensor_msgs::ImagePtr rect_msg = cv_bridge::CvImage(m_colorImage->header, m_colorImage->encoding, rect).toImageMsg();
          m_pub_rect.publish(rect_msg);
        }

        publishPointCloud();

        break;
      }
    }
  }

  void CameraHandler::run()
  {
    while(!m_shouldExit)
      step();

    if(m_device)
    {
      m_depthStream.stop();
      m_depthStream.destroy();

      m_colorStream.stop();
      m_colorStream.destroy();

      std::string uri = m_device->getDeviceInfo().getUri();

      if(m_device && m_device->isValid())
        m_device->close();

      {
        boost::mutex::scoped_lock lock(g_openURIsMutex);
        g_openURIs.erase(uri);
      }
    }
  }

  void CameraHandler::shutdown()
  {
    m_shouldExit = true;
  }

  sensor_msgs::CameraInfoPtr CameraHandler::getDefaultCameraInfo(int width, int height, double f) const
  {
    sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

    info->width = width;
    info->height = height;

    // No distortion
    info->D.resize(5, 0.0);
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // Simple camera matrix: square pixels (fx = fy), principal point at center
    info->K.assign(0.0);
    info->K[0] = info->K[4] = f;
    info->K[2] = (width / 2) - 0.5;
    // Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
    // This formula keeps the principal point the same in VGA and SXGA modes
    info->K[5] = (width * (3./8.)) - 0.5;
    info->K[8] = 1.0;

    // No separate rectified image plane, so R = I
    info->R.assign(0.0);
    info->R[0] = info->R[4] = info->R[8] = 1.0;

    // Then P=K(I|0) = (K|0)
    info->P.assign(0.0);
    info->P[0] = info->P[5] = f; // fx, fy
    info->P[2] = info->K[2]; // cx
    info->P[6] = info->K[5]; // cy
    info->P[10] = 1.0;

    return info;
  }

  bool CameraHandler::isAlreadyOpen(const std::string& uri)
  {
    boost::mutex::scoped_lock lock(g_openURIsMutex);
    return g_openURIs.find(uri) != g_openURIs.end();
  }

  void CameraHandler::publishPointCloud()
  {
    ros::Time now = ros::Time::now();

    if(!m_colorImage || !m_depthImage)
      return;

    if(m_colorImage->width != m_depthImage->width || m_colorImage->height != m_depthImage->height)
    {
      ROS_ERROR("Color and depth images do not have the same size! Cannot publish point cloud.");
      return;
    }
	
    PointCloud::Ptr cloud(new PointCloud);
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
    cloud->header = pcl_conversions::toPCL(m_depthImage->header);
#else
    cloud->header = m_depthImage->header;
#endif
    cloud->width = m_colorImage->width;
    cloud->height = m_colorImage->height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    float constant = 1.0 / 1000.0 / m_focalLength;
    float mid_x = m_colorImage->width / 2.0 - 0.5;
    float mid_y = m_colorImage->height / 2.0 - 0.5;
	
    if(!m_colorImage)
      return;

    const int color_step = 3;
    uint16_t* depth_ptr = (uint16_t*)m_depthImage->data.data();
    int row_step = m_depthImage->step / sizeof(uint16_t);
    const uint8_t* rgb = &m_colorImage->data[0];
    int rgb_skip = m_colorImage->step - m_colorImage->width * color_step;
    PointCloud::iterator pt_iter = cloud->begin ();

    for (unsigned int v = 0; v < m_colorImage->height; ++v, depth_ptr += row_step, rgb += rgb_skip)
    {
      for (unsigned int u = 0; u < m_colorImage->width; ++u, rgb += color_step)
      {
        PointType& point = *pt_iter++;
        uint16_t depth = depth_ptr[u];

        if(depth == 0)
          point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        else
        {
          point.x = (((float)u) - mid_x) * static_cast<float>(depth) * constant;
          point.y = (((float)v) - mid_y) * static_cast<float>(depth) * constant;
          point.z = static_cast<float>(depth) / 1000.0f;
        }

        // Fill in color
        RGBValue color;
        color.r = rgb[0];
        color.g = rgb[1];
        color.b = rgb[2];
        color.a = 0;
        point.rgb = color.float_value;
      }
    }

    m_pub_cloud.publish(cloud);
  }

}
