// Handles one camera
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
// Contact (package maintainer): Dirk Holz <holz@ais.uni-bonn.de>

#include "camera_handler.h"

#include <PS1080.h>

#include <set>

#include <boost/make_shared.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
#include <pcl_conversions/pcl_conversions.h>
#else
#include <pcl/ros/conversions.h>
#endif
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
    m_pub_cloud = m_nh->advertise<sensor_msgs::PointCloud2>(name + "/depth_registered/points", 1);

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

        for(unsigned int i = 0; i < m_depthImage->width*m_depthImage->height; ++i)
        {
          uint16_t* pixel = ((uint16_t*)(m_depthImage->data.data())) + i;
          if(*pixel < 600)
            *pixel = 0;
        }

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

        publishPointCloud();
        // publishPointCloudDirectly();

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
	
    sensor_msgs::PointCloud2::Ptr cloud = boost::make_shared<sensor_msgs::PointCloud2>();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
    cloud_pcl->header.stamp = now.toNSec() / 1e3;
#else
    cloud_pcl->header.stamp = now;
#endif

    cloud_pcl->header.frame_id = m_name + "_rgb_optical_frame";
  
    cloud_pcl->width = m_colorImage->width;
    cloud_pcl->height = m_colorImage->height;
    cloud_pcl->is_dense = false;
    cloud_pcl->points.resize(cloud_pcl->width * cloud_pcl->height);


    float constant = 1.0 / 1000.0 / m_focalLength;
    float bad = std::numeric_limits<float>::quiet_NaN();
    float mid_x = m_colorImage->width / 2.0 - 0.5;
    float mid_y = m_colorImage->height / 2.0 - 0.5;
	
    if(!m_colorImage)
      return;

    const int color_step = 3;
    uint16_t* depth_ptr = (uint16_t*)m_depthImage->data.data();
    int row_step = m_depthImage->step / sizeof(uint16_t);
    const uint8_t* rgb = &m_colorImage->data[0];
    int rgb_skip = m_colorImage->step - m_colorImage->width * color_step;
    pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud_pcl->begin ();

    for (unsigned int v = 0; v < m_colorImage->height; ++v, depth_ptr += row_step, rgb += rgb_skip)
    {
      for (unsigned int u = 0; u < m_colorImage->width; ++u, rgb += color_step)
      {
        pcl::PointXYZRGB& point = *pt_iter++;
        uint16_t depth = depth_ptr[u];

        if(depth == 0)
          point.x = point.y = point.z = bad;
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

#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
    pcl::PCLPointCloud2 cloud_pcl2;
    pcl::toPCLPointCloud2(*cloud_pcl, cloud_pcl2);
    pcl_conversions::fromPCL(cloud_pcl2, *cloud);
#else
    pcl::toROSMsg(*cloud_pcl, *cloud);
#endif
    m_pub_cloud.publish(cloud);
  }


  void CameraHandler::publishPointCloudDirectly()
  {
    ros::Time now = ros::Time::now();

    if(!m_colorImage || !m_depthImage)
      return;

    if(m_colorImage->width != m_depthImage->width || m_colorImage->height != m_depthImage->height)
    {
      ROS_ERROR("Color and depth images do not have the same size! Cannot publish point cloud.");
      return;
    }
	
    sensor_msgs::PointCloud2::Ptr cloud = boost::make_shared<sensor_msgs::PointCloud2>();

    cloud->header.stamp = now;
    cloud->header.frame_id = m_name + "_rgb_optical_frame";

    cloud->fields.resize(6);
    cloud->fields[0].name = "x";
    cloud->fields[1].name = "y";
    cloud->fields[2].name = "z";
    cloud->fields[3].name = "_";
    cloud->fields[4].name = "rgb";
    cloud->fields[5].name = "_";
    int channel_offset = 0;
    for (int i = 0; i < 6; ++i, channel_offset += 4)
    {
      cloud->fields[i].offset = channel_offset;
      cloud->fields[i].datatype = sensor_msgs::PointField::FLOAT32;
      cloud->fields[i].count = 1;
    }
    cloud->fields[3].datatype = sensor_msgs::PointField::UINT8;
    cloud->fields[3].count = 4;
    cloud->fields[5].datatype = sensor_msgs::PointField::UINT8;
    cloud->fields[5].count = 12;
    channel_offset += 8;

    cloud->width = m_colorImage->width;
    cloud->height = m_colorImage->height;
    cloud->point_step = channel_offset;
    cloud->is_bigendian = 0;
    cloud->is_dense = 1;
    cloud->row_step = cloud->point_step * cloud->width;
    cloud->data.resize(cloud->row_step * cloud->height);

    uint16_t* depth_ptr = (uint16_t*)m_depthImage->data.data();
    uint8_t* out_ptr = cloud->data.data();
    uint8_t* color = m_colorImage->data.data();

    float constant = 1.0 / 1000.0 / m_focalLength;
    float bad = std::numeric_limits<float>::quiet_NaN();
    float mid_x = m_colorImage->width / 2.0 - 0.5;
    float mid_y = m_colorImage->height / 2.0 - 0.5;
	
    RGBValue rgb;
    rgb.a = 255;

    if(!color)
      return;

    for(unsigned int y = 0; y < m_colorImage->height; ++y)
    {
      for(unsigned int x = 0; x < m_colorImage->width; ++x)
      {
        float px;
        float py;
        float pz;

        if(*depth_ptr == 0)
        {
          px = py = pz = bad;
        }
        else
        {
          float depth = (*depth_ptr);
          px = (((float)x) - mid_x) * depth * constant;
          py = (((float)y) - mid_y) * depth * constant;
          pz = depth / 1000.0;
        }

        *((float*)(out_ptr + cloud->fields[0].offset)) = px;
        *((float*)(out_ptr + cloud->fields[1].offset)) = py;
        *((float*)(out_ptr + cloud->fields[2].offset)) = pz;
			
        rgb.r = color[0];
        rgb.g = color[1];
        rgb.b = color[2];

        *((float*)(out_ptr + cloud->fields[4].offset)) = rgb.float_value;

        out_ptr += cloud->point_step;
        depth_ptr++;
        color += 3;
      }
    }

    m_pub_cloud.publish(cloud);
  }

}
