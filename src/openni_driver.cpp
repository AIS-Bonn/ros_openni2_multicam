// Custom OpenNI2 driver for the sensor head
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
// Contact (package maintainer): Dirk Holz <holz@ais.uni-bonn.de>

#include "openni_driver.h"

#include <PS1080.h>

#include <boost/make_shared.hpp>

#include <pluginlib/class_list_macros.h>

#include <ros/node_handle.h>

#include <XmlRpcValue.h>

namespace ros_openni2_multicam
{

  const std::string invalid_serial = "-1";
  const std::string default_topic = "/camera";

  OpenNIDriver::OpenNIDriver()
  {
  }

  OpenNIDriver::~OpenNIDriver()
  {
    for(HandlerMap::iterator it = m_handlers.begin(); it != m_handlers.end(); ++it)
      it->second->shutdown();

    m_handlers.clear();

    printf("OpenNI shutdown...\n");
    openni::OpenNI::shutdown();
  }

  void OpenNIDriver::detectDevices()
  {
    openni::Array<openni::DeviceInfo> devices;
    openni::OpenNI::enumerateDevices(&devices);

    bool stop_searching = false;
    for(int i = 0; i < devices.getSize(); ++i)
    {
      if (stop_searching)
        break;
      const openni::DeviceInfo& info = devices[i];

      if(CameraHandler::isAlreadyOpen(info.getUri()))
        continue;

      NODELET_INFO("Detected new device: %s by %s", info.getName(), info.getVendor());

      boost::shared_ptr<openni::Device> dev = boost::make_shared<openni::Device>();
      if(dev->open(info.getUri()) != openni::STATUS_OK)
      {
        NODELET_ERROR("Could not open Device: %s", openni::OpenNI::getExtendedError());
        return;
      }

      char buf[1024];
      int bufSize = sizeof(buf);

      if(dev->getProperty(openni::DEVICE_PROPERTY_SERIAL_NUMBER, buf, &bufSize) != openni::STATUS_OK)
      {
        NODELET_ERROR("Could not get serial number: %s", openni::OpenNI::getExtendedError());
        return;
      }

      printf("  with serial number '%s'\n", buf);

      HandlerMap::iterator it = m_handlers.find(buf);
      if(it == m_handlers.end())
      {
        if (!m_handlers.empty() && (m_handlers.begin()->first.compare(invalid_serial) == 0))
        {
          it = m_handlers.begin();
          stop_searching = true;
        }
        else
        {
          NODELET_INFO("Unknown, ignoring");
          continue;
        }
      }

      const boost::shared_ptr<CameraHandler>& handler = (*it).second;

      if(!handler->open(dev))
      {
        NODELET_ERROR("Could not initialize camera %s", buf);

        // Tell the device to do a hard reset. This will cause USB re-enumeration!
        uint64_t type = XN_RESET_TYPE_POWER;
        if(dev->setProperty(XN_MODULE_PROPERTY_RESET, &type, sizeof(type)) != openni::STATUS_OK)
        {
          fprintf(stderr, "Could not reset device %s: %s\n", buf, openni::OpenNI::getExtendedError());
        }

        dev->close();
        return;
      }
    }
  }

  void OpenNIDriver::onInit()
  {
    ros::NodeHandle& nh = getPrivateNodeHandle();

    std::map<std::string, std::string> nameMap;

    int defaultDataSkip;
    nh.param("data_skip", defaultDataSkip, -1);

    int defaultWidth;
    nh.param("width", defaultWidth, 640);

    int defaultHeight;
    nh.param("height", defaultHeight, 480);

    XmlRpc::XmlRpcValue list;
    if (nh.getParam("cameras", list))
    {
      ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

      for(int32_t i = 0; i < list.size(); ++i)
      {
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        ROS_ASSERT(list[i].hasMember("serial_number"));
        ROS_ASSERT(list[i].hasMember("topic"));

        std::string serial = list[i]["serial_number"];
        std::string topic = list[i]["topic"];

        int dataSkip = defaultDataSkip;
        if(list[i].hasMember("data_skip"))
          dataSkip = list[i];

        int width = defaultWidth;
        if(list[i].hasMember("width"))
          width = list[i];

        int height = defaultHeight;
        if(list[i].hasMember("height"))
          height = list[i];

        m_handlers[serial] = boost::make_shared<CameraHandler>(&nh, topic, serial, dataSkip, width, height);

        nameMap[serial] = topic;
      }
    }
    else //if (list.size() == 0) // initialize a handler for any connected camera
      m_handlers[invalid_serial] = boost::make_shared<CameraHandler>(&nh, default_topic, invalid_serial, defaultDataSkip, defaultWidth, defaultHeight);

    if(openni::OpenNI::initialize() != openni::STATUS_OK)
    {
      NODELET_ERROR("Could not initialize OpenNI: %s", openni::OpenNI::getExtendedError());
      std::abort();
    }

    m_timer = nh.createTimer(ros::Duration(1.0), boost::bind(&OpenNIDriver::detectDevices, this));
    m_timer.start();
  }

}

PLUGINLIB_EXPORT_CLASS(ros_openni2_multicam::OpenNIDriver, nodelet::Nodelet)
