// Custom OpenNI2 driver for the sensor head
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
// Contact (package maintainer): Dirk Holz <holz@ais.uni-bonn.de>

#ifndef OPENNI_DRIVER_H
#define OPENNI_DRIVER_H

#include <nodelet/nodelet.h>

#include "camera_handler.h"

namespace ros_openni2_multicam
{

  class OpenNIDriver : public nodelet::Nodelet
  {
    public:
      OpenNIDriver();
      virtual ~OpenNIDriver();

      virtual void onInit();
    private:
      void detectDevices();

      typedef std::map<std::string, boost::shared_ptr<CameraHandler> > HandlerMap;
      HandlerMap m_handlers;
      ros::Timer m_timer;
  };

}

#endif
