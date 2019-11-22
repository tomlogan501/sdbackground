
/*
 * ROS
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>

/*
 * EXTERN
 */
#include "extern/def.h"
#include "extern/nrutil.h"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // ...
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sdbackground");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("in_image_topic", 1, imageCallback);
  image_transport::Publisher pub = it.advertise("out_image_topic", 1);

  ros::spin();
}
