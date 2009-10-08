#include <ros/node.h>
#include "tf/transform_broadcaster.h"

#include <roseus/Odometry.h>
#include <iostream>

class HRP2Odom
{
public:
  tf::TransformBroadcaster tf_;
  roseus::Odometry odom_;
  
  HRP2Odom(ros::Node& node) /*: tf_(node) */
  {
    //node.advertise<deprecated_msgs::RobotBase2DOdom>("odom");
    node.subscribe("odometry", odom_, &HRP2Odom::odomEusReceived, this, 10);
  }

  void odomEusReceived() {
    // Translate from Euslisp data to ROS data
    //ros::Time stamp = ros::Time::now();
    
    //std::cerr << stamp.toNSec() << " --> " ;
    //stamp = stamp + ros::Duration(0.0);
    //std::cerr << stamp.toNSec() << std::endl;


    //odom_.header.frame_id = "odom";
    //odom_.header.stamp.sec = (long long unsigned int)floor(hdr->timestamp); 
    //odom_.header.stamp.nsec = (long long unsigned int)((hdr->timestamp - floor(hdr->timestamp)) * 1000000000ULL); 
      

    // Send transform for odom
    tf_.sendTransform(tf::Transform(tf::Quaternion(odom_.xyr[2],
						   0.0, 
						   0.0), 
				    tf::Point(odom_.xyr[0], 
					      odom_.xyr[1], 
					      0.0) 
				    ),
		      odom_.header.stamp,
                      //ros::Time::now(),
                      //stamp, 	
		      "base_link",
		      "odom");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node node("hrp2_odom");
  HRP2Odom hrp_odom(node);
  node.spin();
  return 0;
}

