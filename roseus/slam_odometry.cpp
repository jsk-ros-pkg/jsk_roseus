#include "ros/node.h"
#include "laser_scan/LaserScan.h"
#include "robot_srvs/StaticMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"

#include <iostream>

#include <time.h>

#include "ros/console.h"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

using namespace std;

class SlamOdometry
{
public:
  ros::Node* node_;
  tf::TransformBroadcaster tf_;
  laser_scan::LaserScan scan_;
  GMapping::OrientedPoint odom_;
  GMapping::ScanMatcher m_matcher;

  double *ranges_double_now, *ranges_double_prev;
  
  double delta_;
  double maxUrange_;
  double maxrange_;
  double sigma_;
  int kernelSize_;
  double lstep_;
  double astep_;
  int iterations_;
  double lsigma_;
  double ogain_;
  int lskip_;

  SlamOdometry(ros::Node& node)
  {
    node_=&node;

    node_->param("~/delta", delta_, 0.1);
    node_->param("~/maxUrange", maxUrange_, 80.0);
    node_->param("~/sigma", sigma_, 0.5);
    node_->param("~/kernelSize", kernelSize_, 1);
    node_->param("~/lstep", lstep_, 0.5);
    node_->param("~/astep", astep_, 0.2);
    node_->param("~/iterations", iterations_, 5);
    node_->param("~/lsigma", lsigma_, 0.075);
    node_->param("~/ogain", ogain_, 3.0);
    node_->param("~/lskip", lskip_, 0);

    std::cerr << "delta = " << delta_ << ", maxUrange = " << maxUrange_
              << ", sigma = " << sigma_ << ", kernelSize = " << kernelSize_
              << ", lstep = " << lstep_ << ", astep = " << astep_
              << ", iterations = " << iterations_ << ", ogain = " << ogain_
              << ", lskip = " << lskip_ << std::endl;

    node_->subscribe("base_scan", scan_, &SlamOdometry::laser_cb, this, 10);
  }

  void laser_init() {
    ranges_double_now = new double[scan_.ranges.size()];
    ranges_double_prev = new double[scan_.ranges.size()];
      
    double* laser_angles = new double[scan_.ranges.size()];

    double theta = scan_.angle_min;
    for(unsigned int i=0; i<scan_.ranges.size(); i++) {
      laser_angles[i]=theta;
      theta += scan_.angle_increment;
    }
    double maxrange = scan_.range_max;
    m_matcher.setLaserParameters(scan_.ranges.size(), laser_angles, 
                                 GMapping::OrientedPoint(0,0,0));
    m_matcher.setMatchingParameters
      ((double)maxUrange_, maxrange, (double)sigma_, 
       (int)kernelSize_, (double)lstep_, (double)astep_, (int)iterations_, 
       (double)lsigma_, (unsigned int)lskip_);
  }

  void laser_cb() {
    static bool is_init_laser = false;
    if  (! is_init_laser ) {
      is_init_laser = true;
      laser_init();
    }

    for(unsigned int i=0; i < scan_.ranges.size(); i++)
      {
        // Must filter out short readings, because the mapper won't
        if(scan_.ranges[i] < scan_.range_min)
          ranges_double_now[i] = (double)scan_.range_max;
        else
          ranges_double_now[i] = (double)scan_.ranges[i];
      }

    GMapping::OrientedPoint pnew, init;
    
    GMapping::Point center;
    GMapping::Point wmin(-0.0, -100.0);
    GMapping::Point wmax(100.0, 100.0);
    center.x=(wmin.x + wmax.x) / 2.0;
    center.y=(wmin.y + wmax.y) / 2.0;
    GMapping::ScanMatcherMap smap(center, wmin.x, wmin.y, wmax.x, wmax.y, delta_);
    m_matcher.registerScan(smap, init, ranges_double_prev);
    m_matcher.computeActiveArea(smap, init, ranges_double_prev);
    double ret = m_matcher.optimize(pnew, smap, init, ranges_double_now);

    odom_.x += pnew.x;
    odom_.y += pnew.y;
    odom_.theta += pnew.theta;

    for(unsigned int i=0; i < scan_.ranges.size(); i++)
      ranges_double_prev[i] = ranges_double_now[i];

    cerr << odom_.x << ", " << odom_.y << ", " << odom_.theta << "> " << ret << endl;
    
    tf_.sendTransform(tf::Transform(tf::Quaternion(odom_.theta,
						   0.0, 
						   0.0), 
				    tf::Point(odom_.x, 
					      odom_.y, 
					      0.0) 
				    ),
                      scan_.header.stamp,
		      "base_link",
		      "odom");
    tf_.sendTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0), 
				    tf::Point(0.0, 0.0, 0.0)),
                      scan_.header.stamp, "base_laser", "base_link");
    
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node node("slam_odometry");
  SlamOdometry slam_odometry(node);
  node.spin();
  return 0;
}

