#include <ros/ros.h>
#include <std_msgs/String.h>
#include <roseus_test_genmsg/String.h>
#include <roseus_test_genmsg/Empty.h>

bool empty(roseus_test_genmsg::EmptyRequest  &req,
           roseus_test_genmsg::EmptyResponse &res){
    return true;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "roseus_test_genmsg");
    ros::NodeHandle n;

    ros::Publisher pub1 = n.advertise<std_msgs::String>("talker1", 100);
    ros::Publisher pub2 = n.advertise<roseus_test_genmsg::String>("talker2", 100);

    roseus_test_genmsg::EmptyRequest srv;
    ros::ServiceServer service = n.advertiseService("empty", empty);

    ros::Rate rate(10);
    while (ros::ok()) {

        std_msgs::String msg1;
        roseus_test_genmsg::String msg2;

        msg1.data = "msg1";
        msg2.data = "msg2";

        pub1.publish(msg1);
        pub2.publish(msg2);


        rate.sleep();

        ros::spinOnce();
    }

    return 0;
}
