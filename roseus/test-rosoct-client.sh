#!/usr/bin/octave

rosoct();
rosoct_add_msgs('rosoct');
rosoct_add_msgs('roslib');
rosoct_add_msgs('std_msgs');
rosoct_add_msgs('robot_msgs');

rosoct_subscribe('clientserver', @robot_msgs_PositionMeasurement, 
    @(msg)display(sprintf('name=%s id=%s pos=%f %f %f, rel=%f cov=%f %f', msg.name, msg.object_id, msg.pos.x, msg.pos.y, msg.pos.z, msg.reliability, msg.covariance(1),msg.covariance(2))), 1);

keyboard

	


