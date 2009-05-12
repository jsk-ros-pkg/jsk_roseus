#!/usr/bin/octave

rosoct();
rosoct_add_msgs('rosoct');
rosoct_add_msgs('roslib');
rosoct_add_msgs('std_msgs');
rosoct_add_msgs('robot_msgs');

suc = rosoct_advertise('clientserver',@robot_msgs_PositionMeasurement, 1);
for i = 1:1000
    msg = robot_msgs_PositionMeasurement();
    msg.name = sprintf('name %d', i);
    msg.object_id = sprintf('objectid %d', i);
    msg.pos.x = (i * 0.1);
    msg.pos.y = (i * 0.2);
    msg.pos.z = (i * 0.3);        
    msg.reliability = (i * 0.4);
    msg.covariance = [i * 1.0, i * 1.1, i * 1.2, i * 1.3, i * 1.4, i * 1.5, i * 1.6, i * 1.7, i * 1.8, i * 1.9];
    suc = rosoct_publish('clientserver',msg);
    if( ~suc )
        error('failed to publish chatter');
    end
    sleep(0.1);
end
