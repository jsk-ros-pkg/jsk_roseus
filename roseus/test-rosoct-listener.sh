#!/usr/bin/octave

rosoct();
rosoct_add_msgs('rosoct');

rosoct_subscribe('chatter', @rosoct_String,
    @(msg)display(sprintf('stringcb:%s', msg.data)), 1)

keyboard

	


