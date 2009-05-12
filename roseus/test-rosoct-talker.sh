#!/usr/bin/octave

rosoct();
rosoct_add_msgs('rosoct');

suc = rosoct_advertise('chatter',@rosoct_String, 1);
for i = 1:1000
    msg = rosoct_String();
    msg.data = sprintf('hello world %d', i);
    suc = rosoct_publish('chatter',msg);
    if( ~suc )
        error('failed to publish chatter');
    end
    disp(msg.data);
    sleep(0.1);
end
