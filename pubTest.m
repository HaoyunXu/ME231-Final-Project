%initialize publisher
pub = rospublisher('/mpcinput','geometry_msgs/Point');
pause(2) %wait to ensure publisher is registered
pubmsg = rosmessage(pub);

while(1)
    pubmsg.X = 1;
    pubmsg.Y = 2;
    pubmsg.Z = 1;
    
    send(pub,pubmsg);
end