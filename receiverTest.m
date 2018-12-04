%initialize receiver
receiver1 = rossubscriber('/VehicleAngVelo');
pause(1)
receiver2 = rossubscriber('/VehicleAtt');
pause(1)
receiver3 = rossubscriber('/VehiclePos');
pause(1)
receiver4 = rossubscriber('/VehicleVelo');
pause(1)

while(1)
    pos = receive(receiver3)
    Att = receive(receiver2)
    Vel = receive(receiver4)
    AngVel = receive(receiver1)
    
end