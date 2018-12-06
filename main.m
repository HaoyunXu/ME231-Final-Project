%initialize receiver
receiver1 = rossubscriber('/VehicleAngVelo');
pause(1)
receiver2 = rossubscriber('/VehicleAtt');
pause(1)
receiver3 = rossubscriber('/VehiclePos');
pause(1)
receiver4 = rossubscriber('/VehicleVelo');
pause(1)
%initialize publisher
pub = rospublisher('/mpcinput','geometry_msgs/Point');
pause(2) %wait to ensure publisher is registered
pubmsg = rosmessage(pub);

%initial conditions
xc=0; yc=0; vc=0; psi_c=0;
a = rand(1,1000)*2-1; %between -1 and 1
deltaF = rand(1,1000)*pi/3-pi/6;%steering angle is between -30 deg and 30 deg
i = 1;

L=10;
History = zeros(2,L);

N=10; %MPC horizon
dt = 0.1; % sampling time

%bounds
Vbar = 1;
Thetaleftbar = -pi/6;
Thetarightbar = pi/6;
while(1)
    pos = receive(receiver3);
    Att = receive(receiver2);
    Vel = receive(receiver4);
    AngVel = receive(receiver1);
    disp('received')
    droneState=[pos.X;Vel.X;Att.X;AngVel.X;pos.Y;Vel.Y;Att.Y;AngVel.Y;pos.Z;Vel.Z];
    
    [xc,yc,vc,psi_c] = bikeFE(xc,yc,vc,psi_c,a(i),deltaF(i));
    i = i+1;
    
    %Record a history of v and psi
    History(:,1:L-1)=History(:,2:L);
    History(:,L)=[vc;psi_c];
    
    %esitmation of car positions
    [xc_hat, thetac_hat, vc_hat] = Estimator(History, Vbar, Thetaleftbar, Thetarightbar, N, dt);
    
    %path generation
    f = xref_interp([pos.X;pos.Y],[xc_hat;0],[Vel.X;Vel.Y],[vc_hat;0],dt,N);
    
    %path following with MPC
    X_wp = MPC(droneState(1:10), f , [xc;yc] , Vbar);
    
    %extract desired waypoint for low level control
    pubmsg.X = X_wp(1);
    pubmsg.Y = X_wp(5);
    display(pubmsg.X,pubmsg.Y)
    
    send(pub,pubmsg);   
    
end