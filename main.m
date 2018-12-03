%initialize receiver
receiver = rossubscriber('/pose');

%initialize publisher
pub = rospublisher('/pose','geometry_msgs/Point');
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
    [xc,yc,vc,psi_c] = bikeFE(xc,yc,vc,psi_c,a(i),deltaF(i));
    i = i+1;
    
    %Record a history of v and psi
    History(:,1:L-1)=History(:,2:L);
    History(:,L)=[vc;psi_c];
    
    %esitmation of car positions
    [xc_hat, thetac_hat, vc_hat] = Estimator(History, Vbar, Thetaleftbar, Thetarightbar, N, dt);
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    % time-out in 1000 seconds
    posData = receive(receiver,1000);
    
    
    
    
    
end