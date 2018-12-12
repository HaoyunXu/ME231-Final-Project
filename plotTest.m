%initial conditions
xc=0; yc=0; vc=0; psi_c=0;
a = rand(1,1000)*2-1; %between -1 and 1
deltaF = rand(1,1000)*pi/3-pi/6;%steering angle is between -30 deg and 30 deg
i = 1;
figure
while(i<1000)
    [xc,yc,vc,psi_c] = bikeFE(xc,yc,vc,psi_c,a(i),deltaF(i));
    i = i+1;
    scatter3(xc,yc,0,1,[1 0 0])
    hold on
end
hold off