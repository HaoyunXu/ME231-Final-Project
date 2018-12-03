function [xp,yp,vp,psip] = bikeFE(x,y,v,psi,a,deltaF,TS=0.1)
    beta = atan(1/2*tan(deltaF));
    xp = x + TS*(v*cos(psi+beta));
    yp = y + TS*(v*sin(psi+beta));
    vp = v + TS*(a);
    psip = psi + TS*(v/1.738*sin(beta));
    
end