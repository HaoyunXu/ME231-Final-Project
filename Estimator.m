function [xchat, thetachat, vchat] = Estimator(History, Vbar, Thetaleftbar, Thetarightbar, betav, betaleft, betaright, N, dt)
% Computing averages
vL=History(1,:);
dL=History(2,:);
vtilde=mean(vL);
dtilde=mean(dL);
vbar=(1-betav)*Vbar+vtilde*betav;
dleftbar=(1-betaleft)*Thetaleftbar+dtilde*betaleft;
drightbar=(1-betaright)*Thetarightbar+dtilde*betaright;
% A possible problem could be the sign of thetaleft or thetaright. Check if
% we are letting variables take care of the signs or whether we are
% focussing on + or - for left and right variables.

% Computing thetaleftbar and thetarightbar (notice the lower case)
sinsumleft=0;
cossumleft=0;
sinsumright=0;
cossumright=0;
for k=1:N
    sinsumleft=sinsumleft+sin(k*dleftbar);
    cossumleft=cossumleft+cos(k*dleftbar);
    sinsumright=sinsumright+sin(k*drightbar);
    cossumright=cossumright+cos(k*drightbar);
end

thetaleftbar=atan2(sinsumleft,cossumleft);
thetarightbar=atan2(sinsumright,cossumright);

R=vbar*N*dt;
thetaencl=thetaleftbar+thetarightbar;
xchat=R/(1+sin(thetaencl/2));
thetachat=thetarightbar-thetaencl/2;
vchat=vbar; %?
end
