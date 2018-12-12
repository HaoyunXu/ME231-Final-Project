 function X_wp = MPC(x0, Xref,xc, V_bar)
 %% Simulation Parameters
 N=10;
 del_T=0.01;
 
 %% Cost Parameters
 Q=eye(10);R=2*eye(3);
 P=10*Q;
 Xref=zeros(10,N+1);
 
 
 %% System Model Parameters
 
 % State Vector = [X Vx Pitch Pitch_Rate Y Vy Roll Roll_Rate Z Vz]^T
 bp0=1;
 bp1=1;
 g=9.8;
 br0=1;
 br1=1;
 A=blkdiag([0 1 0 0;0 0 g 0;0 0 0 1;0 0 -bp0 -bp1],[0 1 0 0;0 0 -g 0;0 0 0 1;0 0 -br0 -br1],[0 1;0 0]);
 ap=1;
 ar=1;
 m=1;
 B=[zeros(3,3);-ap 0 0;zeros(3,3);0 -ar 0;0 0 0;0 0 1/m];
 G=[zeros(9,1);-g];
 % Computing Discrete time system matrices
 Temp=ss(A,B,[],[]);
 AT=Temp.A;
 BT=Temp.B;
 Temp=ss(A,G,[],[]);
 GT=Temp.B;
 
 %% Constraint Sets
 % Input Constraints
 uL=[-1/3*pi*ones(2,1);0];uU=[1/3*pi*ones(2,1);5];
 % State Constraints of the form A_ie*X<=b_ie
 A_ie=[0 0 1 0 0 0 0 0 0 0;
       0 0 -1 0 0 0 0 0 0 0;
       0 0 0 1 0 0 0 0 0 0;
       0 0 0 -1 0 0 0 0 0 0;
       0 0 0 0 0 0 1 0 0 0;
       0 0 0 0 0 0 -1 0 0 0;
       0 0 0 0 0 0 0 1 0 0;
       0 0 0 0 0 0 0 -1 0 0;
       0 0 0 0 0 0 0 0 -1 0
       0 0 0 0 0 0 0 0 1 0];
 max_roll=1/3*pi; max_pitch=1/3*pi;
 max_p=pi/2;max_q=pi/2;
 b_ie=[kron([max_pitch;max_q;max_roll;max_p],ones(2,1));0;200];
 % Terminal Set Computation
 xcl=xc-V_bar*N*del_T*ones(2,1);xcu=xc+V_bar*N*del_T*ones(2,1); 
 A_f=[  1             zeros(1,9); 
      zeros(1,4)  1   zeros(1,5);
       -1             zeros(1,9);
      zeros(1,4) -1   zeros(1,5);
      A_ie];
 b_f=[xcu;-xcl;b_ie];
 
%% Solving CFTOC 
 [feas, xtemp, utemp, JOpt] = solve_cftoc(AT, BT, GT, P, Q, R, N, x0, A_ie, b_ie, uL, uU, A_f, b_f, Xref);                                     
 disp(feas);
 if feas==true
     X_wp=xtemp(:,2);
 else
     warning('CFTOC is infeasible at ')
     x0
 end                                 
 end
 
 
 function [feas, xOpt, uOpt, JOpt] = solve_cftoc(AT, BT, GT, P, Q, R, N, x0, A_ie, b_ie, uL, uU, A_f, b_f, Xref)

nX = size(AT,1);
X=sdpvar(nX,N+1);
nU = size(BT,2);
U=sdpvar(nU,N);
Constraints=[X(:,1)==x0];
Cost=(X(:,N+1)-Xref(:,N+1))'*P*(X(:,N+1)-Xref(:,N+1));
for i=1:N
    Cost=Cost+(X(:,i)-Xref(:,i))'*Q*(X(:,i)-Xref(:,i))+U(:,i)'*R*U(:,i);
    Constraints=Constraints+[X(:,i+1)==AT*X(:,i)+BT*U(:,i)+GT];
    Constraints=Constraints+[uL<=U(:,i)<=uU];
    %     [A_f*X(:,i)<=b_f];
    %     [A_ie*X(:,i)<=b_ie];
end
sol=optimize(Constraints, Cost);
if sol.problem==1
    feas=false;
    xOpt=[];uOpt=[];JOpt=[];
else
    feas=true;
    xOpt=double(X);
    uOpt=double(U);
    JOpt=double(Cost);
end
end
