function xref = xref_interp(x0,x1,v0,v1,dt,N)
%% Interpolation
% Input takes position and velocity at t=0 and t=1, i.e., x(0), x(1), v(0), v(1).
% Returns the interpolation values. NOT function handle.
% 
% syms a b c d
% syms t
% syms f(t)
% 
% syms f0 f1
% syms df0 df1
% 
% f(t) = a*t^3 + b*t^2 + c*t + d
% df(t) = diff(f,t)
% 
% sol = solve ( [ f(0) == f0 , 
% f(1) == f1 , 
% df(0) == df0 , 
% df(1) == df1 ], [a,b,c,d]);
%  
% d == f0
% a + b + c + d == f1
% c == df0
% 3*a + 2*b + c == df1

% [aT;bT;cT;dT] = [0 0 0 1; 1 1 1 1; 0 0 1 0; 3 2 1 0]\[f0T; f1T; df0T; df1T];
% [a b c d] = [f0 f1 df0 df1]*inv([0 0 0 1; 1 1 1 1; 0 0 1 0; 3 2 1 0])'
% %%
% abcd = [x0 x1 v0 v1]*[  2   -3   0   1;
%                        -2    3   0   0;
%                         1   -2   1   0;
%                         1   -1   0   0];                    
% %%
% T = 0:dt:N*dt;
% 
% % f = @(t) abcd*[t.^3; t.^2; t; 1];
% 
% % xref    = abcd*[T.^3; T.^2; T; ones(1,size(T,2))];
% % xdotref = abcd*[3*T.^2; 2*T; ones(1,size(T,2)); zeros(1,size(T,2))];
% 
% xref = abcd*[T.^3 3*T.^2; T.^2 2*T; T ones(1,size(T,2)); ones(1,size(T,2)) zeros(1,size(T,2))]
% 
% %%
% xref = reshape(xref',[10,11]); % 10: state vector dimension, 11: horizon
T=[0 N*dt];
Y=[v0 x0 x1 v1];
T_seq=0:dt:N*dt;
X_pos=spline(T,Y,T_seq);
X_vel=zeros(size(X_pos,1),size(X_pos,2));
X_vel(:,1)=v0;

xref=zeros(10,N+1); xref(:,1)=reshape([x0';v0'],[10 1]);

for i=1:N
    X_vel(:,i+1)=1/dt*(X_pos(:,i+1)-X_pos(:,i));
    xref(:,i+1)=reshape([X_pos(:,i+1)';X_vel(:,i+1)'],[10 1]);
end
