function [dw_dt] = OpenLoopStateSpaceCartPole(t,w,m,M,L,g)

% State Space Model of open loop unactuated Cart Pole System
% There is no external force or torque on the system
% Non-linear state space model and equations of motion

w1 = w(1); % first state: cart horizontal position, meters
w2 = w(2); % second state: pole angular position, radians
w3 = w(3); % third state: cart horizontal speed, meters/second
w4 = w(4); % fourth state: pole angular velocity, radians/second

dw_dt1 = w3;
dw_dt2 = w4;

% expression for pole angular acceleration = num/den
den = (((m*(L)^2)/3) - (((m*L*cos(w2))^2)/(4*(m + M))));
num = (((m*L*sin(w2))/(2))*((w3*w4) + g)) - ((((m*L)^2)*(sin(w2)*cos(w2))*((w4)^2))/(4*(M + m)));

dw_dt4 = num/den;

num1 = (((m*L*sin(w2))/(2))*((w4)^2)) - (((m*L*cos(w2))/(2))*((num/den)));
den1 = M + m;

dw_dt3 = num1/den1;

dw_dt = [dw_dt1;dw_dt2;dw_dt3;dw_dt4];
end