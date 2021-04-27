function [dw_dt] = ClosedLoopStateSpace(t,w,A,B,K)
% State Space Model of the closed loop control system (regulator)
% Closed Loop Control System: Plant + Controller
% A: State Weighing matrix/System Matrix of open loop plant
% B: Controller Cost Matrix of open loop plant
% K: Controller Gain Matrix of the regulator applied in the control system

% State Weighing matrix of the closed loop control system
Acl = A - B*K;

dw_dt = Acl*w;
end