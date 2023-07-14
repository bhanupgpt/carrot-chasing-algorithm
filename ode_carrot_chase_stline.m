function [dydt] = ode_carrot_chase_stline(t,y,w1_x,w1_y,w2_x,w2_y)

%% Initializing Params
dydt = zeros(3,1);

v = 10;
delta = 10;
k = 60;

uav_x = y(1);
uav_y = y(2);
si = y(3);

%% XY Controller 

% Computing the position error
% Distance of point to line (UGV Position - Desired path)

R_u = sqrt((w1_x - uav_x)^2 + (w1_y - uav_y)^2);
theta = atan2((w2_y - w1_y),(w2_x - w1_x));
theta_u = atan2(uav_y - w1_y,uav_x - w1_x);
beta = theta - theta_u;
R = sqrt(R_u^2 - (R_u*sin(beta))^2);
x_target = w1_x + (R+delta)*cos(theta); 
y_target = w1_y + (R+delta)*sin(theta);

si_d = atan2(y_target - uav_y, x_target - uav_x);
u = k*(si_d - si);

% Finally heading angle
si_dot = u;

%% STATE EQUATIONS
dydt(1) = v*cos(si); % y(1) -> uav_x
dydt(2) = v*sin(si); % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si