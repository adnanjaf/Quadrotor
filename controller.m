function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% =================== Your code goes here ===================
% ...
% ==============================

% Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% When you are flying in the lab, they *will* be used (because the platform
% has a built-in attitude controller). Best to fill them in already
% during simulation.

% Position Control Gains

% kP_x = 32; kP_y = 32; kP_z = 7;
% kD_x = 10; kD_y = 10; kD_z = 3;
% kP_x = 33; kP_y = kP_x; kP_z = 7;
% kD_x = 10; kD_y = kD_x; kD_z = 3;

% Last set that worked
% kP_x = 10; kP_y = kP_x; kP_z = 7;
% kD_x = 4; kD_y = kD_x; kD_z = 3;
% K_r = 100; K_w = 10;

kP_x = 15; kP_y = kP_x; kP_z = 30;
kD_x = 12; kD_y = kD_x; kD_z = 10;
% 
kP = [kP_x 0 0; 0 kP_y 0; 0 0 kP_z];
kD = [kD_x 0 0; 0 kD_y 0; 0 0 kD_z];
% 
% % Attitude Control Gains
% % K_r = 500; K_w = 50;
K_r = 3000; K_w = 300;
K_r = K_r*eye(3); K_w = K_w*eye(3);

% Parameters
g = params.grav; m = params.mass; I = params.I;

% Position, Velocity and Acceleration (Trajectory)
r_ddot_T = qd{1}.acc_des; %% Trajectory Acceleration
r_dot_T = qd{1}.vel_des;  %% Trajectory Velocity
r_T = qd{1}.pos_des; %% Trajectory Position

% Position, Velocity (Actual)
r_dot = qd{1}.vel;
r = qd{1}.pos;

% Angles and Angular Velocity
angles = qd{1}.euler;
R = eulzxy2rotmat(angles);
psi_T = qd{1}.yaw_des;

% Compute desired acceleration vector (Eqn. 22)
r_ddot = r_ddot_T - kD*(r_dot - r_dot_T) - kP*(r - r_T);

% Solve for desired force (Eqn. 28)        
F_des = m*r_ddot + [0;0;m*g];

% Solve for u1 (Eqn. 29)
u    = zeros(4,1); % control input u, you should fill this in
b3 = R*[0;0;1];
u(1) = b3'*F_des;

% Thrust
F = u(1);       % This should be F = u(1) from the project handout

% Determine b3_des (Eqn. 30)
b3_des = F_des/norm(F_des);

% Determine a_psi (Eqn. 31)
a_psi = [cos(psi_T); sin(psi_T); 0];

% Determine b2_des (Eqn. 32)
b2_des = cross(b3_des,a_psi)/norm(cross(b3_des,a_psi));

% Determine R_des (Eqn. 33)
R_des = [cross(b2_des,b3_des),b2_des,b3_des];

% Find orientation error vector e_R (Eqn. 34)
e_R = 0.5*veemap(R_des'*R - R'*R_des);

% Find control input u(2:4) (Eqn. 35)
omega = qd{1}.omega;
omega_des = [0;0;0];
e_w = omega - omega_des;
u(2:4) = I*(-K_r*e_R' - K_w*e_w);
 
% Moment
M = u(2:4);     

% Convert R_des to RPY 
RPY=rotmat2eulzxy(R_des);

phi_des   = RPY(1);
theta_des = RPY(2);
psi_des   = RPY(3);

F = m*(g+r_ddot(3));
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, omega_des(1), omega_des(2), omega_des(3)];

end

%
% ------------------------------------------------------------
%    should you decide to write a geometric controller,
%    the following functions should come in handy
%

function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
