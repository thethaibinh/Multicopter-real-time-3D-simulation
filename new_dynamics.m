function [forces, moments] = new_dynamics(pwm, DCM, p, Vb)

%================================Constants=================================

radsec = 0.104719755;
gravity = 9.81;
radians = pi / 180;

% Aircraft Parameters
motor_thrust = 1.450;                   % Maximum thrust of a single motor      (kg)
motor_constant = 470;                   % Motor revolutions per volt            (KV)
motor_voltage = 22.2;                   % Nominal motor voltage                 (V)
motor_efficiency = 0.8;                 % Effective velocity
rotor_diameter = 0.254;                  % Diameter of the rotor                 (meters)                                 
aircraft_mass = 4.0;                    % Aircraft mass                         (kg)                                        
aircraft_diameter = 0.8;                % Motor to motor distance               (meters)                                      
aircraft_dihedral = 0;                  % Angle from base to arm                (degrees)
aircraft_mixer = 3;                     % Rotor position and rotation      
                                        % Quad [X] = 0
                                        % Quad [+] = 1
                                        % Hex  [X] = 2
                                        % Octo [+] = 3
%==============================Inertia Tensor==============================
                                    
% Octocopter BKAeroHelper - 4.0kg (no load), MN3110-26 KV470 Motor

%==============================Initialization==============================

persistent initialize;

g = [0; 0; gravity];
t = zeros(1, 8);

if isempty(initialize)
    DCM = zeros(3);
    initialize = 1;
end

for n = 1:8
    t(n) = (pwm(n) - 1122) / 800;
end
       
%=================================Actuators================================

% [enabled, x(roll), y(pitch), z(yaw)]

M = zeros(8, 4);

if aircraft_mixer == 0
    M(1, :) = [1;  1;  1; -1];
    M(2, :) = [1, -1,  1,  1];
    M(3, :) = [1, -1, -1, -1];
    M(4, :) = [1,  1, -1,  1];
    M(5, :) = [0,  0,  0,  0];
    M(6, :) = [0,  0,  0,  0];
    M(7, :) = [0,  0,  0,  0];
    M(8, :) = [0,  0,  0,  0];
elseif aircraft_mixer == 1
    M(1, :) = [1;  0;  1; -1];
    M(2, :) = [1, -1,  0,  1];
    M(3, :) = [1,  0, -1, -1];
    M(4, :) = [1,  1,  0,  1];
    M(5, :) = [0,  0,  0,  0];
    M(6, :) = [0,  0,  0,  0];
    M(7, :) = [0,  0,  0,  0];
    M(8, :) = [0,  0,  0,  0];
elseif aircraft_mixer == 2
    M(1, :) = [1;  1;  1; -1];
    M(2, :) = [1, -1,  1,  1];
    M(3, :) = [1, -1,  0, -1];
    M(4, :) = [1, -1, -1,  1];
    M(5, :) = [1,  1, -1, -1];
    M(6, :) = [1,  1,  0,  1];
    M(7, :) = [0,  0,  0,  0];
    M(8, :) = [0,  0,  0,  0];
elseif aircraft_mixer == 3   % Octo [+]
    M(1, :) = [1;  0;  1; -1];
    M(2, :) = [1,  0, -1, -1];
    M(3, :) = [1, -sqrt(2)/2,  sqrt(2)/2, 1];
    M(4, :) = [1, -sqrt(2)/2, -sqrt(2)/2, 1];
    M(5, :) = [1,  sqrt(2)/2,  sqrt(2)/2, 1];
    M(6, :) = [1,  sqrt(2)/2, -sqrt(2)/2, 1];
    M(7, :) = [1,  1,  0, -1];
    M(8, :) = [1, -1,  0, -1];
else
    M(1, :) = [0;  0;  0;  0];
    M(2, :) = [0,  0,  0,  0];
    M(3, :) = [0,  0,  0,  0];
    M(4, :) = [0,  0,  0,  0];
    M(5, :) = [0,  0,  0,  0];
    M(6, :) = [0,  0,  0,  0];
    M(7, :) = [0,  0,  0,  0];
    M(8, :) = [0,  0,  0,  0];
end

%==================================Forces==================================

F_z = zeros(1, 8);

for n = 1:8
    F_z(n) = t(n) * motor_thrust * gravity * M(n, 1)...                     % rotor forces
    * sin((90 - aircraft_dihedral) * radians);                         
end

A = [rotor_diameter * aircraft_dihedral,...                          % rotor cross sectional area
    aircraft_diameter ^ 2 / 8,...                                    % body cross sectional area (X) (default: /10)
    aircraft_diameter ^ 2 / 8,...                                    % body cross sectional area (Y) (default: /10)
    pi * ((aircraft_diameter - rotor_diameter) / 2) ^ 2];            % body cross sectional area (Z)

F_d = zeros(1, 3);

for n = 1:3
    F_d(n) = p / 2 * A(n + 1) * (Vb(n)) ^ 2;
    if Vb(n) > 0
        F_d(n) = F_d(n) * -1;
    end
end

forces = zeros(3, 1);

F_g = aircraft_mass .* (DCM * g);                                           % gravitational force
forces(1) = F_g(1) + F_d(1);
forces(2) = F_g(2) + F_d(2);
forces(3) = F_g(3) - sum(F_z) + 2 * F_d(3);

%==================================Moments=================================

m = zeros(8, 4);

for n = 1:8
    m(n, :) = F_z(n) .* M(n, :);
end

m = (aircraft_diameter / 2) .* m;                                           % x & y moments

K = motor_constant * motor_voltage * motor_efficiency * radsec;
Vr = zeros(1, 8);

for n = 1:8    
    Vr(n) = t(n) * K;                                                       % rotor angular velocity
end

d = zeros(1, 8);

for n = 1:8
    d(n) =  M(n, 4) * p / 2 * A(1) * (Vr(n) * rotor_diameter / 10) ^ 2;     % aerodynamic drag force of the rotor
end

T = zeros(1, 8);

for n = 1:8
    T(n) = d(n);                                                            % total rotor torque
end

moments = zeros(3, 1);

moments(1) = sum(m(:, 2));
moments(2) = sum(m(:, 3));
moments(3) = sum(T); 

%==========================================================================