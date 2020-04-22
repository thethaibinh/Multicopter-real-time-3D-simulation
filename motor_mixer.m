function [M1, M2, M3, M4, M5, M6, M7, M8] = motor_mixer(Roll, Pitch, Yaw, Thrust)

%===========================Settings & Constants===========================

idle_PWM = 1122;
aircraft_mixer = 3;
% Quad [X] = 0
% Quad [+] = 1
% Hex  [X] = 2

%===============================Quad X Mixer===============================

if aircraft_mixer == 0;        % Quad X
    M1 = ((Roll + Pitch - Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M2 = ((-Roll + Pitch + Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M3 = ((-Roll - Pitch - Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M4 = ((Roll - Pitch + Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M5 = 1122;
    M6 = 1122;
    M7 = 1122;
    M8 = 1122;
elseif aircraft_mixer == 1;    % Quad +
    M1 = ((Pitch - Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M2 = ((-Roll + Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M3 = ((-Pitch - Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M4 = ((Roll + Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M5 = 1122;
    M6 = 1122;
    M7 = 1122;
    M8 = 1122;
elseif aircraft_mixer == 2;    % Hex X
    M1 = ((Pitch + Roll / 2 - Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M2 = ((Pitch - Roll / 2 + Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M3 = ((-Roll - Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M4 = ((-Pitch - Roll / 2 + Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M5 = ((-Pitch + Roll / 2 - Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M6 = ((Roll + Yaw) * Thrust / 2 + Thrust) * 800 + idle_PWM;
    M7 = 1122;
    M8 = 1122;
elseif aircraft_mixer == 3;    % Octa +
    M1 = Pitch - Yaw + Thrust + idle_PWM;    
    M2 = -Pitch - Yaw + Thrust + idle_PWM;
    M3 = (Pitch - Roll)* sqrt(2)/2 + Yaw + Thrust + idle_PWM;    
    M4 = (-Pitch - Roll)* sqrt(2)/2 + Yaw + Thrust + idle_PWM;    
    M5 = (Pitch + Roll)* sqrt(2)/2 + Yaw + Thrust + idle_PWM;    
    M6 = (-Pitch + Roll)* sqrt(2)/2 + Yaw + Thrust + idle_PWM;    
    M7 = (Roll - Yaw + Thrust) + idle_PWM;
    M8 = -Roll - Yaw + Thrust + idle_PWM;
else
    M1 = 1122;
    M2 = 1122;
    M3 = 1122;
    M4 = 1122;
    M5 = 1122;
    M6 = 1122;
    M7 = 1122;
    M8 = 1122;
end
%==========================================================================