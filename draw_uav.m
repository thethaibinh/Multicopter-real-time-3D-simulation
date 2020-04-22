function draw_uav(phi, the, psi, handles)
% Generate the geometry used to draw the quadcopter
r = 7; d = 30; h = 2.5; %centimeters: rotor dia., quad motor distance from
% center of mass, and rotor height above arms (entirely cosmetic)

% Construct rotor representations
cp8 = [d  0 h].';% m1 rotor center [X Y Z]
cp2 = [0 -d h].';% m4 rotor center
cp1 = [0  d h].';% m2 rotor center
cp7 = [-d 0 h].';% m3 rotor center
p8 = circlePoints(cp8, r, 20); p8 = [p8 p8(:,1)]; % Rotor blade circles
p2 = circlePoints(cp2, r, 20); p2 = [p2 p2(:,1)];
p1 = circlePoints(cp1, r, 20); p1 = [p1 p1(:,1)];
p7 = circlePoints(cp7, r, 20); p7 = [p7 p7(:,1)];

% cp3 = [d/(sqrt(2))  d/(sqrt(2)) h].';% m1 rotor center [X Y Z]
% cp4 = [d/(sqrt(2)) -d/(sqrt(2)) h].';% m4 rotor center
% cp5 = [-d/(sqrt(2))  d/(sqrt(2)) h].';% m2 rotor center
% cp6 = [-d/(sqrt(2)) -d/(sqrt(2)) h].';% m3 rotor center
% p3 = circlePoints(cp3, r, 20); p3 = [p3 p3(:,1)]; % Rotor blade circles
% p4 = circlePoints(cp4, r, 20); p4 = [p4 p4(:,1)];
% p5 = circlePoints(cp5, r, 20); p5 = [p5 p5(:,1)];
% p6 = circlePoints(cp6, r, 20); p6 = [p6 p6(:,1)];

% Motors connecting to center of blade circles
m8 = [d,d;
    0,0;
    h,0];
m2 = [0,0;
    -d,-d;
    h,0];
m1 = [0,0;
    d,d;
    h,0];
m7 = [-d,-d;
    0,0;
    h,0];
%5 6 7 8
% m3 = [d/(sqrt(2)),d/(sqrt(2));
%     d/(sqrt(2)),d/(sqrt(2));
%     h,0];
% m4 = [d/(sqrt(2)),d/(sqrt(2));
%     -d/(sqrt(2)),-d/(sqrt(2));
%     h,0];
% m5 = [-d/(sqrt(2)),-d/(sqrt(2));
%     d/(sqrt(2)),d/(sqrt(2));
%     h,0];
% m6 = [-d/(sqrt(2)),-d/(sqrt(2));
%     -d/(sqrt(2)),-d/(sqrt(2));
%     h,0];
% Construct body plot points
a78 = [ d, -d;
    0,  0;
    0,  0];
%For drawing the body "+" shape

% a45 = [ -d/(sqrt(2)), d/(sqrt(2));
%     d/(sqrt(2)), -d/(sqrt(2));
%     0,  0];

a12 = [ 0,  0;
    d, -d;
    0,  0];

% a36 = [d/(sqrt(2)), -d/(sqrt(2));
%     d/(sqrt(2)), -d/(sqrt(2));
%     0,  0];

% Rib
R = [cos(psi)*cos(the) cos(psi)*sin(the)*sin(phi)-sin(psi)*cos(phi) cos(psi)*sin(the)*cos(phi)+sin(psi)*sin(phi);
    sin(psi)*cos(the) sin(psi)*sin(the)*sin(phi)+cos(psi)*cos(phi) sin(psi)*sin(the)*cos(phi)-cos(psi)*sin(phi);
    -sin(the)         cos(the)*sin(phi)                            cos(the)*cos(phi)];


P8 = R*p8;
P2 = R*p2;
P1 = R*p1;
P7 = R*p7;
A78 = R*a78;
A12 = R*a12;
% P3 = R*p3;
% P4 = R*p4;
% P5 = R*p5;
% P6 = R*p6;
% A45 = R*a45;
% A36 = R*a36;

% Rotate body parts Via Initialized R
M8 = R*m8;
M2 = R*m2;
M1 = R*m1;
M7 = R*m7;
% M3 = R*m3;
% M4 = R*m4;
% M5 = R*m5;
% M6 = R*m6;

orange = [1 0.85 0];
% Plot the quad rotation and ang. velocity and inertial frame velocity vector
cla(handles.attitude_axes);
% axes(handles.attitude_axes)
% ground1 = fill3([-4 4 4 -4],...
%     [-4 -4 4 4],...
%     [-3 -3 -3 -3],'g');
% alpha(ground1,0.3); % Makes the ground "see-through"
plot3(handles.attitude_axes,A78(1,:),A78(2,:),A78(3,:),'k','LineWidth',6); % Body Arm
% plot3(handles.attitude_axes,A45(1,:),A45(2,:),A45(3,:),'k','LineWidth',3); % Body Arm
plot3(handles.attitude_axes,A12(1,:),A12(2,:),A12(3,:),'k','LineWidth',6); % Body Arm
% plot3(handles.attitude_axes,A36(1,:),A36(2,:),A36(3,:),'k','LineWidth',3); % Body Arm

plot3(handles.attitude_axes,M8(1,:),M8(2,:),M8(3,:),'r','LineWidth',6); % Motor
plot3(handles.attitude_axes,M2(1,:),M2(2,:),M2(3,:),'Color',orange,'LineWidth',6); % Motor
plot3(handles.attitude_axes,M1(1,:),M1(2,:),M1(3,:),'Color',orange,'LineWidth',6); % Motor
plot3(handles.attitude_axes,M7(1,:),M7(2,:),M7(3,:),'Color',orange,'LineWidth',6); % Motor
plot3(handles.attitude_axes,P8(1,:),P8(2,:),P8(3,:),'r'); % Motor 8 blades
plot3(handles.attitude_axes,P2(1,:),P2(2,:),P2(3,:),'g'); % Motor 2 blades
plot3(handles.attitude_axes,P1(1,:),P1(2,:),P1(3,:),'g'); % Motor 1 blades
plot3(handles.attitude_axes,P7(1,:),P7(2,:),P7(3,:),'g'); % Motor 7 blades

% plot3(handles.attitude_axes,M3(1,:),M3(2,:),M3(3,:),'Color',orange,'LineWidth',4); % Motor
% plot3(handles.attitude_axes,M4(1,:),M4(2,:),M4(3,:),'Color',orange,'LineWidth',4); % Motor
% plot3(handles.attitude_axes,M5(1,:),M5(2,:),M5(3,:),'Color',orange,'LineWidth',4); % Motor
% plot3(handles.attitude_axes,M6(1,:),M6(2,:),M6(3,:),'Color',orange,'LineWidth',4); % Motor
% plot3(handles.attitude_axes,P3(1,:),P3(2,:),P3(3,:),'g'); % Motor 3 blades
% plot3(handles.attitude_axes,P4(1,:),P4(2,:),P4(3,:),'g'); % Motor 4 blades
% plot3(handles.attitude_axes,P5(1,:),P5(2,:),P5(3,:),'g'); % Motor 5 blades
% plot3(handles.attitude_axes,P6(1,:),P6(2,:),P6(3,:),'g'); % Motor 6 blades

[u,v,w] = sphere();
% colormap([0 1 1]);
u = u(11:end,:);       % Keep top 11 x points
v = v(11:end,:);       % Keep top 11 y points
w = w(11:end,:);       % Keep top 11 z points
radius = 7;
hSurface = surf(handles.attitude_axes, radius*u, radius*v, radius*w, 'FaceAlpha', 0.5,'FaceColor', [0.27 0.79 0.53], 'EdgeColor', 'none' ); % sphere with radius 5 centred at (0,0,0)
direction = [-sin(psi) cos(psi) 0];
rotate(hSurface,direction,the*180/pi);
direction = [-cos(psi) -sin(psi)  0];
rotate(hSurface,direction,-phi*180/pi);


function points = circlePoints(center, radius, numberOfPoints)
% Helper function for plotting points
% Inspired by "circle()" from Peter Corke's MATLAB Robotics Toolbox
c = center.'; % [x;y] location of center
r = radius;
n = numberOfPoints;
% compute points around the circumference
th = (0:n-1)'/n*2*pi; % angles coresponding to each point
x = r*cos(th) + c(1); % x part
y = r*sin(th) + c(2); % y part
points = [x,y].';
if length(c) > 2
    z = ones(size(x))*c(3); % z part
    points = [x, y, z].';
end