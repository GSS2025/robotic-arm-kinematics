clc;

% Derives the general Forward Kinematics of the UR5e robotic arm
% DH Parameters of the 6 joints given

% Equation found in the slides/textbook
function A = FK(theta, a, d, alpha)
    A = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta);
         sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha)   a*sin(theta);
         0           sin(alpha)             cos(alpha)              d;
         0           0                      0                       1];
end

% individual matrices multiplied to find the full FK matrix
h01 = FK(0, 0, 0.1625, pi/2);
h12 = FK(0, -0.425, 0, 0);
h23 = FK(0, -0.3922, 0, 0);
h34 = FK(0, 0, 0.1333, pi/2);
h45 = FK(0, 0, 0.0997, -pi/2);
h56 = FK(0, 0, 0.0996, pi/2);

H = h01*h12*h23*h34*h45*h56