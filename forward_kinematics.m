% fk_and_gravity_compensation_6dof.m
% 6-DOF manipulator: FK (DH), Potential Energy P, Gravity Torques G(q), Jg/Ja
clc; clear;

%% Link parameters (m) and masses (kg)
l1 = 0.101; 
l2 = 0.2495; 
l3 = 0.25; 
l4 = 0; 
l5 = 0.0956; 
l6 = 0.0655;

mass_baseLink     = 0.52654;
mass_shoulderLink = 0.52932;
mass_armLink      = 0.67459;
mass_elbowLink    = 0.43335;
mass_forearmLink  = 0.13384;
mass_wristLink    = 0.11491;
mass_endeff       = 0.17466;
g = 9.81;

%% Symbols
syms a alpha d theta real
syms theta1 theta2 theta3 theta4 theta5 theta6 real
q = [theta1; theta2; theta3; theta4; theta5; theta6];

% Homogeneous transform (standard DH)
A = [ cos(theta),            -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),  a*cos(theta);
      sin(theta),             cos(theta)*cos(alpha), -cos(theta)*sin(alpha),  a*sin(theta);
      0,                      sin(alpha),             cos(alpha),             d;
      0,                      0,                      0,                      1 ];

%% DH (new version you used)
A1 = subs(A, [a, alpha,   d,           theta], [0,  -pi/2, l1,              theta1]);
A2 = subs(A, [a, alpha,   d,           theta], [l2, 0,     0,               theta2]);
A3 = subs(A, [a, alpha,   d,           theta], [l3, 0,     0,               theta3]);
A4 = subs(A, [a, alpha,   d,           theta], [0, -pi/2,  0,          -pi/2+theta4]);
A5 = subs(A, [a, alpha,   d,           theta], [0,  pi/2, l5,           pi/2+theta5]);
A6 = subs(A, [a, alpha,   d,           theta], [l6, 0,     0,           pi/2+theta6]);

T01 = A1;
T02 = T01*A2;
T03 = T02*A3;
T04 = T03*A4;
T05 = T04*A5;
T06 = T05*A6;

%% End-effector pose / position
Px = T06(1,4);
Py = T06(2,4);
Pz = T06(3,4);

%% Euler Z-Y-X (roll=phi about z, pitch=theta about y, yaw=psi about x)
r11 = T06(1,1); r12 = T06(1,2); r13 = T06(1,3);
r21 = T06(2,1); r22 = T06(2,2); r23 = T06(2,3);
r31 = T06(3,1); r32 = T06(3,2); r33 = T06(3,3);

euler_phi   = atan2(r21, r11);                                        % z
euler_theta = atan2(-r31,  cos(euler_phi)*r11 + sin(euler_phi)*r21);  % y
euler_psi   = atan2( sin(euler_phi)*r13 - cos(euler_phi)*r23, ...
                    -sin(euler_phi)*r12 + cos(euler_phi)*r22);        % x

%% CoM heights (simple segment midpoint along z, consistent with your original)
joint1_z = sym(0);
joint2_z = T01(3,4);
joint3_z = T02(3,4);
joint4_z = T03(3,4);
joint5_z = T04(3,4);
joint6_z = T05(3,4);
ee_z     = T06(3,4);

com1 = 0.5*(joint1_z + joint2_z);
com2 = 0.5*(joint2_z + joint3_z);
com3 = 0.5*(joint3_z + joint4_z);
com4 = 0.5*(joint4_z + joint5_z);
com5 = 0.5*(joint5_z + joint6_z);
com6 = 0.5*(joint6_z + ee_z);

%% Potential energy P(q)
P = mass_shoulderLink*g*com1 ...
  + mass_armLink     *g*com2 ...
  + mass_elbowLink   *g*com3 ...
  + mass_forearmLink *g*com4 ...
  + mass_wristLink   *g*com5 ...
  + mass_endeff      *g*com6;

P = simplify(P);

%% Gravity torques vector: G(q) = ∂P/∂q
Gq = simplify( jacobian(P, q).' );   % 6x1

%% Geometric Jacobian (optional)
z0 = [0;0;1]; 
z1 = T01(1:3,3); z2 = T02(1:3,3); z3 = T03(1:3,3);
z4 = T04(1:3,3); z5 = T05(1:3,3);

o0 = [0;0;0];
o1 = T01(1:3,4); o2 = T02(1:3,4); o3 = T03(1:3,4);
o4 = T04(1:3,4); o5 = T05(1:3,4); o6 = T06(1:3,4);

Jv = [ cross(z0,(o6-o0)), cross(z1,(o6-o1)), cross(z2,(o6-o2)), ...
       cross(z3,(o6-o3)), cross(z4,(o6-o4)), cross(z5,(o6-o5)) ];
Jw = [ z0, z1, z2, z3, z4, z5 ];
Jg = [Jv; Jw];   % 6x6

%% Analytic Jacobian (optional) using your B and Z-Y-X Euler
B = [ cos(euler_phi)*cos(euler_theta),  -sin(euler_phi),  0;
      sin(euler_phi)*cos(euler_theta),   cos(euler_phi),  0;
     -sin(euler_theta),                  0,               1];
Ja = [ eye(3), zeros(3); zeros(3), inv(B) ] * Jg;   % 6x6

%% Numeric example (set a test posture and evaluate)
q_deg = [ 0; -60; 60; 30; 0; 0 ];                   % degrees
q_val = deg2rad(q_deg);                             % radians
subs_map = struct('theta1',q_val(1),'theta2',q_val(2),'theta3',q_val(3), ...
                  'theta4',q_val(4),'theta5',q_val(5),'theta6',q_val(6));

P_num   = double(subs(P,   subs_map));
Gq_num  = double(subs(Gq,  subs_map));              % gravity torques [Nm]
T06_num = double(subs(T06, subs_map));
Jg_num  = double(subs(Jg,  subs_map));
Ja_num  = double(subs(Ja,  subs_map));

disp('--- End-Effector Pose (T06) ---'); disp(T06_num);
fprintf('EE position [m]: Px=%.4f  Py=%.4f  Pz=%.4f\n', double(subs(Px,subs_map)), double(subs(Py,subs_map)), double(subs(Pz,subs_map)));
fprintf('Euler Z-Y-X [deg]: phi=%.2f  theta=%.2f  psi=%.2f\n', ...
        rad2deg(double(subs(euler_phi,subs_map))), ...
        rad2deg(double(subs(euler_theta,subs_map))), ...
        rad2deg(double(subs(euler_psi,subs_map))));

fprintf('Potential Energy [J]: %.6f\n', P_num);
disp('--- Gravity Torques G(q) [Nm] ---'); disp(Gq_num);
% Note: Depending on base/joint axes and gravity direction, G1 may be zero.

% Optional displays
% disp('--- Geometric Jacobian Jg ---'); disp(Jg_num);
% disp('--- Analytic  Jacobian Ja ---'); disp(Ja_num);
