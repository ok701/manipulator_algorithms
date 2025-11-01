% 2-link planar manipulator CoM Jacobian control (FIXED)
clc; clear;

g = 9.81;  L1 = 0.25; L2 = 0.25;
m1 = 0.25; m2 = 0.25;
d1 = L1/2; d2 = L2/2;
h  = 0.06;
step_time = 500;
del_t = 0.002;

%------Initial Settings------
Init_th = [pi/3, -pi/2]; % [theta1 theta2]
I1 = (m1*(L1^2+h^2))/12;
I2 = (m2*(L2^2+h^2))/12;

A0 = eye(4);
A1 = [cos(Init_th(1)) -sin(Init_th(1)) 0 L1*cos(Init_th(1));
      sin(Init_th(1))  cos(Init_th(1)) 0 L1*sin(Init_th(1));
      0                0               1 0;
      0                0               0 1];
A2 = [cos(Init_th(2)) -sin(Init_th(2)) 0 L2*cos(Init_th(2));
      sin(Init_th(2))  cos(Init_th(2)) 0 L2*sin(Init_th(2));
      0                0               1 0;
      0                0               0 1];

T00 = A0;
T01 = T00*A1;
T02 = T01*A2;
T12 = T02 - T01;

% link CoM (init)
CoM1 = [0.5*T01(1,4), 0.5*T01(2,4), 0];
CoM2 = [T01(1,4)+0.5*T12(1,4), T01(2,4)+0.5*T12(2,4), 0];

m  = m1 + m2;

CoM_total = [(m1*CoM1(1)+m2*CoM2(1))/m, (m1*CoM1(2)+m2*CoM2(2))/m];
init_CoM  = CoM_total;

CoM_x = zeros(1,step_time);
CoM_y = zeros(1,step_time);
CoM_x(1) = init_CoM(1);
CoM_y(1) = init_CoM(2);

theta_1     = zeros(1,step_time); theta_1(1) = Init_th(1);
theta_1_dot = zeros(1,step_time);
theta_1_ddot= zeros(1,step_time);

theta_2     = zeros(1,step_time); theta_2(1) = Init_th(2);
theta_2_dot = zeros(1,step_time);
theta_2_ddot= zeros(1,step_time);

%-----Reference CoM via joint reference (cosine profile)------
for i = 1:step_time
    s = (i-1)/step_time; % 0~1 normalized  (고침: theta_2도 동일 정규화 사용)
    theta_1(i) = Init_th(1) + deg2rad(10)*0.5*(1 - cos(pi*s));
    theta_2(i) = Init_th(2) + deg2rad(30)*0.5*(1 - cos(pi*s));

    if i >= 2
        theta_1_dot(i) = (theta_1(i) - theta_1(i-1))/del_t;
        theta_2_dot(i) = (theta_2(i) - theta_2(i-1))/del_t;
    end
    if i >= 3
        theta_1_ddot(i) = (theta_1_dot(i) - theta_1_dot(i-1))/del_t;
        theta_2_ddot(i) = (theta_2_dot(i) - theta_2_dot(i-1))/del_t;
    end

    % FK with reference joints (for reference CoM)
    A1 = [cos(theta_1(i)) -sin(theta_1(i)) 0 L1*cos(theta_1(i));
          sin(theta_1(i))  cos(theta_1(i)) 0 L1*sin(theta_1(i));
          0                0               1 0;
          0                0               0 1];
    A2 = [cos(theta_2(i)) -sin(theta_2(i)) 0 L2*cos(theta_2(i));
          sin(theta_2(i))  cos(theta_2(i)) 0 L2*sin(theta_2(i));
          0                0               1 0;
          0                0               0 1];

    T00 = eye(4);
    T01 = T00*A1;
    T02 = T01*A2;
    T12 = T02 - T01;

    CoM1 = [0.5*T01(1,4), 0.5*T01(2,4), 0];
    CoM2 = [T01(1,4)+0.5*T12(1,4), T01(2,4)+0.5*T12(2,4), 0];

    CoM_x(i) = (m1*CoM1(1)+m2*CoM2(1))/m;
    CoM_y(i) = (m1*CoM1(2)+m2*CoM2(2))/m;
end

%-----CoM control Initial parameters------
Kp = [2000, 5000];
Kd = [60,   150 ];

act_th      = zeros(2,step_time+1);  act_th(:,1)      = Init_th(:);
act_th_dot  = zeros(2,step_time+1);
act_th_ddot = zeros(2,step_time);

act_CoM_x      = zeros(1,step_time); act_CoM_x(1) = init_CoM(1);
act_CoM_y      = zeros(1,step_time); act_CoM_y(1) = init_CoM(2);
act_CoM_x_dot  = zeros(1,step_time);
act_CoM_y_dot  = zeros(1,step_time);

% (선택) 작은 점성 감쇠로 수치 안정성 향상
B = diag([0.02, 0.02]);

%-----Figure (핸들 재사용)------
figure(1); clf;
axis equal; grid on; hold on;
axis([-0.1 0.7 -0.1 0.7]);
xlabel('x [m]'); ylabel('y [m]');
title('2-link planar manipulator CoM');

% 초기 드로잉
p0 = [0;0]; 
p1 = [L1*cos(act_th(1,1)); L1*sin(act_th(1,1))];
p2 = [L1*cos(act_th(1,1))+L2*cos(sum(act_th(:,1))); ...
      L1*sin(act_th(1,1))+L2*sin(sum(act_th(:,1)))];

h_l1 = plot([p0(1), p1(1)], [p0(2), p1(2)], 'LineWidth',2);
h_l2 = plot([p1(1), p2(1)], [p1(2), p2(2)], 'LineWidth',2);
h_c1 = plot(NaN,NaN,'bo','MarkerFaceColor','b'); % link1 CoM
h_c2 = plot(NaN,NaN,'go','MarkerFaceColor','g'); % link2 CoM
h_c  = plot(init_CoM(1), init_CoM(2), 'ro','MarkerFaceColor','r'); % total CoM

%-----CoM control loop------
for i = 1:step_time
    q1 = act_th(1,i);  q2 = act_th(2,i);
    % current FK
    A1 = [cos(q1) -sin(q1) 0 L1*cos(q1);
          sin(q1)  cos(q1) 0 L1*sin(q1);
          0        0       1 0;
          0        0       0 1];
    A2 = [cos(q2) -sin(q2) 0 L2*cos(q2);
          sin(q2)  cos(q2) 0 L2*sin(q2);
          0        0       1 0;
          0        0       0 1];

    T00 = eye(4);
    T01 = T00*A1;
    T02 = T01*A2;
    T12 = T02 - T01;

    % gravity torques (올바른 G 사용)
    G1 = (m1*g*d1 + m2*g*L1)*cos(q1) + m2*g*d2*cos(q1+q2);
    G2 =  m2*g*d2*cos(q1+q2);
    G   = [G1; G2];

    % link CoM (현재 자세)
    CoM1 = [0.5*T01(1,4), 0.5*T01(2,4), 0];
    CoM2 = [T01(1,4)+0.5*T12(1,4), T01(2,4)+0.5*T12(2,4), 0];

    % total CoM (현재)
    act_CoM_x(i) = (m1*CoM1(1)+m2*CoM2(1))/m;
    act_CoM_y(i) = (m1*CoM1(2)+m2*CoM2(2))/m;

    % CoM 속도 (유한차분)
    if i >= 2
        act_CoM_x_dot(i) = (act_CoM_x(i) - act_CoM_x(i-1))/del_t;
        act_CoM_y_dot(i) = (act_CoM_y(i) - act_CoM_y(i-1))/del_t;
    end

    % Jacobian columns (기하학적: cross)
    z0 = T00(1:3,3); p0w = T00(1:3,4);
    z1 = T01(1:3,3); p1w = T01(1:3,4);

    p_com = [act_CoM_x(i); act_CoM_y(i); 0];
    p_c2  = [CoM2(1); CoM2(2); 0];

    % d(CoM)/dq1 = z0 × (p_com - p0)
    Jcol1 = cross(z0, (p_com - p0w));
    % d(CoM)/dq2 = (m2/m) * z1 × (p_c2 - p1)
    Jcol2 = (m2/m) * cross(z1, (p_c2 - p1w));

    J = [Jcol1, Jcol2];   % 3×2 (z성분 0이므로 x,y만 실제 사용)

    % Task-space PD force at CoM
    ex = CoM_x(i) - act_CoM_x(i);
    ey = CoM_y(i) - act_CoM_y(i);
    evx = -act_CoM_x_dot(i);
    evy = -act_CoM_y_dot(i);
    F = [Kp(1)*ex + Kd(1)*evx;
         Kp(2)*ey + Kd(2)*evy;
         0];

    % Joint torques with gravity compensation
    tau = J.'*F + G;

    % Dynamics (FIX: cos(q2) 현재값 사용)
    q1d = act_th_dot(1,i); q2d = act_th_dot(2,i);

    M11 = I1 + I2 + m1*d1^2 + m2*(L1^2 + d2^2 + 2*L1*d2*cos(q2));
    M12 = I2 + m2*(d2^2 + L1*d2*cos(q2));
    M21 = M12;
    M22 = I2 + m2*d2^2;
    Mmat = [M11 M12; M21 M22];

    % Coriolis/centrifugal (표준 2링크)
    h = -m2*L1*d2*sin(q2);
    Cvec = [ h*(2*q1d*q2d + q2d^2);
            -h*(q1d^2) ];

    % 작은 점성 감쇠
    tau_damp = B * [q1d; q2d];

    % Forward dynamics (semi-implicit Euler)
    qdd = Mmat \ (tau - Cvec - G - tau_damp);
    act_th_ddot(:,i)   = qdd;
    act_th_dot(:,i+1)  = act_th_dot(:,i) + del_t*qdd;
    act_th(:,i+1)      = act_th(:,i)     + del_t*act_th_dot(:,i+1);

    % ----- draw -----
    p1 = [L1*cos(act_th(1,i)); L1*sin(act_th(1,i))];
    p2 = [L1*cos(act_th(1,i))+L2*cos(sum(act_th(:,i))); ...
          L1*sin(act_th(1,i))+L2*sin(sum(act_th(:,i)))];

    set(h_l1,'XData',[0, p1(1)], 'YData',[0, p1(2)]);
    set(h_l2,'XData',[p1(1), p2(1)], 'YData',[p1(2), p2(2)]);
    set(h_c1,'XData', CoM1(1), 'YData', CoM1(2));
    set(h_c2,'XData', CoM2(1), 'YData', CoM2(2));
    set(h_c, 'XData', act_CoM_x(i),'YData', act_CoM_y(i));
    drawnow;
end

%-----Plots------
figure(2); clf;
i = 1:step_time;
subplot(2,1,1);
plot(i, CoM_x, 'r.', i, act_CoM_x, 'b.'); grid on;
title('CoM position X'); legend('Ref CoM X','Act CoM X');

subplot(2,1,2);
plot(i, CoM_y, 'r.', i, act_CoM_y, 'b.'); grid on;
title('CoM position Y'); legend('Ref CoM Y','Act CoM Y');

% (참고) 실제 중력 vs. 잘못 썼던 partial diff 비교 제거
% 필요하면 아래처럼 테이블 만들 수 있음:
% varNames = ["G1","G2"];
% T = table((G1_hist)', (G2_hist)', 'VariableNames', varNames)
