% === 3-링크 플래너 매니퓰레이터 IK + 동역학, 애니메이션 & 요약 플롯 ===

%% 기본 setup
clear; clc;

% step마다 걸리는 시간
step_1_time = 0.7;
step_2_time = 1.5;
step_3_time = 2.0;
step_4_time = 3.5;
step_5_time = 3.5;
step_6_time = 2.0;

del_t = 0.005;              % [sec] 샘플링

% 3개의 link
a1 = 0.250; a2 = 0.250; a3 = 0.150; % [m]
m1 = 1.0; m2 = 0.5; m3 = 0.5;       % [kg]
lc1 = a1/2; lc2 = a2/2; lc3 = a3/2; % [m]
g = 9.81;
reach_dis = 0.07;

% mass moment of inertia (각 링크 길이 사용)
I1 = (m1*a1^2)/12;
I2 = (m2*a2^2)/12;
I3 = (m3*a3^2)/12;

% 애니메이션 figure
f1 = figure('Name','Animation');
plot(0,0,'ko'); hold on; grid on;
axis([-0.7 0.7 -0.5 0.8]); axis equal;
title('3-Link Planar Manipulator');

% 최초 EE 위치
theta0_1 = pi/4;
theta0_2 = pi/4+pi*3/2;
theta0_3 = pi/4+pi*3/2+pi*7/4;
x_0 = [0, ...
       a1*cos(theta0_1), ...
       a1*cos(theta0_1)+a2*cos(theta0_2), ...
       a1*cos(theta0_1)+a2*cos(theta0_2)+a3*cos(theta0_3)];
y_0 = [0, ...
       a1*sin(theta0_1), ...
       a1*sin(theta0_1)+a2*sin(theta0_2), ...
       a1*sin(theta0_1)+a2*sin(theta0_2)+a3*sin(theta0_3)];
p = line(x_0,y_0,'LineWidth',4); % (신버전 호환)

%% 저장 컨테이너 (스텝 요약용)
Step = struct('t',[],'x',[],'y',[], ...
              'th1',[],'th2',[],'th3',[], ...
              'th1dot',[],'th2dot',[],'th3dot',[], ...
              'T',[],'P',[],'t_TP',[]);

% 전역 누적 토크/파워 버퍼
T = zeros(3, 0);
P = zeros(3, 0);

%% 메인 루프
count = 1;
for step = 1:6
    % 스텝별 시간
    switch count
        case 1, step_time = step_1_time;
        case 2, step_time = step_2_time;
        case 3, step_time = step_3_time;
        case 4, step_time = step_4_time;
        case 5, step_time = step_5_time;
        case 6, step_time = step_6_time;
    end

    % 이 스텝의 샘플 수 (스텝 길이에 맞춤)
    Ns = max(2, round(step_time/del_t));

    % 로컬 버퍼
    x  = zeros(1,Ns);
    y  = zeros(1,Ns);
    th1 = zeros(1,Ns);
    th2 = zeros(1,Ns);
    th3 = zeros(1,Ns);
    th1dot = zeros(1,Ns);
    th2dot = zeros(1,Ns);
    th3dot = zeros(1,Ns);
    th1ddot = zeros(1,Ns);
    th2ddot = zeros(1,Ns);
    th3ddot = zeros(1,Ns);

    % T,P 인덱스 시작점
    idx0 = size(T,2) + 1;

    for i = 1:Ns
        % 스텝 내 시간 (0, del_t, 2*del_t, ...)
        t = (i-1) * del_t;

        % ===== 경로/자세 계획 =====
        if count == 1
            % 수직 하강, 말단 각도 유지
            x(i) = x_0(4);
            y(i) = y_0(4) - reach_dis*0.5*(1 - cos(pi/step_1_time * t));
            theta_d = 3*pi/2;

            D = ((x(i)-a3*cos(theta_d))^2 + (y(i)-a3*sin(theta_d))^2 - a1^2 - a2^2)/(2*a1*a2);
            D = max(-1,min(1,D));
            th2(i) = atan2(-sqrt(1-D^2), D);
            th1(i) = atan2((y(i)-a3*sin(theta_d)), (x(i)-a3*cos(theta_d))) - atan2(a2*sin(th2(i)), a1+a2*cos(th2(i)));
            th3(i) = 3/2*pi - (th1(i)+th2(i));

            if i == Ns
                step3_init_x = x(i);
                step3_init_y = y(i);
            end

        elseif count == 2
            % 상승
            x(i) = x_0(4);
            y(i) = y_0(4) - reach_dis + reach_dis*0.5*(1 - cos(pi/step_2_time * t));
            theta_d = 3*pi/2;

            D = ((x(i)-a3*cos(theta_d))^2 + (y(i)-a3*sin(theta_d))^2 - a1^2 - a2^2)/(2*a1*a2);
            D = max(-1,min(1,D));
            th2(i) = atan2(-sqrt(1-D^2), D);
            th1(i) = atan2((y(i)-a3*sin(theta_d)), (x(i)-a3*cos(theta_d))) - atan2(a2*sin(th2(i)), a1+a2*cos(th2(i)));
            th3(i) = 3/2*pi - (th1(i)+th2(i));

            if i == Ns
                step3_init_x = x(i);
                step3_init_y = y(i);
            end

        elseif count == 3
            % 대각선 이동
            rootTerm = sqrt(a2^2 - (a1-a3)^2);
            x(i) = step3_init_x - ( step3_init_x - rootTerm ) * 0.5*(1 - cos(pi/step_3_time * t));
            y(i) = ( step3_init_y / (step3_init_x - rootTerm) ) * ( x(i) - rootTerm );

            theta_d = 3*pi/2;
            D = ((x(i)-a3*cos(theta_d))^2 + (y(i)-a3*sin(theta_d))^2 - a1^2 - a2^2)/(2*a1*a2);
            D = max(-1,min(1,D));
            th2(i) = atan2(-sqrt(1-D^2), D);
            th1(i) = atan2((y(i)-a3*sin(theta_d)), (x(i)-a3*cos(theta_d))) - atan2(a2*sin(th2(i)), a1+a2*cos(th2(i)));
            th3(i) = 3/2*pi - (th1(i)+th2(i));

            if i == Ns
                step4_init_x = x(i);
                step4_init_y = y(i);
                step4_init_theta_1 = th1(i);
                step4_init_theta_2 = th2(i);
                step4_init_theta_3 = th3(i);
            end

        elseif count == 4
            % th2만 +170deg 회전 (FK로 x,y)
            th1(i) = step4_init_theta_1;
            th2(i) = step4_init_theta_2 + deg2rad(170)*0.5*(1 - cos(pi/step_4_time * t));
            th3(i) = step4_init_theta_3;

            Ax = [0, a1*cos(th1(i)), a1*cos(th1(i))+a2*cos(th1(i)+th2(i)), ...
                  a1*cos(th1(i))+a2*cos(th1(i)+th2(i))+a3*cos(th1(i)+th2(i)+th3(i))];
            Ay = [0, a1*sin(th1(i)), a1*sin(th1(i))+a2*sin(th1(i)+th2(i)), ...
                  a1*sin(th1(i))+a2*sin(th1(i)+th2(i))+a3*sin(th1(i)+th2(i)+th3(i))];
            x(i) = Ax(4);
            y(i) = Ay(4);

            if i == Ns
                step5_init_theta_1 = th1(i);
                step5_init_theta_2 = th2(i);
                step5_init_theta_3 = th3(i);
            end

        elseif count == 5
            % th2만 -170deg 복귀 (FK로 x,y)
            th1(i) = step5_init_theta_1;
            th2(i) = step5_init_theta_2 - deg2rad(170)*0.5*(1 - cos(pi/step_5_time * t));
            th3(i) = step5_init_theta_3;

            Ax = [0, a1*cos(th1(i)), a1*cos(th1(i))+a2*cos(th1(i)+th2(i)), ...
                  a1*cos(th1(i))+a2*cos(th1(i)+th2(i))+a3*cos(th1(i)+th2(i)+th3(i))];
            Ay = [0, a1*sin(th1(i)), a1*sin(th1(i))+a2*sin(th1(i)+th2(i)), ...
                  a1*sin(th1(i))+a2*sin(th1(i)+th2(i))+a3*sin(th1(i)+th2(i)+th3(i))];
            x(i) = Ax(4);
            y(i) = Ay(4);

        elseif count == 6
            % 대각선 원위치
            rootTerm = sqrt(a2^2 - (a1-a3)^2);
            x(i) = step4_init_x + ( step3_init_x - rootTerm ) * 0.5*(1 - cos(pi/step_6_time * t));
            y(i) = ( step3_init_y / (step3_init_x - rootTerm) ) * ( x(i) - rootTerm );

            theta_d = 3*pi/2;
            D = ((x(i)-a3*cos(theta_d))^2 + (y(i)-a3*sin(theta_d))^2 - a1^2 - a2^2)/(2*a1*a2);
            D = max(-1,min(1,D));
            th2(i) = atan2(-sqrt(1-D^2), D);
            th1(i) = atan2((y(i)-a3*sin(theta_d)), (x(i)-a3*cos(theta_d))) - atan2(a2*sin(th2(i)), a1+a2*cos(th2(i)));
            th3(i) = 3/2*pi - (th1(i)+th2(i));
        end

        % ===== 애니메이션 (현재 자세로 FK 점들) =====
        Ax = [0, a1*cos(th1(i)), a1*cos(th1(i))+a2*cos(th1(i)+th2(i)), ...
              a1*cos(th1(i))+a2*cos(th1(i)+th2(i))+a3*cos(th1(i)+th2(i)+th3(i))];
        Ay = [0, a1*sin(th1(i)), a1*sin(th1(i))+a2*sin(th1(i)+th2(i)), ...
              a1*sin(th1(i))+a2*sin(th1(i)+th2(i))+a3*sin(th1(i)+th2(i)+th3(i))];
        set(p,'XData',Ax,'YData',Ay); drawnow;

        % ===== 속도/가속 (현재 인덱스에 저장)
        if i >= 2
            th1dot(i) = (th1(i) - th1(i-1))/del_t;
            th2dot(i) = (th2(i) - th2(i-1))/del_t;
            th3dot(i) = (th3(i) - th3(i-1))/del_t;
        else
            th1dot(i) = 0; th2dot(i) = 0; th3dot(i) = 0;
        end
        if i >= 3
            th1ddot(i) = (th1dot(i) - th1dot(i-1))/del_t;
            th2ddot(i) = (th2dot(i) - th2dot(i-1))/del_t;
            th3ddot(i) = (th3dot(i) - th3dot(i-1))/del_t;
        else
            th1ddot(i) = 0; th2ddot(i) = 0; th3ddot(i) = 0;
        end

        % ===== 동역학 (M,C,G) =====
        C1 = cos(th1(i)); S1 = sin(th1(i));
        C2 = cos(th2(i)); S2 = sin(th2(i));
        C3 = cos(th3(i)); S3 = sin(th3(i));
        C12 = cos(th1(i)+th2(i)); S12 = sin(th1(i)+th2(i));
        C23 = cos(th2(i)+th3(i)); S23 = sin(th2(i)+th3(i));
        C123 = cos(th1(i)+th2(i)+th3(i));

        % Inertia Matrix
        M11 = m1*lc1^2 + I1 + m2*a1^2 + m2*lc2^2 + 2*m2*a1*lc2*C2 + I2 + ...
              m3*a1^2 + m3*a2^2 + m3*lc3^2 + 2*m3*a1*a2*C2 + 2*m3*a2*lc3*C3 + 2*m3*a1*lc3*C23 + I3;
        M12 = m2*lc2^2 + m2*a1*lc2*C2 + I2 + m3*a2^2 + m3*lc3^2 + m3*a1*a2*C2 + 2*m3*a2*lc3*C3 + m3*a1*lc3*C23 + I3;
        M13 = m3*lc3^2 + m3*a2*lc3*C3 + m3*a1*lc3*C23 + I3;
        M21 = m2*lc2^2 + m2*a1*lc2*C2 + I2 + m3*a2^2 + m3*lc3^2 + m3*a1*a2*C2 + 2*m3*a2*lc3*C3 + m3*a1*lc3*C23 + I3;
        M22 = m2*lc2^2 + I2 + m3*a2^2 + m3*lc3^2 + 2*m3*a2*lc3*C3 + I3;
        M23 = m3*lc3^2 + m3*a2*lc3*C3 + I3;
        M31 = m3*lc3^2 + m3*a2*lc3*C3 + m3*a1*lc3*C23 + I3;
        M32 = m3*lc3^2 + m3*a2*lc3*C3 + I3;
        M33 = m3*lc3^2 + I3;
        M = [M11 M12 M13; M21 M22 M23; M31 M32 M33];

        % Centrifugal & Coriolis (A * dot terms)
        A112 = -2*m2*a1*lc2*S2 - 2*m3*a1*a2*S2 - 2*m3*a1*lc3*S23;
        A123 = -2*m3*a2*lc3*S3 - 2*m3*a1*lc3*S23;
        A113 = -2*m3*a2*lc3*S3 - 2*m3*a1*lc3*S23;
        A122 = -m2*a1*lc3*S3 - m3*a1*a2*S2 - m3*a1*lc3*S23;
        A133 = -m3*a1*lc3*S23 - m3*a2*lc3*S3;
        A211 =  m2*a1*lc2*S2 + m3*a1*a2*S2 + m3*a1*lc3*S23;
        A233 = -m3*a2*lc3*S3;
        A212 = 0;
        A223 = -2*m3*a2*lc3*S3;
        A213 = -2*m3*a2*lc3*S3;
        A311 =  m3*a2*lc3*S3 + m3*a1*lc3*S23;
        A322 =  m3*a2*lc3*S3;
        A312 =  2*m3*a2*lc3*S3;
        A323 =  0;
        A313 =  0;

        % 속도 항(현재 시점)
        th1dot_i = th1dot(i); th2dot_i = th2dot(i); th3dot_i = th3dot(i);
        dot12 = th1dot_i*th2dot_i;
        dot13 = th1dot_i*th3dot_i;
        dot23 = th2dot_i*th3dot_i;
        dot1sq = th1dot_i^2; dot2sq = th2dot_i^2; dot3sq = th3dot_i^2;

        Cvec = [A112*dot12 + A123*dot23 + A113*dot13 + A133*dot3sq + A122*dot2sq;
                A211*dot1sq + A233*dot3sq + A212*dot12 + A223*dot23 + A213*dot13;
                A311*dot1sq + A322*dot2sq + A312*dot12 + A323*dot23 + A313*dot13];

        % Gravity
        G1 = m1*g*lc1*cos(th1(i)) + m2*g*(a1*cos(th1(i)) + lc2*cos(th1(i)+th2(i))) + m3*g*(a1*cos(th1(i)) + a2*cos(th1(i)+th2(i)) + lc3*cos(th1(i)+th2(i)+th3(i)));
        G2 = m2*g*lc2*cos(th1(i)+th2(i)) + m3*g*(a2*cos(th1(i)+th2(i)) + lc3*cos(th1(i)+th2(i)+th3(i)));
        G3 = m3*g*lc3*cos(th1(i)+th2(i)+th3(i));
        Gvec = [G1; G2; G3];

        % 토크 / 파워
        tau = M * [th1ddot(i); th2ddot(i); th3ddot(i)] + Cvec + Gvec;
        pow = abs([th1dot_i; th2dot_i; th3dot_i] .* tau);

        % 누적
        T(:, end+1) = tau;
        P(:, end+1) = pow;
    end % for i = 1:Ns

    % ===== 스텝 종료: 구조체 저장 =====
    s = count;
    Step(s).t      = linspace(0, step_time, Ns);
    Step(s).x      = x;
    Step(s).y      = y;
    Step(s).th1    = th1;
    Step(s).th2    = th2;
    Step(s).th3    = th3;
    Step(s).th1dot = th1dot;
    Step(s).th2dot = th2dot;
    Step(s).th3dot = th3dot;

    idx1 = size(T,2);                % 이번 스텝 종료 인덱스
    TPslice = idx0:idx1;             % 이 스텝에 해당하는 토크/파워 구간
    Step(s).T   = T(:, TPslice);
    Step(s).P   = P(:, TPslice);
    Step(s).t_TP = linspace(0, step_time, numel(TPslice));

    count = count + 1;
end % for step = 1:6

%% 요약 플롯 (한 번만 생성, 탭 6개)
uif = uifigure('Name','Summary (All Steps)', 'Position',[100 100 1200 800]);
tg  = uitabgroup(uif, 'Units','normalized', 'Position',[0 0 1 1]);

names = {'Step 1','Step 2','Step 3','Step 4','Step 5','Step 6'};
for s = 1:6
    ttab = uitab(tg,'Title',names{s});
    gl = uigridlayout(ttab,[2,2]);
    gl.RowHeight   = {'1x','1x'};
    gl.ColumnWidth = {'1x','1x'};

    % Path (x-y)
    ax1 = uiaxes(gl); title(ax1,'Path'); xlabel(ax1,'x (m)'); ylabel(ax1,'y (m)'); grid(ax1,'on'); axis(ax1,'equal');
    plot(ax1, Step(s).x, Step(s).y, 'LineWidth',1.5);

    % Trajectory (x,y vs time)
    ax2 = uiaxes(gl); title(ax2,'Trajectory'); xlabel(ax2,'time (s)'); ylabel(ax2,'disp (m)'); grid(ax2,'on');
    plot(ax2, Step(s).t, Step(s).x, 'LineWidth',1.2); hold(ax2,'on');
    plot(ax2, Step(s).t, Step(s).y, 'LineWidth',1.2); legend(ax2,{'x','y'},'Location','best');

    % Joint angles (deg)
    ax3 = uiaxes(gl); title(ax3,'Joint Angles'); xlabel(ax3,'time (s)'); ylabel(ax3,'angle (deg)'); grid(ax3,'on');
    plot(ax3, Step(s).t, rad2deg(Step(s).th1), 'LineWidth',1.2); hold(ax3,'on');
    plot(ax3, Step(s).t, rad2deg(Step(s).th2), 'LineWidth',1.2);
    plot(ax3, Step(s).t, rad2deg(Step(s).th3), 'LineWidth',1.2);
    legend(ax3,{'θ1','θ2','θ3'},'Location','best');

    % Torque
    ax4 = uiaxes(gl); title(ax4,'Torque'); xlabel(ax4,'time (s)'); ylabel(ax4,'T (Nm)'); grid(ax4,'on');
    plot(ax4, Step(s).t_TP, Step(s).T(1,:), 'LineWidth',1.1); hold(ax4,'on');
    plot(ax4, Step(s).t_TP, Step(s).T(2,:), 'LineWidth',1.1);
    plot(ax4, Step(s).t_TP, Step(s).T(3,:), 'LineWidth',1.1);
    legend(ax4,{'J1','J2','J3'},'Location','best');
end

disp('완료: 애니메이션(실시간) + 요약 플롯(마지막 1회) 생성됨.');
