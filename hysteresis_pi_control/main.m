close all;

dt = 0.001;
ITERATION_TIMES = 10000;

bldc = bldc_dynamics;
bldc = bldc.init(dt);

%3-phase currents
i_a = zeros(1, ITERATION_TIMES);
i_b = zeros(1, ITERATION_TIMES);
i_c = zeros(1, ITERATION_TIMES);

%motor speed
omega_m = zeros(1, ITERATION_TIMES);

%motor position
theta_r = zeros(1, ITERATION_TIMES);

%3-phase back EMF
e_a = zeros(1, ITERATION_TIMES);
e_b = zeros(1, ITERATION_TIMES);
e_c = zeros(1, ITERATION_TIMES);

%normalized 3-phase back EMF
f_a = zeros(1, ITERATION_TIMES);
f_b = zeros(1, ITERATION_TIMES);
f_c = zeros(1, ITERATION_TIMES);

%3-phase control voltages
v_a = zeros(1, ITERATION_TIMES);
v_b = zeros(1, ITERATION_TIMES);
v_c = zeros(1, ITERATION_TIMES);

%time sequence
time_arr = zeros(1, ITERATION_TIMES);

%PI speed control
w_d = zeros(1, ITERATION_TIMES); %desited motor speed
T_d = zeros(1, ITERATION_TIMES); %desired torque
T_d_last = 0; %desired torque of last time interval
e_w_last = 0; %speed error of last time interval
Kp = 0.25;    %P gain
Ki = 0.0025;  %I gain

%ysteresis control parameters
delta_i = 0.001;                   %hysteresis band
i_d = zeros(1, ITERATION_TIMES);   %desited current
i_a_d = zeros(1, ITERATION_TIMES); %desired i_a current
i_b_d = zeros(1, ITERATION_TIMES); %desired i_b current
i_c_d = zeros(1, ITERATION_TIMES); %desired i_c current

%MOSFET signal
S1 = 0;
S2 = 0;
S3 = 0;
S4 = 0;
S5 = 0;
S6 = 0;

%rotor position sensing
phase_a = zeros(1, ITERATION_TIMES);
phase_b = zeros(1, ITERATION_TIMES);
phase_c = zeros(1, ITERATION_TIMES);
theta_sense = zeros(1, ITERATION_TIMES);

%motor torque
torque = zeros(1, ITERATION_TIMES);
w_m_last = 0;

for i = 1: ITERATION_TIMES
    bldc = bldc.update();

    bldc.u(4) = 0; %no external torque
       
    %===========================%
    % Speed trajectory planning %
    %===========================%
    %for debugging:
    %i_d(i) = i * dt * 0.25;
    %i_d(i) = 1 * sin(2 * i * dt);
    
    %fixed speed target
    if 0
        w_d(i) = 5;
    end
    
    %linear speed trajectory planning
    if 0
        traj_slope = 10;
        traj_x = i * dt;
        w_d(i) = traj_slope * traj_x; %[RPM]
    end
    
    %nonlinear speed trajectory planning
    if 1
        traj_coeff = 150;
        traj_speed = 5;
        w_d(i) = traj_coeff * abs(sin(traj_speed * i * dt));
    end
    
    %convert the speed unit from [RPM] to [rad/s]
    w_d(i) = w_d(i) * 0.10472;
    
    %=====================================================%
    % Rotor position sensing with back-EMF sensing method %
    %=====================================================%
    
    %phase a
    if bldc.e(1) >= 0
        phase_a(i) = 1;
    else
        phase_a(i) = 0;
    end
    
    %phase b
    if bldc.e(2) >= 0
        phase_b(i) = 1;
    else
        phase_b(i) = 0;
    end
    
    %phase c
    if bldc.e(3) >= 0
        phase_c(i) = 1;
    else
        phase_c(i) = 0;
    end
    
    %decode the phase signal
    if isequal([phase_a(i) phase_b(i) phase_c(i)], [1 0 1])
        theta_sense(i) = 0;
    elseif isequal([phase_a(i) phase_b(i) phase_c(i)], [1 0 0])
        theta_sense(i) = 60;
    elseif isequal([phase_a(i) phase_b(i) phase_c(i)], [1 1 0])
        theta_sense(i) = 120;
    elseif isequal([phase_a(i) phase_b(i) phase_c(i)], [0 1 0])
        theta_sense(i) = 180;
    elseif isequal([phase_a(i) phase_b(i) phase_c(i)], [0 1 1])
        theta_sense(i) = 240;
    elseif isequal([phase_a(i) phase_b(i) phase_c(i)], [0 0 1])
        theta_sense(i) = 300;
    end
    
    %==================%
    % PI speed control %
    %==================%
    %Incremental PI control
    e_w = w_d(i) - bldc.x(4);
    T_d(i) = T_d_last + (Kp * (e_w - e_w_last)) + (Ki * e_w);
    e_w_last = e_w;
    T_d_last = T_d(i);
    i_d(i) = T_d(i) / bldc.Kt;
           
    %=============================%
    % 6-steps phase control logic %
    %=============================%
    if(bldc.x(5) >= deg2rad(0) && bldc.x(5) < deg2rad(60))
        %disp('motor position between 0-60 degree')
        %disp([phase_a(i) phase_b(i) phase_c(i)])
        i_a_d(i) = +i_d(i);
        i_b_d(i) = -i_d(i);
        i_c_d(i) = 0;
    elseif(bldc.x(5) >= deg2rad(60) && bldc.x(5) < deg2rad(120))
        %disp('motor position between 60-120 degree')
        %disp([phase_a(i) phase_b(i) phase_c(i)])
        i_a_d(i) = +i_d(i);
        i_b_d(i) = 0;
        i_c_d(i) = -i_d(i);
    elseif(bldc.x(5) >= deg2rad(120) && bldc.x(5) < deg2rad(180))
        %disp('motor position between 120-180 degree')
        %disp([phase_a(i) phase_b(i) phase_c(i)])
        i_a_d(i) = 0;
        i_b_d(i) = +i_d(i);
        i_c_d(i) = -i_d(i);
    elseif(bldc.x(5) >= deg2rad(180) && bldc.x(5) < deg2rad(240))
        %disp('motor position between 180-240 degree')
        %disp([phase_a(i) phase_b(i) phase_c(i)])
        i_a_d(i) = -i_d(i);
        i_b_d(i) = +i_d(i);
        i_c_d(i) = 0;
    elseif(bldc.x(5) >= deg2rad(240) && bldc.x(5) < deg2rad(300))
        %disp('motor position between 240-300 degree')
        %disp([phase_a(i) phase_b(i) phase_c(i)])
        i_a_d(i) = -i_d(i);
        i_b_d(i) = 0;
        i_c_d(i) = +i_d(i);
    elseif(bldc.x(5) >= deg2rad(300) && bldc.x(5) < deg2rad(360))
        %disp('300-360')
        %disp([phase_a(i) phase_b(i) phase_c(i)])
        i_a_d(i) = 0;
        i_b_d(i) = -i_d(i);
        i_c_d(i) = +i_d(i);
    end

    %============================%
    % hysteresis current control %
    %============================%
       
    %phase a control
    if i_a_d >= 0
        if bldc.x(1) <= (i_a_d(i) - delta_i)
            S1 = 1;
            S2 = 0;
            bldc.u(1) = bldc.v_bldc / 2;
        elseif bldc.x(1) >= (i_a_d(i) + delta_i)
            S1 = 0;
            S2 = 1;
            bldc.u(1) = -bldc.v_bldc / 2;
        end
    else
        if bldc.x(1) >= (i_a_d(i) + delta_i)
            S1 = 0;
            S2 = 1;
            bldc.u(1) = -bldc.v_bldc / 2;
        elseif bldc.x(1) <= (i_a_d(i) - delta_i)
            S1 = 1;
            S2 = 0;
            bldc.u(1) = bldc.v_bldc / 2;
        end
    end
    
    %phase b control
    if i_b_d >= 0
        if bldc.x(2) <= (i_b_d(i) - delta_i)
            S3 = 1;
            S4 = 0;
            bldc.u(2) = bldc.v_bldc / 2;
        elseif bldc.x(2) >= (i_b_d(i) + delta_i)
            S3 = 0;
            S4 = 1;
            bldc.u(2) = -bldc.v_bldc / 2;
        end
    else
        if bldc.x(2) >= (i_b_d(i) + delta_i)
            S3 = 0;
            S4 = 1;
            bldc.u(2) = -bldc.v_bldc / 2;
        elseif bldc.x(2) <= (i_b_d(i) - delta_i)
            S3 = 1;
            S4 = 0;
            bldc.u(2) = bldc.v_bldc / 2;
        end
    end
 
    %phase c control
    if i_c_d >= 0
        if bldc.x(3) <= (i_c_d(i) - delta_i)
            S5 = 1;
            S6 = 0;
            bldc.u(3) = bldc.v_bldc / 2;
        elseif bldc.x(3) >= (i_c_d(i) + delta_i)
            S5 = 0;
            S6 = 1;
            bldc.u(3) = -bldc.v_bldc / 2;
        end
    else
        if bldc.x(3) >= (i_c_d(i) + delta_i)
            S5 = 0;
            S6 = 1;
            bldc.u(3) = -bldc.v_bldc / 2;
        elseif bldc.x(3) <= (i_c_d(i) - delta_i)
            S5 = 1;
            S6 = 0;
            bldc.u(3) = bldc.v_bldc / 2;
        end
    end
    
    if(abs(i_a_d(i)) < 0.001)
        S1 = 0;
        S2 = 0;
    end
    
    if(abs(i_b_d(i)) < 0.001)
        S3 = 0;
        S4 = 0;
    end
    
    if(abs(i_c_d(i)) < 0.001)
        S5 = 0;
        S6 = 0;
    end
    
    bldc = bldc.set_mosfet_gate(S1, S2, S3, S4, S5, S6);
       
    %currents of motor phases
    v_a(i) = bldc.u(1);
    v_b(i) = bldc.u(2);
    v_c(i) = bldc.u(3);
    
    %currents of motor phases
    i_a(i) = bldc.x(1);
    i_b(i) = bldc.x(2);
    i_c(i) = bldc.x(3);

    %motor speed
    omega_m(i) = bldc.x(4);
    %motor position
    theta_r(i) = bldc.x(5);
    
    %back EMF
    e_a(i) = bldc.e(1);
    e_b(i) = bldc.e(2);
    e_c(i) = bldc.e(3);
    
    %normalized back EMF
    f_a(i) = bldc.f_a;
    f_b(i) = bldc.f_b;
    f_c(i) = bldc.f_c;
    
    %motor torque
    torque(i) = bldc.torque;
    
    %time sequence
    time_arr(i) = (i - 1) * dt;
end

figure('Name', 'Desired current');
plot(time_arr(:), i_d(:));
xlim([0 time_arr(end)]);
ylim([-20 50]);
xlabel('time [s]');
ylabel('i_d');

%3-phase back EMF
figure('Name', '3 phase currents');
subplot (3, 1, 1);
plot(time_arr(:), i_a(:), time_arr(:), i_a_d(:));
legend('Actual current', 'Desired current');
xlim([0 time_arr(end)]);
ylim([-5 5]);
xlabel('time [s]');
ylabel('i_a');
subplot (3, 1, 2);
plot(time_arr(:), i_b(:), time_arr(:), i_b_d(:));
legend('Actual current', 'Desired current');
xlim([0 time_arr(end)]);
ylim([-5 5]);
xlabel('time [s]');
ylabel('i_b');
subplot (3, 1, 3);
plot(time_arr(:), i_c(:), time_arr(:), i_c_d(:));
legend('Actual current', 'Desired current');
xlim([0 time_arr(end)]);
ylim([-5 5]);
xlabel('time [s]');
ylabel('i_c');

%control voltage
figure('Name', 'Control votage');
subplot (3, 1, 1);
plot(time_arr(:), v_a(:));
xlim([0 time_arr(end)]);
ylim([-110 110]);
xlabel('time [s]');
ylabel('v_a');
subplot (3, 1, 2);
plot(time_arr(:), v_b(:));
xlim([0 time_arr(end)]);
ylim([-110 110]);
xlabel('time [s]');
ylabel('v_b');
subplot (3, 1, 3);
plot(time_arr(:), v_c(:));
xlim([0 time_arr(end)]);
ylim([-110 110]);
xlabel('time [s]');
ylabel('v_c');

figure('Name', 'Motor speed');
plot(time_arr(:), 9.5493 .* omega_m(:), time_arr(:), 9.5493 .* w_d(:));
legend('Actual speed', 'Desired speed');
xlim([0 time_arr(end)]);
ylim([-20 200]);
xlabel('time [s]');
ylabel('\omega_m [RPM]');

figure('Name', 'Motor position');
plot(time_arr(:), rad2deg(theta_r(:)), time_arr(:), theta_sense(:));
legend('Actual position', 'Estimated position');
xlim([0 time_arr(end)]);
ylim([-20 380]);
xlabel('time [s]');
ylabel('\theta_r');

%3-phase back EMF
figure('Name', 'Back EMF');
subplot (3, 1, 1);
plot(time_arr(:), e_a(:));
xlim([0 time_arr(end)]);
ylim([-5 5]);
xlabel('time [s]');
ylabel('e_a');
subplot (3, 1, 2);
plot(time_arr(:), e_b(:));
xlim([0 time_arr(end)]);
ylim([-5 5]);
xlabel('time [s]');
ylabel('e_b');
subplot (3, 1, 3);
plot(time_arr(:), e_c(:));
xlim([0 time_arr(end)]);
ylim([-5 5]);
xlabel('time [s]');
ylabel('e_c');

%3-phase normalized back EMF
figure('Name', 'Normalized back EMF');
subplot (3, 1, 1);
plot(time_arr(:), f_a(:));
xlim([0 time_arr(end)]);
ylim([-1.3 1.3]);
xlabel('time [s]');
ylabel('f_a');
subplot (3, 1, 2);
plot(time_arr(:), f_b(:));
xlim([0 time_arr(end)]);
ylim([-1.3 1.3]);
xlabel('time [s]');
ylabel('f_b');
subplot (3, 1, 3);
plot(time_arr(:), f_c(:));
xlim([0 time_arr(end)]);
ylim([-1.3 1.3]);
xlabel('time [s]');
ylabel('f_c');

%3-phase normalized back EMF
figure('Name', 'Normalized back EMF');
subplot (3, 1, 1);
plot(time_arr(:), phase_a(:));
xlim([0 time_arr(end)]);
ylim([-0.2 1.3]);
xlabel('time [s]');
ylabel('Phase A');
subplot (3, 1, 2);
plot(time_arr(:), phase_b(:));
xlim([0 time_arr(end)]);
ylim([-0.2 1.3]);
xlabel('time [s]');
ylabel('Phase B');
subplot (3, 1, 3);
plot(time_arr(:), phase_c(:));
xlim([0 time_arr(end)]);
ylim([-0.2 1.3]);
xlabel('time [s]');
ylabel('Phase C');

figure('Name', 'Motor torque');
plot(time_arr(:), torque(:), time_arr(:), T_d);
legend('Actual torque', 'Desired torque');
xlim([0 time_arr(end)]);
ylim([-0.2, 0.5]);
xlabel('time [s]');
ylabel('\tau [Nm]');