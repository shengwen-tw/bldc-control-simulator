close all;

%===================%
% Simulator options %
%===================%

dt = 0.001;           %simulation period
SIMULATION_TIME = 10; %[s], total time of the simulation
ITERATION_TIMES = SIMULATION_TIME * (1/dt);

%==============================%
% Process model and controller %
%==============================%

bldc = bldc_dynamics;
bldc = bldc.init(dt);
bldc.u(4) = 0; %no external torque

%===========================%
%PI speed control paramters %
%===========================%

w_d = 0;          %desired motor speed
e_w = 0;          %error of the motor speed
T_d = 0;          %control output (torque) of the speed controller
T_d_arr_last = 0; %desired torque of last time interval
e_w_last = 0;     %speed error of last time interval
Kp = 0.15;        %Kp gain of the speed controller
Ki = 0.001;       %Ki gain of the speed controller

%===============================%
% hysteresis control parameters %
%===============================%

delta_I = 0.001; %hysteresis band of the current

%============%
% Plot datas %
%============%

%time sequence
time_arr = zeros(1, ITERATION_TIMES);

%3-phase currents
I_a = zeros(1, ITERATION_TIMES);
I_b = zeros(1, ITERATION_TIMES);
I_c = zeros(1, ITERATION_TIMES);

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
V_a = zeros(1, ITERATION_TIMES); %control voltage of phase A
V_b = zeros(1, ITERATION_TIMES); %control voltage of phase B
V_c = zeros(1, ITERATION_TIMES); %control voltage of phase C

w_d_arr = zeros(1, ITERATION_TIMES); %desited motor speed
T_d_arr = zeros(1, ITERATION_TIMES); %desired torque

I_d = zeros(1, ITERATION_TIMES);   %desited current
I_a_d = zeros(1, ITERATION_TIMES); %desired i_a current
I_b_d = zeros(1, ITERATION_TIMES); %desired i_b current
I_c_d = zeros(1, ITERATION_TIMES); %desired i_c current

%sensing of the rotor position sector
hall_a = zeros(1, ITERATION_TIMES); %hall signal of phase A
hall_b = zeros(1, ITERATION_TIMES); %hall signal of phase B
hall_c = zeros(1, ITERATION_TIMES); %hall signal of phase C
theta_sense = zeros(1, ITERATION_TIMES); %estimated rotor angle

%motor torque
torque = zeros(1, ITERATION_TIMES);

%======================%
% Simulation main loop %
%======================%

for i = 1: ITERATION_TIMES
    %===========================%
    % Speed trajectory planning %
    %===========================%

    %current control (speed open-loop)
    %i_d(i) = i * dt * 0.25;
    %i_d(i) = 1 * sin(2 * i * dt);
    
    %fixed speed target
    if 0
        w_d = 5;
    end
    
    %step impulse
    if 0
        time = i * dt;
        if time < (ITERATION_TIMES * dt / 2)
            w_d = 0;
        else
            w_d = 50;
        end
    end
    
    %linear speed trajectory planning
    if 0
        traj_slope = 10;
        traj_x = i * dt;
        w_d = traj_slope * traj_x; %[RPM]
    end
    
    %nonlinear speed trajectory planning
    if 1
        traj_coeff = 150;
        traj_speed = 5;
        w_d = traj_coeff * abs(sin(traj_speed * i * dt));
    end
    
    %convert the speed unit from [RPM] to [rad/s]
    w_d = w_d * 0.10472;
    
    %=====================================================%
    % Rotor position sensing with back EMF sensing method %
    %=====================================================%
    
    %Hall signal of phase A
    if bldc.e(1) >= 0
        hall_a(i) = 1;
    else
        hall_a(i) = 0;
    end
    
    %Hall signal of phase B
    if bldc.e(2) >= 0
        hall_b(i) = 1;
    else
        hall_b(i) = 0;
    end
    
    %Hall signal of phase C
    if bldc.e(3) >= 0
        hall_c(i) = 1;
    else
        hall_c(i) = 0;
    end
    
    %decode the hall signal
    if isequal([hall_a(i) hall_b(i) hall_c(i)], [1 0 1])
        theta_sense(i) = 0;
    elseif isequal([hall_a(i) hall_b(i) hall_c(i)], [1 0 0])
        theta_sense(i) = 60;
    elseif isequal([hall_a(i) hall_b(i) hall_c(i)], [1 1 0])
        theta_sense(i) = 120;
    elseif isequal([hall_a(i) hall_b(i) hall_c(i)], [0 1 0])
        theta_sense(i) = 180;
    elseif isequal([hall_a(i) hall_b(i) hall_c(i)], [0 1 1])
        theta_sense(i) = 240;
    elseif isequal([hall_a(i) hall_b(i) hall_c(i)], [0 0 1])
        theta_sense(i) = 300;
    end
    
    %==================%
    % PI speed control %
    %==================%
    
    %speed error
    e_w = w_d - bldc.x(4);
   
    %incremental PI control
    T_d = T_d_arr_last + (Kp * (e_w - e_w_last)) + (Ki * e_w);
    
    %save for next iteration
    e_w_last = e_w;
    T_d_arr_last = T_d;
    
    %convert the control output from torque to current
    I_d(i) = T_d / bldc.Kt;
           
    %=============================%
    % 6-steps phase control logic %
    %=============================%
    
    %calculate desired current for phase A, B and C
    if(bldc.x(5) >= deg2rad(0) && bldc.x(5) < deg2rad(60))
        %disp('motor position between 0-60 degree')
        %disp([hall_a(i) hall_b(i) hall_c(i)])
        I_a_d(i) = +I_d(i);
        I_b_d(i) = -I_d(i);
        I_c_d(i) = 0;
    elseif(bldc.x(5) >= deg2rad(60) && bldc.x(5) < deg2rad(120))
        %disp('motor position between 60-120 degree')
        %disp([hall_a(i) hall_b(i) hall_c(i)])
        I_a_d(i) = +I_d(i);
        I_b_d(i) = 0;
        I_c_d(i) = -I_d(i);
    elseif(bldc.x(5) >= deg2rad(120) && bldc.x(5) < deg2rad(180))
        %disp('motor position between 120-180 degree')
        %disp([hall_a(i) hall_b(i) hall_c(i)])
        I_a_d(i) = 0;
        I_b_d(i) = +I_d(i);
        I_c_d(i) = -I_d(i);
    elseif(bldc.x(5) >= deg2rad(180) && bldc.x(5) < deg2rad(240))
        %disp('motor position between 180-240 degree')
        %disp([hall_a(i) hall_b(i) hall_c(i)])
        I_a_d(i) = -I_d(i);
        I_b_d(i) = +I_d(i);
        I_c_d(i) = 0;
    elseif(bldc.x(5) >= deg2rad(240) && bldc.x(5) < deg2rad(300))
        %disp('motor position between 240-300 degree')
        %disp([hall_a(i) hall_b(i) hall_c(i)])
        I_a_d(i) = -I_d(i);
        I_b_d(i) = 0;
        I_c_d(i) = +I_d(i);
    elseif(bldc.x(5) >= deg2rad(300) && bldc.x(5) < deg2rad(360))
        %disp('300-360')
        %disp([hall_a(i) hall_b(i) hall_c(i)])
        I_a_d(i) = 0;
        I_b_d(i) = -I_d(i);
        I_c_d(i) = +I_d(i);
    end

    %============================%
    % hysteresis current control %
    %============================%
       
    %hysteresis control for phase A
    if I_a_d >= 0
        if bldc.x(1) <= (I_a_d(i) - delta_I)
            bldc.u(1) = bldc.v_bldc / 2;  %S1 = 1, S2 = 0
        elseif bldc.x(1) >= (I_a_d(i) + delta_I)
            bldc.u(1) = -bldc.v_bldc / 2; %S1 = 0, S2 = 1
        end
    else
        if bldc.x(1) >= (I_a_d(i) + delta_I)
            bldc.u(1) = -bldc.v_bldc / 2; %S1 = 0, S2 = 1
        elseif bldc.x(1) <= (I_a_d(i) - delta_I)
            bldc.u(1) = bldc.v_bldc / 2;  %S1 = 1, S2 = 0
        end
    end
    
    %hysteresis control for phase B
    if I_b_d >= 0
        if bldc.x(2) <= (I_b_d(i) - delta_I)
            bldc.u(2) = bldc.v_bldc / 2;  %S3 = 1, S4 = 0
        elseif bldc.x(2) >= (I_b_d(i) + delta_I)
            bldc.u(2) = -bldc.v_bldc / 2; %S3 = 0, S4 = 1
        end
    else
        if bldc.x(2) >= (I_b_d(i) + delta_I)
            bldc.u(2) = -bldc.v_bldc / 2; %S3 = 0, S4 = 1
        elseif bldc.x(2) <= (I_b_d(i) - delta_I)
            bldc.u(2) = bldc.v_bldc / 2;  %S3 = 1, S4 = 0
        end
    end
 
    %hysteresis control for phase C
    if I_c_d >= 0
        if bldc.x(3) <= (I_c_d(i) - delta_I)
            bldc.u(3) = bldc.v_bldc / 2;  %S5 = 1, S6 = 0
        elseif bldc.x(3) >= (I_c_d(i) + delta_I)
            bldc.u(3) = -bldc.v_bldc / 2; %S5 = 0, S6 = 1
        end
    else
        if bldc.x(3) >= (I_c_d(i) + delta_I)
            bldc.u(3) = -bldc.v_bldc / 2; %S5 = 0, S6 = 1;
        elseif bldc.x(3) <= (I_c_d(i) - delta_I)
            bldc.u(3) = bldc.v_bldc / 2; %S5 = 1, S6 = 0;
        end
    end
    
    %==========================%
    % update the BLDC dynamics %
    %==========================%
    bldc = bldc.update();
    
    %============%
    % Plot datas %
    %============%
    
    %time sequence
    time_arr(i) = (i - 1) * dt;
    
    %currents of motor phases
    V_a(i) = bldc.u(1);
    V_b(i) = bldc.u(2);
    V_c(i) = bldc.u(3);
    
    %state variables
    I_a(i) = bldc.x(1); %current of phase A
    I_b(i) = bldc.x(2); %current of phase B
    I_c(i) = bldc.x(3); %current of phase C
    omega_m(i) = bldc.x(4); %motor speed
    theta_r(i) = bldc.x(5); %rotor position
    
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
    
    %desired motor speed
    w_d_arr(i) = w_d;
    
    %control output (torque) of the speed controller
    T_d_arr(i) = T_d;
end

figure('Name', 'Desired current');
plot(time_arr(:), I_d(:));
xlim([0 time_arr(end)]);
ylim([-20 50]);
xlabel('time [s]');
ylabel('I_d');

%3-phase back EMF
figure('Name', '3 phase currents');
subplot (3, 1, 1);
plot(time_arr(:), I_a(:), time_arr(:), I_a_d(:));
legend('Actual current', 'Desired current');
xlim([0 time_arr(end)]);
ylim([-5 5]);
xlabel('time [s]');
ylabel('I_a');
subplot (3, 1, 2);
plot(time_arr(:), I_b(:), time_arr(:), I_b_d(:));
legend('Actual current', 'Desired current');
xlim([0 time_arr(end)]);
ylim([-5 5]);
xlabel('time [s]');
ylabel('I_b');
subplot (3, 1, 3);
plot(time_arr(:), I_c(:), time_arr(:), I_c_d(:));
legend('Actual current', 'Desired current');
xlim([0 time_arr(end)]);
ylim([-5 5]);
xlabel('time [s]');
ylabel('I_c');

%control voltage
figure('Name', 'Control votage');
subplot (3, 1, 1);
plot(time_arr(:), V_a(:));
xlim([0 time_arr(end)]);
ylim([-110 110]);
xlabel('time [s]');
ylabel('V_a');
subplot (3, 1, 2);
plot(time_arr(:), V_b(:));
xlim([0 time_arr(end)]);
ylim([-110 110]);
xlabel('time [s]');
ylabel('V_b');
subplot (3, 1, 3);
plot(time_arr(:), V_c(:));
xlim([0 time_arr(end)]);
ylim([-110 110]);
xlabel('time [s]');
ylabel('V_c');

figure('Name', 'Motor speed');
plot(time_arr(:), 9.5493 .* omega_m(:), time_arr(:), 9.5493 .* w_d_arr(:));
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
figure('Name', 'Hall signal');
subplot (3, 1, 1);
plot(time_arr(:), hall_a(:));
xlim([0 time_arr(end)]);
ylim([-0.2 1.3]);
xlabel('time [s]');
ylabel('Phase A');
subplot (3, 1, 2);
plot(time_arr(:), hall_b(:));
xlim([0 time_arr(end)]);
ylim([-0.2 1.3]);
xlabel('time [s]');
ylabel('Phase B');
subplot (3, 1, 3);
plot(time_arr(:), hall_c(:));
xlim([0 time_arr(end)]);
ylim([-0.2 1.3]);
xlabel('time [s]');
ylabel('Phase C');

figure('Name', 'Motor torque');
plot(time_arr(:), torque(:), time_arr(:), T_d_arr);
legend('Actual torque', 'Desired torque');
xlim([0 time_arr(end)]);
ylim([-0.2, 0.5]);
xlabel('time [s]');
ylabel('\tau [Nm]');