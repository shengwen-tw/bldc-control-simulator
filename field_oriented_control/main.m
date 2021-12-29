close all;

%=======================%
% Simulation parameters %
%=======================%

%simulation run time
dt = 0.001;           %[s]
simulation_time = 10; %[s]
ITERATION_TIMES = simulation_time / dt;

bldc_pwm_freq = 1000; %[hz]
bldc = bldc_dynamics;
bldc = bldc.init(bldc_pwm_freq);

%============%
% Plot datas %
%============%

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

%ysteresis control parameters
i_d = zeros(1, ITERATION_TIMES);   %desited current
i_a_d = zeros(1, ITERATION_TIMES); %desired i_a current
i_b_d = zeros(1, ITERATION_TIMES); %desired i_b current
i_c_d = zeros(1, ITERATION_TIMES); %desired i_c current

%rotor position sensing
phase_a = zeros(1, ITERATION_TIMES);
phase_b = zeros(1, ITERATION_TIMES);
phase_c = zeros(1, ITERATION_TIMES);
theta_sense = zeros(1, ITERATION_TIMES);

%motor torque
torque = zeros(1, ITERATION_TIMES);

%======================%
% Simulation main loop %
%======================%

SVPWM_state = 1;
bldc.T_SVPWM = 1/7 * [dt; dt; dt; dt; dt; dt; dt];

for i = 1: ITERATION_TIMES   
    %main loop has 7 procedures to handle 7-segment SVPWM
    switch(SVPWM_state)
        case 1
            %execute field-oriented control algorithm
            
            bldc.u(1:3) = bldc.u_SVPWM(1, 1:3).';
            dt = bldc.T_SVPWM(1);
        case 2
            bldc.u(1:3) = bldc.u_SVPWM(2, 1:3).';
            dt = bldc.T_SVPWM(2);
        case 3
            bldc.u(1:3) = bldc.u_SVPWM(3, 1:3).';
            dt = bldc.T_SVPWM(3);
        case 4
            bldc.u(1:3) = bldc.u_SVPWM(4, 1:3).';
            dt = bldc.T_SVPWM(4);
        case 5
            bldc.u(1:3) = bldc.u_SVPWM(5, 1:3).';
            dt = bldc.T_SVPWM(5);
        case 6
            bldc.u(1:3) = bldc.u_SVPWM(6, 1:3).';
            dt = bldc.T_SVPWM(6);
        case 7
            bldc.u(1:3) = bldc.u_SVPWM(7, 1:3).';
            dt = bldc.T_SVPWM(7);
    end
    
    SVPWM_state = SVPWM_state + 1;
    
    if(SVPWM_state > 7)
        SVPWM_state = 1;
    end
    
    %update the BLDC dynamics
    bldc = bldc.update(dt);
        
    %============%
    % Plot datas %
    %============%
    
    %time sequence
    time_arr(i) = (i - 1) * dt;
    
    %state variables
    i_a(i) = bldc.x(1); %current of phase A
    i_b(i) = bldc.x(2); %current of phase B
    i_c(i) = bldc.x(3); %current of phase C
    omega_m(i) = bldc.x(4); %motor speed
    theta_r(i) = bldc.x(5); %rotor position
    
    %control variables
    v_a(i) = bldc.u(1); %voltage of phase A
    v_b(i) = bldc.u(2); %voltage of phase B
    v_c(i) = bldc.u(3); %voltage of phase C
    
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