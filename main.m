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
e_a = zeros(1, ITERATION_TIMES);
e_b = zeros(1, ITERATION_TIMES);
e_c = zeros(1, ITERATION_TIMES);

v_a = zeros(1, ITERATION_TIMES);
v_b = zeros(1, ITERATION_TIMES);
v_c = zeros(1, ITERATION_TIMES);

%time sequence
time_arr = zeros(1, ITERATION_TIMES);

for i = 1: ITERATION_TIMES
    bldc = bldc.update();

    v_ctrl = 100; %control voltage
    bldc.u(4) = 0; %no external torque
    
    %bldc speed control
    if(bldc.x(5) >= deg2rad(0) && bldc.x(5) < deg2rad(60))
        bldc = bldc.set_mosfet_gate(1, 0, 0, 1, 0, 0);
    elseif(bldc.x(5) >= deg2rad(60) && bldc.x(5) < deg2rad(120))
        bldc = bldc.set_mosfet_gate(1, 0, 0, 0, 0, 1);
    elseif(bldc.x(5) >= deg2rad(120) && bldc.x(5) < deg2rad(180))
        bldc = bldc.set_mosfet_gate(0, 0, 1, 0, 0, 1);
    elseif(bldc.x(5) >= deg2rad(180) && bldc.x(5) < deg2rad(240))
        bldc = bldc.set_mosfet_gate(0, 1, 1, 0, 0, 0);
    elseif(bldc.x(5) >= deg2rad(240) && bldc.x(5) < deg2rad(300))
        bldc = bldc.set_mosfet_gate(0, 1, 0, 0, 1, 0);
    elseif(bldc.x(5) >= deg2rad(300) && bldc.x(5) < deg2rad(360))
        bldc = bldc.set_mosfet_gate(0, 0, 0, 1, 1, 0);
    end

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
    
    %bldc voltage
    v_a(i) = bldc.v(1);
    v_b(i) = bldc.v(2);
    v_c(i) = bldc.v(3);
    
    %time sequence
    time_arr(i) = (i - 1) * dt;
end

%3-phase back EMF
figure('Name', 'Current');
subplot (3, 1, 1);
plot(time_arr(:), i_a(:));
xlim([0 time_arr(end)]);
ylim([-1.3 1.3]);
xlabel('time [s]');
ylabel('i_a');
subplot (3, 1, 2);
plot(time_arr(:), i_b(:));
xlim([0 time_arr(end)]);
ylim([-1.3 1.3]);
xlabel('time [s]');
ylabel('i_b');
subplot (3, 1, 3);
plot(time_arr(:), i_c(:));
xlim([0 time_arr(end)]);
ylim([-1.3 1.3]);
xlabel('time [s]');
ylabel('i_c');

%control voltage
figure('Name', 'Control votage');
subplot (3, 1, 1);
plot(time_arr(:), v_a(:));
xlim([0 time_arr(end)]);
ylim([-15 15]);
xlabel('time [s]');
ylabel('v_a');
subplot (3, 1, 2);
plot(time_arr(:), v_b(:));
xlim([0 time_arr(end)]);
ylim([-15 15]);
xlabel('time [s]');
ylabel('v_b');
subplot (3, 1, 3);
plot(time_arr(:), v_c(:));
xlim([0 time_arr(end)]);
ylim([-15 15]);
xlabel('time [s]');
ylabel('v_c');

figure('Name', 'omega_m');
plot(time_arr(:), omega_m(:));
xlim([0 time_arr(end)]);
ylim([-20 20]);
xlabel('time [s]');
ylabel('motor speed');

figure('Name', 'theta_r');
plot(time_arr(:), rad2deg(theta_r(:)));
xlim([0 time_arr(end)]);
ylim([-20 380]);
xlabel('time [s]');
ylabel('motor position');

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
ylabel('e_a');
subplot (3, 1, 2);
plot(time_arr(:), f_b(:));
xlim([0 time_arr(end)]);
ylim([-1.3 1.3]);
xlabel('time [s]');
ylabel('e_b');
subplot (3, 1, 3);
plot(time_arr(:), f_c(:));
xlim([0 time_arr(end)]);
ylim([-1.3 1.3]);
xlabel('time [s]');
ylabel('e_c');

%3 phase voltage of the bldc
figure('Name', 'voltage');
subplot (3, 1, 1);
plot(time_arr(:), v_a(:));
xlim([0 time_arr(end)]);
ylim([-13 13]);
xlabel('time [s]');
ylabel('v_a');
subplot (3, 1, 2);
plot(time_arr(:), v_b(:));
xlim([0 time_arr(end)]);
ylim([-13 13]);
xlabel('time [s]');
ylabel('v_b');
subplot (3, 1, 3);
plot(time_arr(:), v_c(:));
xlim([0 time_arr(end)]);
ylim([-13 13]);
xlabel('time [s]');
ylabel('v_c');