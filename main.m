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

%time sequence
time_arr = zeros(1, ITERATION_TIMES);

for i = 1: ITERATION_TIMES
    bldc = bldc.update();

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

figure('Name', 'omega_m');
plot(time_arr(:), omega_m(:));
xlim([0 time_arr(end)]);
ylim([-20 20]);
xlabel('time [s]');
ylabel('motor speed');

figure('Name', 'theta_r');
plot(time_arr(:), rad2deg(theta_r(:)));
xlim([0 time_arr(end)]);
ylim([-200 200]);
xlabel('time [s]');
ylabel('motor position');

%3-phase back EMF
figure('Name', 'Back EMF');
subplot (3, 1, 1);
plot(time_arr(:), e_a(:));
xlim([0 time_arr(end)]);
ylim([-5 15]);
xlabel('time [s]');
ylabel('e_a');
subplot (3, 1, 2);
plot(time_arr(:), e_b(:));
xlim([0 time_arr(end)]);
ylim([-15 15]);
xlabel('time [s]');
ylabel('e_b');
subplot (3, 1, 3);
plot(time_arr(:), e_c(:));
xlim([0 time_arr(end)]);
ylim([-15 15]);
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