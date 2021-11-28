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

%hysteresis control parameters
i_d = 2.5;      %desited motor speed
delta_i = 0.01; %hysteresis band
i_a_d = 0; %desired i_a current
i_b_d = 0; %desired i_b current
i_c_d = 0; %desired i_c current
%
S1 = 0;
S2 = 0;
S3 = 0;
S4 = 0;
S5 = 0;
S6 = 0;

for i = 1: ITERATION_TIMES
    bldc = bldc.update();

    bldc.u(4) = 0; %no external torque
        
    %==================%
    % PI speed control %
    %==================%
    
    %=======================%
    % 6-steps phase control %
    %=======================%
    if(bldc.x(5) >= deg2rad(0) && bldc.x(5) < deg2rad(60))
        i_a_d = +i_d;
        i_b_d = -i_d;
        i_c_d = 0;
    elseif(bldc.x(5) >= deg2rad(60) && bldc.x(5) < deg2rad(120))
        i_a_d = +i_d;
        i_b_d = 0;
        i_c_d = -i_d;
    elseif(bldc.x(5) >= deg2rad(120) && bldc.x(5) < deg2rad(180))
        i_a_d = 0;
        i_b_d = +i_d;
        i_c_d = -i_d;
    elseif(bldc.x(5) >= deg2rad(180) && bldc.x(5) < deg2rad(240))
        i_a_d = -i_d;
        i_b_d = +i_d;
        i_c_d = 0;
    elseif(bldc.x(5) >= deg2rad(240) && bldc.x(5) < deg2rad(300))
        i_a_d = -i_d;
        i_b_d = 0;
        i_c_d = +i_d;
    elseif(bldc.x(5) >= deg2rad(300) && bldc.x(5) < deg2rad(360))
        i_a_d = 0;
        i_b_d = -i_d;
        i_c_d = +i_d;
    end

    %============================%
    % hysteresis current control %
    %============================%
       
    %phase a control
    if i_a_d >= 0
        if bldc.x(1) <= (i_a_d - delta_i)
            S1 = 1;
            S2 = 0;
        elseif bldc.x(1) >= (i_a_d + delta_i)
            S1 = 0;
            S2 = 1;
        end
    else
        if bldc.x(1) >= (i_a_d + delta_i)
            S1 = 0;
            S2 = 1;
        elseif bldc.x(1) <= (i_a_d - delta_i)
            S1 = 1;
            S2 = 0;
        end
    end
    
    %phase b control
    if i_b_d >= 0
        if bldc.x(2) <= (i_b_d - delta_i)
            S3 = 1;
            S4 = 0;
        elseif bldc.x(2) >= (i_b_d + delta_i)
            S3 = 0;
            S4 = 1;
        end
    else
        if bldc.x(2) >= (i_b_d + delta_i)
            S3 = 0;
            S4 = 1;
        elseif bldc.x(2) <= (i_b_d - delta_i)
            S3 = 1;
            S4 = 0;
        end
    end
 
    %phase c control
    if i_c_d >= 0
        if bldc.x(3) <= (i_c_d - delta_i)
            S5 = 1;
            S6 = 0;
        elseif bldc.x(3) >= (i_c_d + delta_i)
            S5 = 0;
            S6 = 1;
        end
    else
        if bldc.x(3) >= (i_c_d + delta_i)
            S5 = 0;
            S6 = 1;
        elseif bldc.x(3) <= (i_c_d - delta_i)
            S5 = 1;
            S6 = 0;
        end
    end
    
    if(abs(i_a_d) < 0.001)
        S1 = 0;
        S2 = 0;
    end
    
    if(abs(i_b_d) < 0.001)
        S3 = 0;
        S4 = 0;
    end
    
    if(abs(i_c_d) < 0.001)
        S5 = 0;
        S6 = 0;
    end
    
    bldc = bldc.set_mosfet_gate(S1, S2, S3, S4, S5, S6);
    
    %disp([S1 S2 S3 S4 S5 S6])
    
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
ylim([-5 5]);
xlabel('time [s]');
ylabel('i_a');
subplot (3, 1, 2);
plot(time_arr(:), i_b(:));
xlim([0 time_arr(end)]);
ylim([-5 5]);
xlabel('time [s]');
ylabel('i_b');
subplot (3, 1, 3);
plot(time_arr(:), i_c(:));
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

figure('Name', 'omega_m');
plot(time_arr(:), omega_m(:));
xlim([0 time_arr(end)]);
ylim([-20 50]);
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