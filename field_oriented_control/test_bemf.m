close all;

ITERATION_TIMES = 10000;

bldc = bldc_dynamics;
bldc = bldc.init(0.001);

theta_r = zeros(1, ITERATION_TIMES);
fa = zeros(1, ITERATION_TIMES);
fb = zeros(1, ITERATION_TIMES);
fc = zeros(1, ITERATION_TIMES);

%forward clarke transform
V_alpha = zeros(1, ITERATION_TIMES);
V_beta = zeros(1, ITERATION_TIMES);
V_gamma = zeros(1, ITERATION_TIMES);

%forward park transform
V_d = zeros(1, ITERATION_TIMES);
V_q = zeros(1, ITERATION_TIMES);
V_z = zeros(1, ITERATION_TIMES);

%inverse park transform
V_alpha_inv = zeros(1, ITERATION_TIMES);
V_beta_inv = zeros(1, ITERATION_TIMES);
V_gamma_inv = zeros(1, ITERATION_TIMES);

%inverse clarke transform
V_a_inv = zeros(1, ITERATION_TIMES);
V_b_inv = zeros(1, ITERATION_TIMES);
V_c_inv = zeros(1, ITERATION_TIMES);

for i = 1: ITERATION_TIMES
    %simulate rotor angle change
    theta_r(i) = (i - 1) * ((2 * pi) / ITERATION_TIMES);
    
    %generate back EMF wave
    fa(i) = bldc.back_emf_fa(theta_r(i));
    fb(i) = bldc.back_emf_fb(theta_r(i));
    fc(i) = bldc.back_emf_fc(theta_r(i));
    
    V_abc = [fa(i); fb(i); fc(i)];
    
    %forward clark transform
    V_alpha_beta_gamma = bldc.clarke_transform(V_abc);
    
    %forward park transform
    V_dqz = bldc.park_transform(V_alpha_beta_gamma, theta_r(i));
      
    %inverse park transform
    V_alpha_beta_gamma_inv = bldc.inv_park_transform(V_dqz, theta_r(i));
    
    %inverse clarke transform
    V_abc_inv = bldc.inv_clarke_transform(V_alpha_beta_gamma_inv);
    
    %verify result (the difference should be very small)
    norm(V_alpha_beta_gamma_inv - V_alpha_beta_gamma)
    norm(V_abc_inv - V_abc)
    
    %update plot data
    V_alpha(i) = V_alpha_beta_gamma(1);
    V_beta(i) = V_alpha_beta_gamma(2);
    V_gamma(i) = V_alpha_beta_gamma(3);
    %
    V_d(i) = V_dqz(1);
    V_q(i) = V_dqz(2);
    V_z(i) = V_dqz(3);
    %
    V_alpha_inv(i) = V_alpha_beta_gamma_inv(1);
    V_beta_inv(i) = V_alpha_beta_gamma_inv(2);
    V_gamma_inv(i) = V_alpha_beta_gamma_inv(3);
    %
    V_a_inv(i) = V_abc_inv(1);
    V_b_inv(i) = V_abc_inv(2);
    V_c_inv(i) = V_abc_inv(3);
end

figure('Name', 'Back EMF');
subplot (3, 1, 1);
plot(rad2deg(theta_r(:)), fa(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('f_a');
subplot (3, 1, 2);
plot(rad2deg(theta_r(:)), fb(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('f_b');
subplot (3, 1, 3);
plot(rad2deg(theta_r(:)), fc(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('f_c');

%clarke transformation
figure('Name', 'Clarke Transformation');
subplot (3, 1, 1);
plot(rad2deg(theta_r(:)), V_alpha(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('V_{\alpha}');
subplot (3, 1, 2);
plot(rad2deg(theta_r(:)), V_beta(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('V_{\beta}');
subplot (3, 1, 3);
plot(rad2deg(theta_r(:)), V_gamma(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('V_{\gamma}');

%park transformation
figure('Name', 'Park Transformation');
subplot (3, 1, 1);
plot(rad2deg(theta_r(:)), V_d(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('V_d');
subplot (3, 1, 2);
plot(rad2deg(theta_r(:)), V_q(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('V_q');
subplot (3, 1, 3);
plot(rad2deg(theta_r(:)), V_z(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('V_z');

%clarke transformation
figure('Name', 'Inverse Park Transformation');
subplot (3, 1, 1);
plot(rad2deg(theta_r(:)), V_alpha_inv(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('V_{\alpha}');
subplot (3, 1, 2);
plot(rad2deg(theta_r(:)), V_beta_inv(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('V_{\beta}');
subplot (3, 1, 3);
plot(rad2deg(theta_r(:)), V_gamma_inv(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('V_{\gamma}');

figure('Name', 'Inverse Clarke Transform');
subplot (3, 1, 1);
plot(rad2deg(theta_r(:)), V_a_inv(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('f_a');
subplot (3, 1, 2);
plot(rad2deg(theta_r(:)), V_b_inv(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('f_b');
subplot (3, 1, 3);
plot(rad2deg(theta_r(:)), V_c_inv(:));
xlim([0 rad2deg(theta_r(end))]);
ylim([-1.3 1.3]);
xlabel('theta_r');
ylabel('f_c');