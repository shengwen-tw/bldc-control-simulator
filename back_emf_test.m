close all;

ITERATION_TIMES = 10000;

bldc = bldc_dynamics;
bldc = bldc.init(0.001);

theta_r = zeros(1, ITERATION_TIMES);
fa = zeros(1, ITERATION_TIMES);
fb = zeros(1, ITERATION_TIMES);
fc = zeros(1, ITERATION_TIMES);

for i = 1: ITERATION_TIMES
    theta_r(i) = (i - 1) * ((2 * pi) / ITERATION_TIMES);
    
    fa(i) = bldc.back_emf_fa(theta_r(i));
    fb(i) = bldc.back_emf_fb(theta_r(i));
    fc(i) = bldc.back_emf_fc(theta_r(i));
end

%position
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