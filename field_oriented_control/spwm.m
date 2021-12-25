close all;

dt = 0.001;
ITERATION_TIMES = 100000;

%triangle carrier wave
carrier = zeros(1, ITERATION_TIMES);
carrier_freq = 10;
carrier_cnt = 0;
amplitude = 1;
carrier_inc = amplitude / (1/dt) * 2 * carrier_freq
sign = 1;

%time sequence
time_arr = zeros(1, ITERATION_TIMES);

%modulation wave
sin_wave = zeros(1, ITERATION_TIMES);

%pwm wave
pwm = zeros(1, ITERATION_TIMES);

%lpf of pwm
lpf_pwm = zeros(1, ITERATION_TIMES);

for i = 1: ITERATION_TIMES
    t = (i - 1) * dt;
    
    %modulation wave = abs(sin(t))
    sin_wave(i) = abs(sin(t));
    
    if(abs(carrier_cnt - amplitude) < 1e-7) %carrier == amplitude
        sign = -1;
    elseif (abs(carrier_cnt) < 1e-7) %carrier == 0
        sign = 1;
    end
    
    %carrier wave = triangle wave
    carrier_cnt = carrier_cnt + (sign * carrier_inc);
    carrier(i) = carrier_cnt;
    
    %SPWM
    if(carrier_cnt >= sin_wave(i))
        pwm(i) = 0;
    else
        pwm(i) = 1;
    end
    
    %low pass filtering the PWM signal
    alpha = 0.01; %percentage of using new sampling
    if(i > 1)
        lpf_pwm(i) = (lpf_pwm(i - 1) * (1 - alpha)) + (pwm(i) * alpha);
    end
    
    time_arr(i) = t;
end

figure();
plot(time_arr(:), carrier(:), time_arr(:), sin_wave(:));
xlim([0 time_arr(end)]);
ylim([-0.2 1.2]);
xlabel('time [s]');
ylabel('voltage');
legend('carrier (triangle wave)', 'modulation (sine wave)')

figure();
plot(time_arr(:), pwm(:), time_arr(:), lpf_pwm(:));
xlim([0 time_arr(end)]);
ylim([-0.2 1.2]);
xlabel('time [s]');
ylabel('voltage');
legend('PWM signal', 'LPF(PWM signal)')

figure();
plot(time_arr(:), sin_wave(:), time_arr(:), lpf_pwm(:));
xlim([0 time_arr(end)]);
ylim([-0.2 1.2]);
xlabel('time [s]');
ylabel('voltage');
legend('desired signal', 'modulated signal')

