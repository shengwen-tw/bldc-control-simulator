classdef bldc_dynamics
    properties
        dt;               %time period of one iteration
        
        R = 0.7;          %resistance [ohm]
        L = 2.72;         %self-inductance [mH]
        M = 1.5;          %mutual-inductance [mH]
        lambda_m = 0.105; %flux linkage of the motor [wb]
        P = 4;            %number of the motor poles
        Jm = 0.000284;    %inertia of the motor [kg*m/s^2]
        Jl = 0;           %inertia of the load [kg*m/s^2]
        J = 0;            %J = Jm + Jl
        B_vis = 0.02;     %motor viscous friction constant [N*m/rad/s]
        Kt = 0.2;         %torque-current constant
        
        f_a = 0;          %back EMF of the phase A
        f_b = 0;          %back EMF of the phase B
        f_c = 0;          %back EMF of the phase C
        
        %matrices of the LTV (Linear Time Variant) system
        A = zeros(5, 5);  %state transition matrix
        B = zeros(5, 4);  %control matrix
        C = zeros(5, 3);  %back emf matrix
        
        %state vector = [i_a; i_b; i_c; omega_m; theta_r]
        x = [0;            %initial phase A current
             0;            %initial phase B current
             0;            %initial phase C current
             deg2rad(0);   %initial motor speed
             deg2rad(0)];  %initial motor position
        u = zeros(4, 1);   %control vector = [v_a; v_b; v_c; T_l]
        e = zeros(3, 1);   %back emf vector = [e_a; e_b; e_c]
        
        x_dot = zeros(5, 1);
        
        torque = 0;        %torque of the motor
        
        PWM_freq = 100     %frequency of the PWM for bldc control
        
        %BLDC control voltage
        v_bldc = 100;
    end
    
    methods
        function ret_obj = init(obj, dt)
            obj.dt = dt;
            
            obj.J = obj.Jm + obj.Jl;
            
            obj.A(1, 1) = -obj.R/(obj.L-obj.M);
            obj.A(2, 2) = -obj.R/(obj.L-obj.M);
            obj.A(3, 3) = -obj.R/(obj.L-obj.M);
            %
            %A(4, 1) = lambda_div_J * obj.f_a; %change dynamically
            %A(4, 2) = lambda_div_J * obj.f_b; %change dynamically
            %A(4, 3) = lambda_div_J * obj.f_c; %change dynamically
            obj.A(4, 4) = -obj.B_vis/obj.J;
            %
            obj.A(5, 4) = obj.P/2;
            
            obj.B(1, 1) = 1/(obj.L-obj.M);
            obj.B(2, 2) = 1/(obj.L-obj.M);
            obj.B(3, 3) = 1/(obj.L-obj.M);
            obj.B(4, 4) = 1;
             
            obj.C(1, 1) = -1/(obj.L-obj.M);
            obj.C(2, 2) = -1/(obj.L-obj.M);
            obj.C(3, 3) = -1/(obj.L-obj.M);
            
            ret_obj = obj;
        end
             
        function signal = synthesis_gate_signal(obj, S1, S2, S3, S4, S5, S6)
            signal = bitshift(S6, 5) + ...
                     bitshift(S5, 4) + ...
                     bitshift(S4, 3) + ...
                     bitshift(S3, 2) + ...
                     bitshift(S2, 1) + ...
                     bitshift(S1, 0);
        end
               
        function ret_obj = update_dynamics(obj)                        
            %A matrix is time variant, update the entry corresponding to
            %the back EMF
            lambda_div_J = obj.lambda_m / obj.J;
            obj.A(4, 1) = lambda_div_J * obj.f_a;
            obj.A(4, 2) = lambda_div_J * obj.f_b;
            obj.A(4, 3) = lambda_div_J * obj.f_c;
           
            %x_dot = Ax + Bu + Ce
            obj.x_dot = obj.A*obj.x + obj.B*obj.u + obj.C*obj.e;
            
            ret_obj = obj;
        end
        
        function f_next = integrator(obj, f_now, f_dot, dt)
            %eulers method
            f_next = [f_now(1) + (f_dot(1) * dt);
                      f_now(2) + (f_dot(2) * dt);
                      f_now(3) + (f_dot(3) * dt);
                      f_now(4) + (f_dot(4) * dt);
                      f_now(5) + (f_dot(5) * dt)];
        end
        
        function ret_obj = update(obj)
            %update the back EMF according to the motor angle
            obj.f_a = obj.back_emf_fa(obj.x(5));
            obj.f_b = obj.back_emf_fb(obj.x(5));
            obj.f_c = obj.back_emf_fc(obj.x(5));
            %            
            omega_times_lambda = obj.x(4) * obj.lambda_m;
            obj.e(1) = omega_times_lambda * obj.f_a;
            obj.e(2) = omega_times_lambda * obj.f_b;
            obj.e(3) = omega_times_lambda * obj.f_c;
            
            obj = obj.update_dynamics();
            obj.x = obj.integrator(obj.x, obj.x_dot, obj.dt);
                        
            %restrict motor position in +-pi
            obj.x(5) = mod(obj.x(5), 2*pi);
            
            %update torque of the motor
            sum_of_e_times_i = obj.e(1) * obj.x(1) +  obj.e(2) * obj.x(2) + obj.e(3) * obj.x(3);
            obj.torque = sum_of_e_times_i / obj.x(4);
            
            ret_obj = obj;
        end
        
        function retval = back_emf_fa(obj, theta_r)
            retval = sin(theta_r);
        end
        
        function retval = back_emf_fb(obj, theta_r)
            retval = sin(theta_r - deg2rad(120));
        end
        
        function retval = back_emf_fc(obj, theta_r)
            retval = sin(theta_r - deg2rad(240));
        end
        
        function V_abc = space_vector_to_three_phase_voltage(obj, x)
            V = obj.v_bldc;
            
            %Ux = (Sa, Sb, Sc)
            switch(x)
                case 0 %U0 = (0, 0, 0)
                    V_abc = [0; 0; 0];
                case 1 %U1 = (0, 0, 1)
                    V_abc = [+2/3*V; -1/3*V; -1/3*V];
                case 2 %U2 = (0, 1, 0)
                    V_abc = [-1/3*V; +2/3*V; -1/3*V];
                case 3 %U3 = (0, 1, 1)
                    V_abc = [+1/3*V; +1/3*V; -2/3*V];
                case 4 %U4 = (1, 0, 0)
                    V_abc = [-1/3*V; -1/3*V; +2/3*V];
                case 5 %U5 = (1, 0, 1)
                    V_abc = [+1/3*V; -2/3*V; +1/3*V];
                case 6 %U6 = (1, 1, 0)
                    V_abc = [-2/3*V; +1/3*V; +1/3*V];
                case 7 %U7 = (1, 1, 1)
                    V_abc = [0; 0; 0];
                otherwise
                    V_abc = [0; 0; 0];
                    warning('undefined space vector.');
            end
        end
        
        function periods = calculate_SVPWM_period(obj, theta)
            theta_deg = rad2deg(theta);
            
            %pwm period, -1 means currently not used
            T0 = -1;
            T1 = -1;
            T2 = -1;
            T3 = -1;
            T4 = -1;
            T5 = -1;
            T6 = -1;
            T7 = -1;
            T_off = 0;
            
            m = sqrt(3); %modulation index
            T = 1 / obj.PWM_freq;  %pwm frequency
            
            %calculate the section number from the rotation angle
            if(theta_deg >= 0 && theta_deg < 60)
                T4 = m * T * sin(pi/3 - theta);
                T6 = m * T * sin(theta);
                T_off = 1/2*(T - T4 - T6);
            elseif(theta_deg >= 60 && theta_deg < 120)
                T2 = m * T;
                T6 = m * T;
                T_off = 1/2*(T - T2 - T6);
            elseif(theta_deg >= 120 && theta_deg < 180)
                T2 = m * T;
                T3 = m * T;
                T_off = 1/2*(T - T2 - T3);
            elseif(theta_deg >= 180 && theta_deg < 240)
                T1 = m * T;
                T3 = m * T;
                T_off = 1/2*(T - T4 - T6);
            elseif(theta_deg >= 240 && theta_deg < 300)
                T1 = m * T;
                T5 = m * T;
                T_off = 1/2*(T - T4 - T6);
            elseif(theta_deg >= 300 && theta_deg < 360)
                T4 = m * T;
                T5 = m * T;
                T_off = 1/2*(T - T4 - T6);
            end
            
            T0 = T_off;
            T7 = T_off;
            periods = [T0; T1; T2; T3; T4; T5; T6; T7];
        end
        
        function generate_SVPWM_signal(obj)
            %output the three phase voltage according to the segment number
        end
        
        function alpha_beta_gemma = clarke_transform(obj, abc)
            T_c(1, 1) = 1;
            T_c(1, 2) = -1/2;
            T_c(1, 3) = -1/2;
            T_c(2, 1) = 0;
            T_c(2, 2) = sqrt(3)/2;
            T_c(2, 3) = -sqrt(3)/2;
            T_c(3, 1) = 1/2;
            T_c(3, 2) = 1/2;
            T_c(3, 3) = 1/2;
            
            alpha_beta_gemma = (2/3) * T_c * abc;
        end
        
        function abc = inv_clarke_transform(obj, alpha_beta_gemma)
            T_c_inv(1, 1) = 1;
            T_c_inv(1, 2) = 0;
            T_c_inv(1, 3) = 1;
            T_c_inv(2, 1) = -1/2;
            T_c_inv(2, 2) = sqrt(3)/2;
            T_c_inv(2, 3) = 1;
            T_c_inv(3, 1) = -1/2;
            T_c_inv(3, 2) = -sqrt(3)/2;
            T_c_inv(3, 3) = 1;
            
            abc = T_c_inv * alpha_beta_gemma;
        end
        
        function dqz = park_transform(obj, alpha_beta_gamma, theta)
            T_p(1, 1) = cos(theta);
            T_p(1, 2) = sin(theta);
            T_p(1, 3) = 0;
            T_p(2, 1) = -sin(theta);
            T_p(2, 2) = cos(theta);
            T_p(2, 3) = 0;
            T_p(3, 1) = 0;
            T_p(3, 2) = 0;
            T_p(3, 3) = 1;
            
            dqz = T_p * alpha_beta_gamma;
        end
        
        function alpha_beta_gamma = inv_park_transform(obj, dqz, theta)
            T_p_inv(1, 1) = cos(theta);
            T_p_inv(1, 2) = -sin(theta);
            T_p_inv(1, 3) = 0;
            T_p_inv(2, 1) = sin(theta);
            T_p_inv(2, 2) = cos(theta);
            T_p_inv(2, 3) = 0;
            T_p_inv(3, 1) = 0;
            T_p_inv(3, 2) = 0;
            T_p_inv(3, 3) = 1;
            
            dqz(3) = 0;
            alpha_beta_gamma = T_p_inv * dqz;
        end
    end
end