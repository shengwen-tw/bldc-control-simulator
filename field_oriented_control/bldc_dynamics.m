classdef bldc_dynamics
    properties       
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
                
        %BLDC control voltage
        v_bldc = 100;
        
        %SVPWM
        pwm_freq;              %frequency of the PWM for bldc control
        u_SVPWM = zeros(7, 3); %3 phase control voltage of the 7-segement SVPWM
        T_SVPWM = zeros(7, 1); %time period of the 7-segment SVPWM
    end
    
    methods
        function ret_obj = init(obj, pwm_freq)
            obj.pwm_freq = pwm_freq;
            
            %J
            obj.J = obj.Jm + obj.Jl;
            
            %A
            obj.A(1, 1) = -obj.R/(obj.L-obj.M);
            obj.A(2, 2) = -obj.R/(obj.L-obj.M);
            obj.A(3, 3) = -obj.R/(obj.L-obj.M);
            obj.A(4, 4) = -obj.B_vis/obj.J;
            obj.A(5, 4) = obj.P/2;
            
            %B
            obj.B(1, 1) = 1/(obj.L-obj.M);
            obj.B(2, 2) = 1/(obj.L-obj.M);
            obj.B(3, 3) = 1/(obj.L-obj.M);
            obj.B(4, 4) = 1;
             
            %C
            obj.C(1, 1) = -1/(obj.L-obj.M);
            obj.C(2, 2) = -1/(obj.L-obj.M);
            obj.C(3, 3) = -1/(obj.L-obj.M);
            
            ret_obj = obj;
        end
                            
        function ret_obj = new_dynamics(obj)                        
            %update the time-variant entry of the matrix A
            lambda_div_J = obj.lambda_m / obj.J;
            obj.A(4, 1) = lambda_div_J * obj.f_a;
            obj.A(4, 2) = lambda_div_J * obj.f_b;
            obj.A(4, 3) = lambda_div_J * obj.f_c;
           
            %x_dot = Ax + Bu + Ce
            obj.x_dot = (obj.A*obj.x) + (obj.B*obj.u) + (obj.C*obj.e);
            
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
        
        function ret_obj = update(obj, dt)
            %update the back EMF according to the motor angle
            obj.f_a = obj.back_emf_fa(obj.x(5));
            obj.f_b = obj.back_emf_fb(obj.x(5));
            obj.f_c = obj.back_emf_fc(obj.x(5));
            %            
            omega_times_lambda = obj.x(4) * obj.lambda_m;
            obj.e(1) = omega_times_lambda * obj.f_a;
            obj.e(2) = omega_times_lambda * obj.f_b;
            obj.e(3) = omega_times_lambda * obj.f_c;
            
            obj = obj.new_dynamics();
            obj.x = obj.integrator(obj.x, obj.x_dot, dt);
                        
            %restrict rotor position in +-pi
            obj.x(5) = mod(obj.x(5), 2*pi);
            
            %calculate the motor torque
            sum_of_e_times_i = obj.e(1)*obj.x(1) +  obj.e(2)*obj.x(2) + obj.e(3)*obj.x(3);
            obj.torque = sum_of_e_times_i / obj.x(4);
            
            ret_obj = obj;
        end
        
        function retval = back_emf_fa(obj, theta_r)
            retval = sin(theta_r);
        end
        
        function retval = back_emf_fb(obj, theta_r)
            retval = sin(theta_r - 2/3*pi);

        end
        
        function retval = back_emf_fc(obj, theta_r)
            retval = sin(theta_r + 2/3*pi);
        end
        
        function V_abc = space_vector_to_phase_voltage(obj, x)
            Vdc = obj.v_bldc;
            
            %Vx = (Sa, Sb, Sc)
            switch(x)
                case 0 %V0 = (0, 0, 0)
                    V_abc = [0 0 0];
                    %disp('U0');
                case 1 %V1 = (1, 0, 0)
                    V_abc = [2/3*Vdc -1/3*Vdc -1/3*Vdc];
                    %disp('V1');
                case 2 %V2 = (1, 1, 0)
                    V_abc = [1/3*Vdc 1/3*Vdc -2/3*Vdc];
                    %disp('V2');
                case 3 %V3 = (0, 1, 0)
                    V_abc = [-1/3*Vdc 2/3*Vdc -1/3*Vdc];
                    %disp('V3');
                case 4 %V4 = (0, 1, 1)
                    V_abc = [-2/3*Vdc 1/3*Vdc 1/3*Vdc];
                    %disp('V4');
                case 5 %V5 = (0, 0, 1)
                    V_abc = [-1/3*Vdc -1/3*Vdc 2/3*Vdc];
                    %disp('V5');
                case 6 %V6 = (1, 0, 1)
                    V_abc = [1/3*Vdc -2/3*Vdc 1/3*Vdc];
                    %disp('V6');
                case 7 %V7 = (1, 1 ,1)
                    V_abc = [0 0 0];
                    %disp('V7');
                otherwise
                    V_abc = [0; 0; 0];
                    warning('fatal error: undefined space vector.');
            end
        end
        
        function ret_obj = generate_SVPWM_signal(obj, V_ref, theta)
            %disp(rad2deg(theta));
            
            Vdc = obj.v_bldc;
            a = V_ref / (2/3*Vdc);
            theta_local = mod(theta, pi/3);
            sin60 = sin(pi/3);
            
            Ts = 1 / obj.pwm_freq; %period of the PWM
            T1 = Ts * a * sin(pi/3 - theta_local) / sin60;
            T2 = Ts * a * sin(theta_local) / sin60;           
            T0 = Ts - (T1 + T2);
            obj.T_SVPWM = [T0/2; T1; T2; T0; T2; T1; T0/2];
            
            %disp((T1+T2)/Ts);
            
            %space vectors
            V0 = obj.space_vector_to_phase_voltage(0);
            V1 = obj.space_vector_to_phase_voltage(1);
            V2 = obj.space_vector_to_phase_voltage(2);
            V3 = obj.space_vector_to_phase_voltage(3);
            V4 = obj.space_vector_to_phase_voltage(4);
            V5 = obj.space_vector_to_phase_voltage(5);
            V6 = obj.space_vector_to_phase_voltage(6);
            V7 = obj.space_vector_to_phase_voltage(7);
                       
            %calculate the section number from the rotation angle
            if(theta >= deg2rad(0) && theta < deg2rad(60))
                %section 1: V0-V1-V2-V7-V7-V2-V1-V0
                %disp('section 1');
                obj.u_SVPWM(1, :) = V0;
                obj.u_SVPWM(2, :) = V1;
                obj.u_SVPWM(3, :) = V2;
                obj.u_SVPWM(4, :) = V7;
                obj.u_SVPWM(5, :) = V2;
                obj.u_SVPWM(6, :) = V1;
                obj.u_SVPWM(7, :) = V0;
            elseif(theta >= deg2rad(60) && theta < deg2rad(120))
                %section 2: V0-V3-V2-V7-V7-V2-V3-V0
                %disp('section 2');
                obj.u_SVPWM(1, :) = V0;
                obj.u_SVPWM(2, :) = V3;
                obj.u_SVPWM(3, :) = V2;
                obj.u_SVPWM(4, :) = V7;
                obj.u_SVPWM(5, :) = V2;
                obj.u_SVPWM(6, :) = V3;
                obj.u_SVPWM(7, :) = V0;
            elseif(theta >= deg2rad(120) && theta < deg2rad(180))
                %section 3: V0-V3-V4-V7-V7-V4-V3-V0
                %disp('section 3');
                obj.u_SVPWM(1, :) = V0;
                obj.u_SVPWM(2, :) = V3;
                obj.u_SVPWM(3, :) = V4;
                obj.u_SVPWM(4, :) = V7;
                obj.u_SVPWM(5, :) = V4;
                obj.u_SVPWM(6, :) = V3;
                obj.u_SVPWM(7, :) = V0;
            elseif(theta >= deg2rad(180) && theta < deg2rad(240))
                %section 4: V0-V5-V4-V7-V7-V4-V5-V0
                %disp('section 4');
                obj.u_SVPWM(1, :) = V0;
                obj.u_SVPWM(2, :) = V5;
                obj.u_SVPWM(3, :) = V4;
                obj.u_SVPWM(4, :) = V7;
                obj.u_SVPWM(5, :) = V4;
                obj.u_SVPWM(6, :) = V5;
                obj.u_SVPWM(7, :) = V0;
            elseif(theta >= deg2rad(240) && theta < deg2rad(300))
                %section 5: V0-V5-V6-V7-V7-V6-V5-V0
                %disp('section 5');
                obj.u_SVPWM(1, :) = V0;
                obj.u_SVPWM(2, :) = V5;
                obj.u_SVPWM(3, :) = V6;
                obj.u_SVPWM(4, :) = V7;
                obj.u_SVPWM(5, :) = V6;
                obj.u_SVPWM(6, :) = V5;
                obj.u_SVPWM(7, :) = V0;
            elseif(theta >= deg2rad(300) && theta < deg2rad(360))
                %section 6: V0-V1-V6-V7-V7-V6-V1-V0
                %disp('section 6');
                obj.u_SVPWM(1, :) = V0;
                obj.u_SVPWM(2, :) = V1;
                obj.u_SVPWM(3, :) = V6;
                obj.u_SVPWM(4, :) = V7;
                obj.u_SVPWM(5, :) = V6;
                obj.u_SVPWM(6, :) = V1;
                obj.u_SVPWM(7, :) = V0;
            end
            
            %disp(obj.u_SVPWM);
            ret_obj = obj;
        end
                
        function alpha_beta_gemma = clarke_transform(obj, abc)
            sqrt3_div_2 = sqrt(3)/2;
            
            T_c(1, 1) = 1;
            T_c(1, 2) = -0.5;
            T_c(1, 3) = -0.5;
            T_c(2, 1) = 0;
            T_c(2, 2) = sqrt3_div_2;
            T_c(2, 3) = -sqrt3_div_2;
            T_c(3, 1) = 0.5;
            T_c(3, 2) = 0.5;
            T_c(3, 3) = 0.5;
            
            alpha_beta_gemma = (2/3) * T_c * abc;
        end
        
        function abc = inv_clarke_transform(obj, alpha_beta_gemma)
            sqrt3_div_2 = sqrt(3)/2;
            
            T_c_inv(1, 1) = 1;
            T_c_inv(1, 2) = 0;
            T_c_inv(1, 3) = 1;
            T_c_inv(2, 1) = -0.5;
            T_c_inv(2, 2) = sqrt3_div_2;
            T_c_inv(2, 3) = 1;
            T_c_inv(3, 1) = -0.5;
            T_c_inv(3, 2) = -sqrt3_div_2;
            T_c_inv(3, 3) = 1;
            
            abc = T_c_inv * alpha_beta_gemma;
        end
        
        function dqz = park_transform(obj, alpha_beta_gamma, theta)
            cos_theta = cos(theta);
            sin_theta = sin(theta);
            
            T_p(1, 1) = cos_theta;
            T_p(1, 2) = sin_theta;
            T_p(1, 3) = 0;
            T_p(2, 1) = -sin_theta;
            T_p(2, 2) = cos_theta;
            T_p(2, 3) = 0;
            T_p(3, 1) = 0;
            T_p(3, 2) = 0;
            T_p(3, 3) = 1;
            
            dqz = T_p * alpha_beta_gamma;
        end
        
        function alpha_beta_gamma = inv_park_transform(obj, dqz, theta)
            cos_theta = cos(theta);
            sin_theta = sin(theta);
            
            T_p_inv(1, 1) = cos_theta;
            T_p_inv(1, 2) = -sin_theta;
            T_p_inv(1, 3) = 0;
            T_p_inv(2, 1) = sin_theta;
            T_p_inv(2, 2) = cos_theta;
            T_p_inv(2, 3) = 0;
            T_p_inv(3, 1) = 0;
            T_p_inv(3, 2) = 0;
            T_p_inv(3, 3) = 1;
            
            dqz(3) = 0;
            alpha_beta_gamma = T_p_inv * dqz;
        end
    end
end