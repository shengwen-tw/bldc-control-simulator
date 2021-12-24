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
        
        %MOSFET signals
        S1 = 0;
        S2 = 0;
        S3 = 0;
        S4 = 0;
        S5 = 0;
        S6 = 0;
        
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
        
        function ret_obj = set_mosfet_gate(obj, S1, S2, S3, S4, S5, S6)
            obj.S1 = S1;
            obj.S2 = S2;
            obj.S3 = S3;
            obj.S4 = S4;
            obj.S5 = S5;
            obj.S6 = S6;
            
            %signal = obj.synthesis_gate_signal(S1, S2, S3, S4, S5, S6);
            %disp(signal);
            
            ret_obj = obj;
        end
        
        function ret_obj = gate_signal_to_control_voltage(obj)
            signal = obj.synthesis_gate_signal(obj.S1, obj.S2, obj.S3, obj.S4, obj.S5, obj.S6);
            
            switch signal
                case 9 %i_a(+), i_b(-)
                    obj.u(1) = +obj.v_bldc;
                    obj.u(2) = -obj.v_bldc;
                    obj.u(3) = 0;
                case 33 %i_a(+), i_c(-)
                    obj.u(1) = +obj.v_bldc;
                    obj.u(2) = 0;
                    obj.u(3) = -obj.v_bldc;
                case 36 %i_b(+), i_c(-)
                    obj.u(1) = 0;
                    obj.u(2) = +obj.v_bldc;
                    obj.u(3) = -obj.v_bldc;
                case 6 %i_b(+), i_a(-)
                    obj.u(1) = -obj.v_bldc;
                    obj.u(2) = +obj.v_bldc;
                    obj.u(3) = 0;
                case 18 %i_c(+), i_a(-)
                    obj.u(1) = -obj.v_bldc;
                    obj.u(2) = 0;
                    obj.u(3) = +obj.v_bldc;
                case 24 %i_c(+), i_b(-)
                    obj.u(1) = 0;
                    obj.u(2) = -obj.v_bldc;
                    obj.u(3) = +obj.v_bldc;
                otherwise
                    warning('unexpected gate signal combination.');
            end
            
            ret_obj = obj;
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
            %convert MOSFET control signals to voltage
            %obj = obj.gate_signal_to_control_voltage();
            
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
            %restrict theta_r in [0, 2*pi]
            theta_r = mod(theta_r, 2*pi);
            
            retval = sin(theta_r);
        end
        
        function retval = back_emf_fb(obj, theta_r)
            %phase shifting
            theta_r = theta_r - deg2rad(120);
            
            %restrict theta_r in [0, 2*pi]
            theta_r = mod(theta_r, 2*pi);
            
            retval = back_emf_fa(obj, theta_r);
        end
        
        function retval = back_emf_fc(obj, theta_r)
            %phase shifting
            theta_r = theta_r - deg2rad(240);
            
            %restrict theta_r in [0, 2*pi]
            theta_r = mod(theta_r, 2*pi);
            
            retval = back_emf_fa(obj, theta_r);
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
    end
end